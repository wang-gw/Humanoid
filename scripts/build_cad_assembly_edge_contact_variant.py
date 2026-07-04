from __future__ import annotations

import argparse
import json
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco
import numpy as np


CAD_ASSEMBLY_POINTS_MM = {
    "left": {
        "body": "foot_L_1",
        "geom": "foot_L_1_sole_collision",
        "points": [
            (41.750, -120.000, 25.000),
            (111.750, -120.000, 25.000),
            (41.750, -120.000, 35.000),
            (111.750, -120.000, 35.000),
        ],
    },
    "right": {
        "body": "foot_R_v1_1",
        "geom": "foot_R_v1_1_sole_collision",
        "points": [
            (-112.250, -120.000, 25.000),
            (-42.250, -120.000, 25.000),
            (-112.250, -120.000, 35.000),
            (-42.250, -120.000, 35.000),
        ],
    },
}


def cad_to_mujoco_m(point_mm: tuple[float, float, float], z_ground_mm: float) -> np.ndarray:
    cad_x, cad_y, cad_z = point_mm
    return np.array([cad_y, cad_x, cad_z - z_ground_mm], dtype=np.float64) / 1000.0


def find_body(root: ET.Element, name: str) -> ET.Element:
    for body in root.iter("body"):
        if body.attrib.get("name") == name:
            return body
    raise ValueError(f"Body not found: {name}")


def remove_existing_contacts(root: ET.Element) -> int:
    removed = 0
    for body in root.iter("body"):
        for geom in list(body.findall("geom")):
            name = geom.attrib.get("name", "")
            if "sole_collision" in name or "sole_pad" in name:
                body.remove(geom)
                removed += 1
    return removed


def fmt(values: np.ndarray | list[float]) -> str:
    return " ".join(f"{float(value):.8g}" for value in values)


def body_pose_qpos0(model_path: Path, body_name: str) -> tuple[np.ndarray, np.ndarray]:
    model = mujoco.MjModel.from_xml_path(str(model_path))
    data = mujoco.MjData(model)
    data.qpos[:] = model.qpos0
    data.qvel[:] = 0.0
    if model.nq >= 7:
        data.qpos[0:3] = np.array([0.0, 0.0, 0.0])
        data.qpos[3:7] = np.array([1.0, 0.0, 0.0, 0.0])
    mujoco.mj_forward(model, data)
    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body_name)
    return np.asarray(data.xpos[body_id]).copy(), np.asarray(data.xmat[body_id]).reshape(3, 3).copy()


def main() -> int:
    parser = argparse.ArgumentParser(description="Build foot contact from user-supplied CAD assembly edge coordinates.")
    parser.add_argument("--source", type=Path, default=Path("envs/robots/urdf_f_link/URDF_F_link_user_size_mass_inertia_direct_nobase_contact.xml"))
    parser.add_argument("--out", type=Path, default=Path("envs/robots/urdf_f_link/URDF_F_link_user_size_mass_inertia_cad_assembly_edge_contact.xml"))
    parser.add_argument("--z-ground-mm", type=float, default=25.0)
    parser.add_argument("--pad-min-halfsize", default="0.006 0.006 0.003")
    parser.add_argument("--friction", default="1.0 0.02 0.001")
    parser.add_argument("--doc-dir", type=Path, default=Path("docs/hardware_validation"))
    args = parser.parse_args()

    source = args.source.resolve()
    out = args.out.resolve()
    doc_dir = args.doc_dir.resolve()
    doc_dir.mkdir(parents=True, exist_ok=True)
    pad_min_halfsize = np.array([float(value) for value in args.pad_min_halfsize.split()], dtype=np.float64)

    tree = ET.parse(source)
    root = tree.getroot()
    root.attrib["model"] = out.stem
    removed = remove_existing_contacts(root)

    records = []
    for side, spec in CAD_ASSEMBLY_POINTS_MM.items():
        points_world = np.array([cad_to_mujoco_m(point, args.z_ground_mm) for point in spec["points"]], dtype=np.float64)
        bottom_points = points_world[np.isclose(points_world[:, 2], points_world[:, 2].min())]
        center_world = bottom_points.mean(axis=0)
        span = np.ptp(bottom_points, axis=0)
        halfsize = np.maximum(span * 0.5, pad_min_halfsize)
        halfsize[2] = pad_min_halfsize[2]
        center_world[2] = halfsize[2]

        body_pos, body_xmat = body_pose_qpos0(source, str(spec["body"]))
        center_body = body_xmat.T @ (center_world - body_pos)

        body = find_body(root, str(spec["body"]))
        ET.SubElement(
            body,
            "geom",
            {
                "name": str(spec["geom"]),
                "type": "box",
                "pos": fmt(center_body),
                "size": fmt(halfsize),
                "rgba": "0.1 0.8 0.2 0.35",
                "friction": args.friction,
                "condim": "3",
            },
        )
        records.append(
            {
                "side": side,
                "body": spec["body"],
                "geom": spec["geom"],
                "cad_points_mm": spec["points"],
                "mujoco_points_world_m": points_world.tolist(),
                "bottom_points_world_m": bottom_points.tolist(),
                "center_world_m": center_world.tolist(),
                "center_body_m": center_body.tolist(),
                "halfsize_m": halfsize.tolist(),
                "body_pos_qpos0": body_pos.tolist(),
            }
        )

    ET.indent(root, space="  ")
    tree.write(out, encoding="utf-8", xml_declaration=False)
    compiled = mujoco.MjModel.from_xml_path(str(out))
    payload = {
        "source": str(source),
        "out": str(out),
        "axis_mapping": "MuJoCo XYZ = [CAD Y, CAD X, CAD Z]",
        "z_ground_mm": args.z_ground_mm,
        "removed_contact_geoms": removed,
        "pad_min_halfsize": pad_min_halfsize.tolist(),
        "friction": args.friction,
        "records": records,
        "compiled": {"nq": int(compiled.nq), "nv": int(compiled.nv), "nu": int(compiled.nu), "njnt": int(compiled.njnt), "ngeom": int(compiled.ngeom)},
    }
    json_path = doc_dir / "cad_assembly_edge_contact_variant_report.json"
    json_path.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(f"Wrote {out}")
    print(f"Wrote {json_path}")
    print(json.dumps(payload["compiled"], indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
