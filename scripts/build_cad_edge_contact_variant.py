from __future__ import annotations

import argparse
import json
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco
import numpy as np


CAD_EDGE_POINTS_MM = {
    "left": {
        "body": "foot_L_1",
        "geom": "foot_L_1_sole_collision",
        "ground_points_cad_world": [(-35.0, -60.0, 0.0), (35.0, -60.0, 0.0)],
        "ground_points_cad_local": [(-111.75, 0.0, -25.0), (-41.75, 0.0, -25.0)],
    },
    "right": {
        "body": "foot_R_v1_1",
        "geom": "foot_R_v1_1_sole_collision",
        "ground_points_cad_world": [(-35.0, -60.0, 0.0), (35.0, -60.0, 0.0)],
        "ground_points_cad_local": [(-42.25, 0.0, 25.0), (-112.25, 0.0, 25.0)],
    },
}


def cad_to_mujoco_m(point_mm: tuple[float, float, float]) -> np.ndarray:
    cad_x, cad_y, cad_z = point_mm
    return np.array([cad_y, cad_x, cad_z], dtype=np.float64) / 1000.0


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


def model_body_pose_at_qpos0(model_path: Path, body_name: str) -> tuple[np.ndarray, np.ndarray]:
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


def contact_from_cad_local(side: str, pad_halfsize: np.ndarray) -> dict[str, object]:
    spec = CAD_EDGE_POINTS_MM[side]
    points = np.array([cad_to_mujoco_m(point) for point in spec["ground_points_cad_local"]], dtype=np.float64)
    edge_center = points.mean(axis=0)
    span = np.ptp(points, axis=0)
    center = edge_center.copy()
    center[2] += pad_halfsize[2]
    halfsize = np.maximum(span * 0.5, pad_halfsize)
    halfsize[0] = pad_halfsize[0]
    halfsize[2] = pad_halfsize[2]
    return {"center": center, "halfsize": halfsize, "edge_points_mujoco_local": points.tolist()}


def contact_from_cad_world_fit(source: Path, side: str, pad_halfsize: np.ndarray) -> dict[str, object]:
    spec = CAD_EDGE_POINTS_MM[side]
    points_world = np.array([cad_to_mujoco_m(point) for point in spec["ground_points_cad_world"]], dtype=np.float64)
    edge_center_world = points_world.mean(axis=0)
    edge_center_world[2] += pad_halfsize[2]
    body_pos, body_xmat = model_body_pose_at_qpos0(source, str(spec["body"]))
    center = body_xmat.T @ (edge_center_world - body_pos)
    span = np.ptp(points_world, axis=0)
    halfsize = np.maximum(span * 0.5, pad_halfsize)
    halfsize[0] = pad_halfsize[0]
    halfsize[2] = pad_halfsize[2]
    return {
        "center": center,
        "halfsize": halfsize,
        "edge_points_mujoco_world": points_world.tolist(),
        "body_pos_qpos0": body_pos.tolist(),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description="Build CAD edge-derived foot contact MJCF variants.")
    parser.add_argument("--source", type=Path, default=Path("envs/robots/urdf_f_link/URDF_F_link_user_size_mass_inertia_direct_nobase_contact.xml"))
    parser.add_argument("--mode", choices=["cad_local", "cad_world_fit"], default="cad_world_fit")
    parser.add_argument("--out", type=Path, default=Path("envs/robots/urdf_f_link/URDF_F_link_user_size_mass_inertia_cad_edge_contact.xml"))
    parser.add_argument("--pad-halfsize", default="0.006 0.035 0.003")
    parser.add_argument("--friction", default="1.0 0.02 0.001")
    parser.add_argument("--doc-dir", type=Path, default=Path("docs/hardware_validation"))
    args = parser.parse_args()

    source = args.source.resolve()
    out = args.out.resolve()
    doc_dir = args.doc_dir.resolve()
    doc_dir.mkdir(parents=True, exist_ok=True)
    pad_halfsize = np.array([float(value) for value in args.pad_halfsize.split()], dtype=np.float64)
    if pad_halfsize.shape != (3,):
        raise ValueError("--pad-halfsize must contain exactly three floats")

    tree = ET.parse(source)
    root = tree.getroot()
    root.attrib["model"] = out.stem
    removed = remove_existing_contacts(root)
    records = []
    for side, spec in CAD_EDGE_POINTS_MM.items():
        contact = contact_from_cad_local(side, pad_halfsize) if args.mode == "cad_local" else contact_from_cad_world_fit(source, side, pad_halfsize)
        body = find_body(root, str(spec["body"]))
        ET.SubElement(
            body,
            "geom",
            {
                "name": str(spec["geom"]),
                "type": "box",
                "pos": fmt(contact["center"]),
                "size": fmt(contact["halfsize"]),
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
                "center_xyz_in_body": [float(v) for v in contact["center"]],
                "halfsize_xyz_m": [float(v) for v in contact["halfsize"]],
                **{k: v for k, v in contact.items() if k not in {"center", "halfsize"}},
            }
        )

    ET.indent(root, space="  ")
    tree.write(out, encoding="utf-8", xml_declaration=False)
    compiled = mujoco.MjModel.from_xml_path(str(out))
    payload = {
        "source": str(source),
        "out": str(out),
        "mode": args.mode,
        "axis_mapping": "MuJoCo XYZ = [CAD Y, CAD X, CAD Z]",
        "removed_contact_geoms": removed,
        "pad_halfsize": [float(v) for v in pad_halfsize],
        "friction": args.friction,
        "records": records,
        "compiled": {"nq": int(compiled.nq), "nv": int(compiled.nv), "nu": int(compiled.nu), "njnt": int(compiled.njnt), "ngeom": int(compiled.ngeom)},
    }
    json_path = doc_dir / f"cad_edge_contact_variant_{args.mode}_report.json"
    json_path.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(f"Wrote {out}")
    print(f"Wrote {json_path}")
    print(json.dumps(payload["compiled"], indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
