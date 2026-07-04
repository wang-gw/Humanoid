from __future__ import annotations

import argparse
import json
import math
import shutil
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco
import numpy as np

from audit_step_assembly_transforms import parse_step, stl_name_for_occurrence


def normalize(vector: np.ndarray) -> np.ndarray:
    norm = float(np.linalg.norm(vector))
    if norm < 1e-12:
        raise ValueError(f"Cannot normalize near-zero vector: {vector}")
    return vector / norm


def placement_rotation(axis: list[float], ref_direction: list[float]) -> np.ndarray:
    z_axis = normalize(np.array(axis, dtype=np.float64))
    x_hint = normalize(np.array(ref_direction, dtype=np.float64))
    x_axis = normalize(x_hint - np.dot(x_hint, z_axis) * z_axis)
    y_axis = normalize(np.cross(z_axis, x_axis))
    return np.column_stack([x_axis, y_axis, z_axis])


def quat_from_matrix(matrix: np.ndarray) -> np.ndarray:
    trace = float(np.trace(matrix))
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * s
        qx = (matrix[2, 1] - matrix[1, 2]) / s
        qy = (matrix[0, 2] - matrix[2, 0]) / s
        qz = (matrix[1, 0] - matrix[0, 1]) / s
    else:
        idx = int(np.argmax(np.diag(matrix)))
        if idx == 0:
            s = math.sqrt(1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2]) * 2.0
            qw = (matrix[2, 1] - matrix[1, 2]) / s
            qx = 0.25 * s
            qy = (matrix[0, 1] + matrix[1, 0]) / s
            qz = (matrix[0, 2] + matrix[2, 0]) / s
        elif idx == 1:
            s = math.sqrt(1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2]) * 2.0
            qw = (matrix[0, 2] - matrix[2, 0]) / s
            qx = (matrix[0, 1] + matrix[1, 0]) / s
            qy = 0.25 * s
            qz = (matrix[1, 2] + matrix[2, 1]) / s
        else:
            s = math.sqrt(1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1]) * 2.0
            qw = (matrix[1, 0] - matrix[0, 1]) / s
            qx = (matrix[0, 2] + matrix[2, 0]) / s
            qy = (matrix[1, 2] + matrix[2, 1]) / s
            qz = 0.25 * s
    quat = np.array([qw, qx, qy, qz], dtype=np.float64)
    return normalize(quat)


def fmt(values: np.ndarray | list[float]) -> str:
    return " ".join(f"{float(value):.9g}" for value in values)


def main() -> int:
    parser = argparse.ArgumentParser(description="Build a visual-only MJCF directly from STEP assembly occurrence transforms.")
    parser.add_argument("--step", type=Path, default=Path("URDF_F_.step"))
    parser.add_argument("--link-dir", type=Path, default=Path("link"))
    parser.add_argument("--out-dir", type=Path, default=Path("envs/robots/urdf_f_step_visual"))
    parser.add_argument("--out-name", default="URDF_F_step_visual_assembly.xml")
    parser.add_argument("--doc-dir", type=Path, default=Path("docs/hardware_validation"))
    args = parser.parse_args()

    step_path = args.step.resolve()
    link_dir = args.link_dir.resolve()
    out_dir = args.out_dir.resolve()
    doc_dir = args.doc_dir.resolve()
    out_dir.mkdir(parents=True, exist_ok=True)
    doc_dir.mkdir(parents=True, exist_ok=True)

    parsed = parse_step(step_path.read_text(encoding="utf-8", errors="replace"))
    occurrence_ids = sorted(parsed["occurrences"])
    transform_ids = sorted(parsed["transforms"])
    if len(occurrence_ids) != len(transform_ids):
        raise ValueError(f"Occurrence/transform count mismatch: {len(occurrence_ids)} vs {len(transform_ids)}")

    root = ET.Element("mujoco", {"model": "URDF_F_step_visual_assembly"})
    ET.SubElement(root, "compiler", {"angle": "radian"})
    asset = ET.SubElement(root, "asset")
    ET.SubElement(asset, "texture", {"name": "grid", "type": "2d", "builtin": "checker", "width": "512", "height": "512", "rgb1": ".18 .18 .18", "rgb2": ".24 .24 .24"})
    ET.SubElement(asset, "material", {"name": "grid", "texture": "grid", "texrepeat": "4 4", "reflectance": "0.05"})

    records: list[dict[str, object]] = []
    for occurrence_id, transform_id in zip(occurrence_ids, transform_ids):
        occurrence = parsed["occurrences"][occurrence_id]
        name = str(occurrence["name"])
        stl_name = stl_name_for_occurrence(name)
        source = link_dir / stl_name
        if not source.exists():
            raise FileNotFoundError(source)
        mesh_name = name.replace(" ", "_").replace("(", "").replace(")", "")
        dest_name = stl_name.replace(" (1)", "_R")
        shutil.copy2(source, out_dir / dest_name)
        ET.SubElement(asset, "mesh", {"name": mesh_name, "file": dest_name, "scale": "0.001 0.001 0.001"})
        records.append(
            {
                "occurrence_id": occurrence_id,
                "transform_id": transform_id,
                "name": name,
                "mesh_name": mesh_name,
                "source_stl": str(source),
                "copied_stl": str(out_dir / dest_name),
            }
        )

    worldbody = ET.SubElement(root, "worldbody")
    ET.SubElement(worldbody, "light", {"pos": "0 -1 1.2", "dir": "0 1 -1", "diffuse": "0.8 0.8 0.8"})
    ET.SubElement(worldbody, "geom", {"name": "floor", "type": "plane", "size": "1 1 0.02", "material": "grid"})

    colors = [
        "0.75 0.75 0.78 1",
        "0.30 0.55 0.85 1",
        "0.85 0.45 0.25 1",
        "0.35 0.70 0.45 1",
        "0.80 0.65 0.30 1",
    ]
    for idx, (occurrence_id, transform_id) in enumerate(zip(occurrence_ids, transform_ids)):
        occurrence = parsed["occurrences"][occurrence_id]
        transform = parsed["transforms"][transform_id]
        target = parsed["placements"][transform["target_placement"]]
        origin_mm = np.array(target["origin_mm"], dtype=np.float64)
        rotation = placement_rotation(target["axis"], target["ref_direction"])
        quat = quat_from_matrix(rotation)
        mesh_name = str(occurrence["name"]).replace(" ", "_").replace("(", "").replace(")", "")
        body = ET.SubElement(
            worldbody,
            "body",
            {
                "name": f"step_{mesh_name}",
                "pos": fmt(origin_mm / 1000.0),
                "quat": fmt(quat),
            },
        )
        ET.SubElement(
            body,
            "geom",
            {
                "name": f"vis_{mesh_name}",
                "type": "mesh",
                "mesh": mesh_name,
                "rgba": colors[idx % len(colors)],
                "contype": "0",
                "conaffinity": "0",
            },
        )
        records[idx].update(
            {
                "target_origin_mm": [float(v) for v in origin_mm],
                "target_axis": target["axis"],
                "target_ref_direction": target["ref_direction"],
                "mjcf_pos_m": [float(v) for v in origin_mm / 1000.0],
                "mjcf_quat": [float(v) for v in quat],
            }
        )

    ET.indent(root, space="  ")
    out_xml = out_dir / args.out_name
    ET.ElementTree(root).write(out_xml, encoding="utf-8", xml_declaration=False)
    model = mujoco.MjModel.from_xml_path(str(out_xml))

    payload = {
        "step": str(step_path),
        "link_dir": str(link_dir),
        "out_model": str(out_xml),
        "purpose": "visual_only_step_assembly_check",
        "compiled": {"nq": int(model.nq), "nv": int(model.nv), "nbody": int(model.nbody), "ngeom": int(model.ngeom), "nmesh": int(model.nmesh)},
        "records": records,
    }
    report_path = doc_dir / "step_visual_assembly_report.json"
    report_path.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")

    print(f"Wrote {out_xml}")
    print(f"Wrote {report_path}")
    print(json.dumps(payload["compiled"], indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
