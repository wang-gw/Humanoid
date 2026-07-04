from __future__ import annotations

import argparse
import json
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco
import numpy as np

from audit_step_assembly_transforms import parse_step, stl_name_for_occurrence
from build_step_visual_assembly import placement_rotation, quat_from_matrix


FILE_ALIASES = {
    "AK45-36_trR.stl": "AK45-36_trL (1).stl",
}


def fmt(values: np.ndarray | list[float]) -> str:
    return " ".join(f"{float(value):.9g}" for value in values)


def mesh_geom_elements(root: ET.Element) -> list[ET.Element]:
    return [geom for geom in root.findall(".//geom") if geom.attrib.get("type") == "mesh"]


def step_targets(step_path: Path) -> dict[str, dict[str, object]]:
    parsed = parse_step(step_path.read_text(encoding="utf-8", errors="replace"))
    occurrence_ids = sorted(parsed["occurrences"])
    transform_ids = sorted(parsed["transforms"])
    if len(occurrence_ids) != len(transform_ids):
        raise ValueError(f"Occurrence/transform count mismatch: {len(occurrence_ids)} vs {len(transform_ids)}")

    targets: dict[str, dict[str, object]] = {}
    for occurrence_id, transform_id in zip(occurrence_ids, transform_ids):
        occurrence = parsed["occurrences"][occurrence_id]
        transform = parsed["transforms"][transform_id]
        target = parsed["placements"][transform["target_placement"]]
        stl_name = stl_name_for_occurrence(str(occurrence["name"]))
        rotation = placement_rotation(target["axis"], target["ref_direction"])
        targets[stl_name] = {
            "name": occurrence["name"],
            "position_m": np.array(target["origin_mm"], dtype=np.float64) / 1000.0,
            "rotation": rotation,
            "quat": quat_from_matrix(rotation),
        }
    return targets


def main() -> int:
    parser = argparse.ArgumentParser(description="Rewrite visual mesh geom offsets using STEP assembly world poses.")
    parser.add_argument("--source", type=Path, default=Path("envs/robots/urdf_f_link/URDF_F_link_user_size_mass_inertia_user_pad_toeheel_z_grounded.xml"))
    parser.add_argument("--step", type=Path, default=Path("URDF_F_.step"))
    parser.add_argument("--out", type=Path, default=Path("envs/robots/urdf_f_link/URDF_F_link_user_size_mass_inertia_user_pad_toeheel_z_grounded_step_visual.xml"))
    parser.add_argument("--doc-dir", type=Path, default=Path("docs/hardware_validation"))
    args = parser.parse_args()

    source = args.source.resolve()
    out = args.out.resolve()
    doc_dir = args.doc_dir.resolve()
    doc_dir.mkdir(parents=True, exist_ok=True)

    tree = ET.parse(source)
    root = tree.getroot()
    root.attrib["model"] = out.stem
    mesh_files = {mesh.attrib["name"]: mesh.attrib.get("file", "") for mesh in root.findall("./asset/mesh")}
    xml_mesh_geoms = mesh_geom_elements(root)

    model = mujoco.MjModel.from_xml_path(str(source))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    model_mesh_geom_ids = [geom_id for geom_id in range(model.ngeom) if model.geom_type[geom_id] == mujoco.mjtGeom.mjGEOM_MESH]
    if len(xml_mesh_geoms) != len(model_mesh_geom_ids):
        raise ValueError(f"XML/model mesh geom mismatch: {len(xml_mesh_geoms)} vs {len(model_mesh_geom_ids)}")

    targets = step_targets(args.step.resolve())
    records = []
    for xml_geom, geom_id in zip(xml_mesh_geoms, model_mesh_geom_ids):
        mesh_name = xml_geom.attrib["mesh"]
        mesh_file = Path(mesh_files[mesh_name]).name
        target_key = FILE_ALIASES.get(mesh_file, mesh_file)
        target = targets.get(target_key)
        if target is None:
            records.append({"mesh": mesh_name, "mesh_file": mesh_file, "status": "missing_step_target"})
            continue

        body_id = int(model.geom_bodyid[geom_id])
        body_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, body_id) or f"body_{body_id}"
        body_pos = np.array(data.xpos[body_id], dtype=np.float64)
        body_rot = np.array(data.xmat[body_id], dtype=np.float64).reshape(3, 3)
        target_pos = target["position_m"]
        target_rot = target["rotation"]
        local_pos = body_rot.T @ (target_pos - body_pos)
        local_rot = body_rot.T @ target_rot
        local_quat = quat_from_matrix(local_rot)

        old_pos = xml_geom.attrib.get("pos", "0 0 0")
        old_quat = xml_geom.attrib.get("quat", "1 0 0 0")
        xml_geom.attrib["pos"] = fmt(local_pos)
        xml_geom.attrib["quat"] = fmt(local_quat)
        records.append(
            {
                "mesh": mesh_name,
                "mesh_file": mesh_file,
                "step_key": target_key,
                "step_name": target["name"],
                "body": body_name,
                "old_pos": old_pos,
                "old_quat": old_quat,
                "new_pos": [float(v) for v in local_pos],
                "new_quat": [float(v) for v in local_quat],
                "target_world_pos": [float(v) for v in target_pos],
                "status": "updated",
            }
        )

    ET.indent(root, space="  ")
    out.parent.mkdir(parents=True, exist_ok=True)
    tree.write(out, encoding="utf-8", xml_declaration=False)
    compiled = mujoco.MjModel.from_xml_path(str(out))
    payload = {
        "source": str(source),
        "step": str(args.step.resolve()),
        "out": str(out),
        "compiled": {"nq": int(compiled.nq), "nv": int(compiled.nv), "nu": int(compiled.nu), "ngeom": int(compiled.ngeom)},
        "records": records,
    }
    report_path = doc_dir / "step_visual_offsets_report.json"
    report_path.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(f"Wrote {out}")
    print(f"Wrote {report_path}")
    print(json.dumps(payload["compiled"], indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
