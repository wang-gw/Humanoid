from __future__ import annotations

import argparse
import json
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco
import numpy as np


FOOT_SPECS = {
    "foot_L_1": {"mesh": "foot_L_1", "visual_geom_pos": np.array([-0.067, 0.0215, -0.04])},
    "foot_R_v1_1": {"mesh": "foot_R_v1_1", "visual_geom_pos": np.array([0.087, 0.0215, -0.04])},
}


def mesh_bbox(model: mujoco.MjModel, mesh_name: str) -> tuple[np.ndarray, np.ndarray]:
    mesh_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_MESH, mesh_name)
    if mesh_id < 0:
        raise ValueError(f"Mesh not found: {mesh_name}")
    start = model.mesh_vertadr[mesh_id]
    count = model.mesh_vertnum[mesh_id]
    vertices = np.asarray(model.mesh_vert[start : start + count])
    return vertices.min(axis=0), vertices.max(axis=0)


def find_body(root: ET.Element, name: str) -> ET.Element:
    for body in root.iter("body"):
        if body.attrib.get("name") == name:
            return body
    raise ValueError(f"Body not found: {name}")


def disable_mesh_collisions(root: ET.Element) -> int:
    count = 0
    for geom in root.iter("geom"):
        if geom.attrib.get("type") == "mesh":
            geom.attrib["contype"] = "0"
            geom.attrib["conaffinity"] = "0"
            count += 1
    return count


def add_foot_boxes(root: ET.Element, model: mujoco.MjModel, friction: str) -> list[dict[str, object]]:
    records: list[dict[str, object]] = []
    for body_name, spec in FOOT_SPECS.items():
        body = find_body(root, body_name)
        mesh_min, mesh_max = mesh_bbox(model, str(spec["mesh"]))
        center = np.asarray(spec["visual_geom_pos"], dtype=float) + 0.5 * (mesh_min + mesh_max)
        halfsize = 0.5 * (mesh_max - mesh_min)

        geom_name = f"{body_name}_sole_collision"
        existing = [geom for geom in body.findall("geom") if geom.attrib.get("name") == geom_name]
        for geom in existing:
            body.remove(geom)

        ET.SubElement(
            body,
            "geom",
            {
                "name": geom_name,
                "type": "box",
                "pos": " ".join(f"{value:.8g}" for value in center),
                "size": " ".join(f"{max(value, 0.005):.8g}" for value in halfsize),
                "rgba": "0.1 0.8 0.2 0.35",
                "friction": friction,
                "condim": "3",
            },
        )
        records.append(
            {
                "body": body_name,
                "mesh": spec["mesh"],
                "box_pos": [float(v) for v in center],
                "box_halfsize": [float(max(v, 0.005)) for v in halfsize],
            }
        )
    return records


def main() -> int:
    parser = argparse.ArgumentParser(description="Build a MuJoCo MJCF with visual meshes and simplified foot collisions.")
    parser.add_argument("--source", type=Path, default=Path("envs/robots/urdf_f/URDF_F_mujoco.xml"))
    parser.add_argument("--out", type=Path, default=Path("envs/robots/urdf_f/URDF_F_contact.xml"))
    parser.add_argument("--friction", default="1.0 0.02 0.001")
    parser.add_argument("--doc-dir", type=Path, default=Path("docs/hardware_validation"))
    args = parser.parse_args()

    source = args.source.resolve()
    out = args.out.resolve()
    doc_dir = args.doc_dir.resolve()
    doc_dir.mkdir(parents=True, exist_ok=True)

    model = mujoco.MjModel.from_xml_path(str(source))
    root = ET.parse(source).getroot()
    disabled = disable_mesh_collisions(root)
    foot_boxes = add_foot_boxes(root, model, args.friction)
    ET.indent(root, space="  ")
    ET.ElementTree(root).write(out, encoding="utf-8", xml_declaration=False)

    compiled = mujoco.MjModel.from_xml_path(str(out))
    payload = {
        "source": str(source),
        "out": str(out),
        "disabled_mesh_collision_geoms": disabled,
        "foot_boxes": foot_boxes,
        "compiled": {
            "nq": int(compiled.nq),
            "nv": int(compiled.nv),
            "nu": int(compiled.nu),
            "ngeom": int(compiled.ngeom),
        },
    }
    report_path = doc_dir / "contact_simplification_report.json"
    report_path.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")

    md_path = doc_dir / "05_contact_simplification.md"
    lines = [
        "# Contact Simplification",
        "",
        "## Purpose",
        "",
        "Neutral standing probe에서 mesh collision 접촉이 과도하게 불안정했다. 이 단계에서는 모든 mesh geom을 visual-only로 바꾸고, 좌우 foot body에 단순 box collision을 추가한다.",
        "",
        "## Command",
        "",
        "```bash",
        "python3 scripts/build_contact_simplified_mjcf.py",
        "python3 scripts/check_mujoco_model.py --model envs/robots/urdf_f/URDF_F_contact.xml",
        "python3 scripts/probe_pd_standing.py --model envs/robots/urdf_f/URDF_F_contact.xml --out-dir outputs/analysis/pd_standing_contact",
        "```",
        "",
        "## Outputs",
        "",
        f"- Contact MJCF: `{out}`",
        f"- Report JSON: `{report_path}`",
        "",
        "## Collision Changes",
        "",
        f"- Disabled mesh collision geoms: `{disabled}`",
        "- Added simplified foot box collisions:",
    ]
    for record in foot_boxes:
        lines.append(
            f"  - `{record['body']}` pos=`{record['box_pos']}` halfsize=`{record['box_halfsize']}`"
        )
    lines.extend(
        [
            "",
            "## Interpretation",
            "",
            "이 모델은 collision 안정성 검증용 중간 모델이다. 최종 구조 검증 전에는 발바닥 실제 접촉면, 고무 패드 크기, 마찰 계수, 충격 흡수 구조를 설계값으로 다시 반영해야 한다.",
            "",
        ]
    )
    md_path.write_text("\n".join(lines), encoding="utf-8")

    print(f"Wrote {out}")
    print(f"Wrote {report_path}")
    print(f"Wrote {md_path}")
    print(json.dumps(payload["compiled"], indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
