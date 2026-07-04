from __future__ import annotations

import argparse
import json
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco


FOOT_BODIES = {
    "left": ("foot_L_1", "foot_L_1_sole"),
    "right": ("foot_R_v1_1", "foot_R_v1_1_sole"),
}


def find_body(root: ET.Element, name: str) -> ET.Element:
    for body in root.iter("body"):
        if body.attrib.get("name") == name:
            return body
    raise ValueError(f"Body not found: {name}")


def remove_existing_sole_geoms(root: ET.Element) -> int:
    removed = 0
    for body in root.iter("body"):
        for geom in list(body.findall("geom")):
            name = geom.attrib.get("name", "")
            if "sole_collision" in name or "sole_pad" in name:
                body.remove(geom)
                removed += 1
    return removed


def add_corner_pads(root: ET.Element, pad_halfsize: tuple[float, float, float], friction: str) -> list[dict[str, str]]:
    records = []
    hx, hy, hz = pad_halfsize
    # Original sole box bottom was local z=-0.04. Keep the same bottom with thinner pads.
    z = -0.04 + hz
    positions = {
        "front_outer": (0.02, 0.035, z),
        "front_inner": (0.02, -0.035, z),
        "rear_outer": (-0.02, 0.035, z),
        "rear_inner": (-0.02, -0.035, z),
    }
    for side, (body_name, prefix) in FOOT_BODIES.items():
        body = find_body(root, body_name)
        for label, pos in positions.items():
            name = f"{prefix}_pad_{label}"
            ET.SubElement(
                body,
                "geom",
                {
                    "name": name,
                    "type": "box",
                    "pos": " ".join(f"{v:g}" for v in pos),
                    "size": f"{hx:g} {hy:g} {hz:g}",
                    "rgba": "0.1 0.8 0.2 0.35",
                    "friction": friction,
                    "condim": "3",
                },
            )
            records.append({"side": side, "body": body_name, "geom": name, "pos": " ".join(f"{v:g}" for v in pos), "size": f"{hx:g} {hy:g} {hz:g}"})
    return records


def main() -> int:
    parser = argparse.ArgumentParser(description="Create a foot contact variant with four corner pads per foot.")
    parser.add_argument("--source", type=Path, default=Path("envs/robots/urdf_f_link/URDF_F_link_user_size_mass_inertia_direct_nobase_contact.xml"))
    parser.add_argument("--out", type=Path, default=Path("envs/robots/urdf_f_link/URDF_F_link_user_size_mass_inertia_corner_contact.xml"))
    parser.add_argument("--pad-halfsize", default="0.015 0.02 0.006")
    parser.add_argument("--friction", default="1.0 0.02 0.001")
    parser.add_argument("--doc-dir", type=Path, default=Path("docs/hardware_validation"))
    args = parser.parse_args()

    source = args.source.resolve()
    out = args.out.resolve()
    doc_dir = args.doc_dir.resolve()
    doc_dir.mkdir(parents=True, exist_ok=True)
    pad_halfsize = tuple(float(v) for v in args.pad_halfsize.split())
    if len(pad_halfsize) != 3:
        raise ValueError("--pad-halfsize must have three values")

    tree = ET.parse(source)
    root = tree.getroot()
    root.attrib["model"] = out.stem
    removed = remove_existing_sole_geoms(root)
    records = add_corner_pads(root, pad_halfsize, args.friction)
    ET.indent(root, space="  ")
    tree.write(out, encoding="utf-8", xml_declaration=False)
    compiled = mujoco.MjModel.from_xml_path(str(out))

    payload = {
        "source": str(source),
        "out": str(out),
        "removed_sole_geoms": removed,
        "pad_halfsize": list(pad_halfsize),
        "friction": args.friction,
        "records": records,
        "compiled": {"nq": int(compiled.nq), "nv": int(compiled.nv), "nu": int(compiled.nu), "njnt": int(compiled.njnt), "ngeom": int(compiled.ngeom)},
    }
    json_path = doc_dir / "corner_foot_contact_variant_report.json"
    json_path.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")

    md_path = doc_dir / "58_corner_foot_contact_variant.md"
    lines = [
        "# Corner Foot Contact Variant",
        "",
        "## 목적",
        "",
        "단일 sole box가 roll 방향 접촉 지지 모멘트를 충분히 만들지 못하는지 확인하기 위해, 각 발의 네 모서리에 작은 contact pad를 둔 실험용 MJCF variant를 만든다.",
        "",
        "## 실행 명령",
        "",
        "```bash",
        "python3 scripts/build_corner_foot_contact_variant.py",
        "```",
        "",
        "## 산출물",
        "",
        f"- 새 모델: `{out}`",
        f"- 리포트 JSON: `{json_path}`",
        "",
        "## Contact Pad",
        "",
        f"- pad halfsize: `{pad_halfsize}`",
        "- original sole bottom local z: `-0.04 m` 유지",
        f"- friction: `{args.friction}`",
        "",
        "| side | body | geom | local pos | halfsize |",
        "| --- | --- | --- | --- | --- |",
    ]
    for record in records:
        lines.append(f"| `{record['side']}` | `{record['body']}` | `{record['geom']}` | `{record['pos']}` | `{record['size']}` |")
    lines.extend(
        [
            "",
            "## Compile 결과",
            "",
            f"- nq: `{compiled.nq}`",
            f"- nv: `{compiled.nv}`",
            f"- nu: `{compiled.nu}`",
            f"- ngeom: `{compiled.ngeom}`",
            "",
            "## 판단",
            "",
            "이 모델은 최종 contact 모델이 아니라 roll 방향 원인 분리용 실험 모델이다. 같은 pose에서 roll impulse response를 비교해 contact 유지가 개선되는지 확인한다.",
            "",
        ]
    )
    md_path.write_text("\n".join(lines), encoding="utf-8")

    print(f"Wrote {out}")
    print(f"Wrote {json_path}")
    print(f"Wrote {md_path}")
    print(json.dumps(payload["compiled"], indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
