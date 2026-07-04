from __future__ import annotations

import argparse
import json
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco


FOOT_CONTACTS = [
    {
        "body": "foot_L_1",
        "name": "foot_L_1_sole_collision",
        "pos": "0 0 -0.035",
        "size": "0.035 0.06 0.005",
    },
    {
        "body": "foot_R_v1_1",
        "name": "foot_R_v1_1_sole_collision",
        "pos": "0 0 -0.035",
        "size": "0.035 0.06 0.005",
    },
]


def find_body(root: ET.Element, name: str) -> ET.Element:
    for body in root.iter("body"):
        if body.attrib.get("name") == name:
            return body
    raise ValueError(f"Body not found: {name}")


def disable_contact_geoms(root: ET.Element) -> int:
    count = 0
    for geom in root.iter("geom"):
        if geom.attrib.get("name") == "floor":
            continue
        geom.attrib["contype"] = "0"
        geom.attrib["conaffinity"] = "0"
        count += 1
    return count


def remove_existing_sole_geoms(root: ET.Element) -> int:
    removed = 0
    for body in root.iter("body"):
        for geom in list(body.findall("geom")):
            name = geom.attrib.get("name", "")
            if "sole_collision" in name:
                body.remove(geom)
                removed += 1
    return removed


def add_contacts(root: ET.Element, friction: str) -> list[dict[str, str]]:
    records = []
    for contact in FOOT_CONTACTS:
        body = find_body(root, contact["body"])
        ET.SubElement(
            body,
            "geom",
            {
                "name": contact["name"],
                "type": "box",
                "pos": contact["pos"],
                "size": contact["size"],
                "rgba": "0.1 0.8 0.2 0.35",
                "friction": friction,
                "condim": "3",
            },
        )
        records.append(dict(contact))
    return records


def main() -> int:
    parser = argparse.ArgumentParser(description="Build a diagnostic model with foot contacts centered on foot bodies.")
    parser.add_argument("--source", type=Path, default=Path("envs/robots/urdf_f_link/URDF_F_link_named.xml"))
    parser.add_argument("--out", type=Path, default=Path("envs/robots/urdf_f_link/URDF_F_link_body_contact.xml"))
    parser.add_argument("--friction", default="1.0 0.02 0.001")
    parser.add_argument("--doc-dir", type=Path, default=Path("docs/hardware_validation"))
    args = parser.parse_args()

    source = args.source.resolve()
    out = args.out.resolve()
    doc_dir = args.doc_dir.resolve()
    doc_dir.mkdir(parents=True, exist_ok=True)

    tree = ET.parse(source)
    root = tree.getroot()
    root.attrib["model"] = "URDF_F_link_body_contact"
    disabled = disable_contact_geoms(root)
    removed = remove_existing_sole_geoms(root)
    contacts = add_contacts(root, args.friction)

    ET.indent(root, space="  ")
    tree.write(out, encoding="utf-8", xml_declaration=False)

    compiled = mujoco.MjModel.from_xml_path(str(out))
    payload = {
        "source": str(source),
        "out": str(out),
        "purpose": "diagnostic_only_not_final_contact_geometry",
        "disabled_non_floor_geoms": disabled,
        "removed_existing_sole_geoms": removed,
        "contacts": contacts,
        "compiled": {
            "nq": int(compiled.nq),
            "nv": int(compiled.nv),
            "nu": int(compiled.nu),
            "njnt": int(compiled.njnt),
            "ngeom": int(compiled.ngeom),
        },
    }

    json_path = doc_dir / "body_centered_foot_contact_report.json"
    json_path.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")

    md_path = doc_dir / "29_body_centered_foot_contact_variant.md"
    lines = [
        "# Body-centered foot contact 진단 모델",
        "",
        "## 목적",
        "",
        "28번 감사에서 좌우 foot body는 떨어져 있지만 sole collision이 같은 world 위치로 겹치는 문제가 확인됐다. 이 모델은 실제 설계 확정 모델이 아니라, foot body 중심에 발바닥 box를 두면 support polygon과 standing probe가 어떻게 달라지는지 확인하기 위한 진단용 variant다.",
        "",
        "## 실행 명령",
        "",
        "```bash",
        "python3 scripts/build_body_centered_foot_contact.py",
        "```",
        "",
        "## 산출물",
        "",
        f"- 진단 모델: `{out}`",
        f"- JSON: `{json_path}`",
        "",
        "## Contact 설정",
        "",
        "- 기존 non-floor geom은 모두 visual-only/contact-off로 둔다.",
        "- 기존 sole collision geom은 제거한다.",
        "- 좌우 foot body에 같은 local contact box를 추가한다.",
        "",
        "| body | geom | local pos | halfsize |",
        "| --- | --- | --- | --- |",
    ]
    for contact in contacts:
        lines.append(f"| `{contact['body']}` | `{contact['name']}` | `{contact['pos']}` | `{contact['size']}` |")
    lines.extend(
        [
            "",
            "## 컴파일 확인",
            "",
            f"- nq: `{compiled.nq}`",
            f"- nv: `{compiled.nv}`",
            f"- nu: `{compiled.nu}`",
            f"- joint 수: `{compiled.njnt}`",
            f"- geom 수: `{compiled.ngeom}`",
            "",
            "## 주의",
            "",
            "이 contact 위치는 CAD 확정값이 아니다. 실제 발바닥 패드/바닥 접촉면 치수와 foot link origin이 확인되면 새 설계 기준 모델을 따로 만들어야 한다.",
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
