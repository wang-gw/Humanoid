from __future__ import annotations

import argparse
import json
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco


def fmt(values: list[float]) -> str:
    return " ".join(f"{float(value):.8g}" for value in values)


def find_body(root: ET.Element, name: str) -> ET.Element:
    for body in root.iter("body"):
        if body.attrib.get("name") == name:
            return body
    raise ValueError(f"Body not found: {name}")


def disable_non_floor_contacts(root: ET.Element) -> int:
    count = 0
    for geom in root.iter("geom"):
        if geom.attrib.get("name") == "floor":
            continue
        geom.attrib["contype"] = "0"
        geom.attrib["conaffinity"] = "0"
        count += 1
    return count


def remove_sole_contacts(root: ET.Element) -> int:
    removed = 0
    for body in root.iter("body"):
        for geom in list(body.findall("geom")):
            if "sole_collision" in geom.attrib.get("name", ""):
                body.remove(geom)
                removed += 1
    return removed


def add_sole(root: ET.Element, key: str, spec: dict[str, object], friction: str) -> dict[str, object]:
    body_name = str(spec["body"])
    body = find_body(root, body_name)
    geom_name = f"{body_name}_sole_collision"
    center = [float(value) for value in spec["center_xyz_in_body"]]
    halfsize = [float(value) for value in spec["halfsize_xyz_m"]]
    ET.SubElement(
        body,
        "geom",
        {
            "name": geom_name,
            "type": str(spec.get("contact_type", "box")),
            "pos": fmt(center),
            "size": fmt(halfsize),
            "rgba": "0.1 0.8 0.2 0.35",
            "friction": friction,
            "condim": "3",
        },
    )
    return {
        "side": key,
        "body": body_name,
        "geom": geom_name,
        "center_xyz_in_body": center,
        "halfsize_xyz_m": halfsize,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description="Apply a foot contact JSON config to a new MJCF variant.")
    parser.add_argument("--source", type=Path, default=Path("envs/robots/urdf_f_link/URDF_F_link_named.xml"))
    parser.add_argument("--config", type=Path, default=Path("configs/prefilled/foot_contact_prefilled_from_step_body_centered.json"))
    parser.add_argument("--out", type=Path, default=Path("envs/robots/urdf_f_link/URDF_F_link_step_contact.xml"))
    parser.add_argument("--doc-dir", type=Path, default=Path("docs/hardware_validation"))
    args = parser.parse_args()

    source = args.source.resolve()
    config_path = args.config.resolve()
    out = args.out.resolve()
    doc_dir = args.doc_dir.resolve()
    doc_dir.mkdir(parents=True, exist_ok=True)

    config = json.loads(config_path.read_text(encoding="utf-8"))
    friction_config = config.get("friction", {})
    friction = " ".join(
        str(float(friction_config.get(key, default)))
        for key, default in (("sliding", 1.0), ("torsional", 0.02), ("rolling", 0.001))
    )

    tree = ET.parse(source)
    root = tree.getroot()
    root.attrib["model"] = out.stem
    disabled = disable_non_floor_contacts(root)
    removed = remove_sole_contacts(root)
    records = [
        add_sole(root, "left", config["left_sole"], friction),
        add_sole(root, "right", config["right_sole"], friction),
    ]

    ET.indent(root, space="  ")
    tree.write(out, encoding="utf-8", xml_declaration=False)
    compiled = mujoco.MjModel.from_xml_path(str(out))

    payload = {
        "source": str(source),
        "config": str(config_path),
        "out": str(out),
        "disabled_non_floor_geoms": disabled,
        "removed_sole_geoms": removed,
        "friction": friction,
        "contacts": records,
        "compiled": {
            "nq": int(compiled.nq),
            "nv": int(compiled.nv),
            "nu": int(compiled.nu),
            "njnt": int(compiled.njnt),
            "ngeom": int(compiled.ngeom),
        },
    }

    report_path = doc_dir / "foot_contact_config_application_report.json"
    report_path.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")

    md_path = doc_dir / "35_apply_step_contact_config.md"
    lines = [
        "# STEP 기반 foot contact config 적용",
        "",
        "## 목적",
        "",
        "foot contact 값을 코드에 하드코딩하지 않고 JSON config에서 읽어 새 MJCF variant를 생성한다. 앞으로 Fusion 360에서 확정한 발바닥 값을 받으면 같은 스크립트로 새 모델을 만들 수 있다.",
        "",
        "## 실행 명령",
        "",
        "```bash",
        "python3 scripts/apply_foot_contact_config.py --source envs/robots/urdf_f_link/URDF_F_link_named.xml --config configs/prefilled/foot_contact_prefilled_from_step_body_centered.json --out envs/robots/urdf_f_link/URDF_F_link_step_contact.xml",
        "```",
        "",
        "## 산출물",
        "",
        f"- 새 모델: `{out}`",
        f"- 사용 config: `{config_path}`",
        f"- 리포트 JSON: `{report_path}`",
        "",
        "## 적용한 contact",
        "",
        "| side | body | geom | center xyz in body | halfsize xyz |",
        "| --- | --- | --- | --- | --- |",
    ]
    for record in records:
        lines.append(
            f"| `{record['side']}` | `{record['body']}` | `{record['geom']}` | `{record['center_xyz_in_body']}` | `{record['halfsize_xyz_m']}` |"
        )
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
            "## 판단",
            "",
            "이 모델은 STEP 기반 초안 contact를 적용한 반복 검증용 variant다. 아직 최종 설계 모델은 아니며, Fusion 360에서 실제 발바닥 중심/크기와 standing pose가 확인되면 config 값을 교체해 다시 생성해야 한다.",
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
