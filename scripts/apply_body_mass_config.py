from __future__ import annotations

import argparse
import csv
import json
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco


def parse_float_triplet(text: str) -> list[float]:
    return [float(part) for part in text.split()]


def fmt(values: list[float]) -> str:
    return " ".join(f"{value:.8g}" for value in values)


def read_mass_config(path: Path) -> dict[str, dict[str, object]]:
    rows: dict[str, dict[str, object]] = {}
    with path.open(newline="", encoding="utf-8") as handle:
        for row in csv.DictReader(handle):
            rows[row["body_name"]] = {
                "cad_mass_kg": float(row["cad_mass_kg"]),
                "components": row.get("components", ""),
                "notes": row.get("notes", ""),
            }
    return rows


def find_body(root: ET.Element, name: str) -> ET.Element:
    for body in root.iter("body"):
        if body.attrib.get("name") == name:
            return body
    raise ValueError(f"Body not found: {name}")


def apply_masses(root: ET.Element, mass_config: dict[str, dict[str, object]]) -> list[dict[str, object]]:
    records = []
    for body_name, spec in mass_config.items():
        body = find_body(root, body_name)
        inertial = body.find("inertial")
        if inertial is None:
            raise ValueError(f"Body has no inertial element: {body_name}")
        old_mass = float(inertial.attrib["mass"])
        new_mass = float(spec["cad_mass_kg"])
        ratio = new_mass / old_mass if old_mass > 0 else 1.0
        old_diaginertia = parse_float_triplet(inertial.attrib["diaginertia"])
        new_diaginertia = [value * ratio for value in old_diaginertia]
        inertial.attrib["mass"] = f"{new_mass:.8g}"
        inertial.attrib["diaginertia"] = fmt(new_diaginertia)
        records.append(
            {
                "body": body_name,
                "old_mass_kg": old_mass,
                "new_mass_kg": new_mass,
                "mass_ratio": ratio,
                "old_diaginertia": old_diaginertia,
                "new_diaginertia": new_diaginertia,
                "components": spec["components"],
            }
        )
    return records


def main() -> int:
    parser = argparse.ArgumentParser(description="Apply aggregated CAD body masses to a new MJCF variant.")
    parser.add_argument("--source", type=Path, default=Path("envs/robots/urdf_f_link/URDF_F_link_step_contact.xml"))
    parser.add_argument("--mass-csv", type=Path, default=Path("configs/cad_body_mass_aggregate_user.csv"))
    parser.add_argument("--out", type=Path, default=Path("envs/robots/urdf_f_link/URDF_F_link_user_mass_contact.xml"))
    parser.add_argument("--doc-dir", type=Path, default=Path("docs/hardware_validation"))
    args = parser.parse_args()

    source = args.source.resolve()
    mass_csv = args.mass_csv.resolve()
    out = args.out.resolve()
    doc_dir = args.doc_dir.resolve()
    doc_dir.mkdir(parents=True, exist_ok=True)

    mass_config = read_mass_config(mass_csv)
    tree = ET.parse(source)
    root = tree.getroot()
    root.attrib["model"] = out.stem
    records = apply_masses(root, mass_config)

    ET.indent(root, space="  ")
    tree.write(out, encoding="utf-8", xml_declaration=False)
    compiled = mujoco.MjModel.from_xml_path(str(out))

    old_total = sum(float(record["old_mass_kg"]) for record in records)
    new_total = sum(float(record["new_mass_kg"]) for record in records)
    payload = {
        "source": str(source),
        "mass_csv": str(mass_csv),
        "out": str(out),
        "old_total_kg": old_total,
        "new_total_kg": new_total,
        "compiled_total_kg": float(compiled.body_mass.sum()),
        "method": "Mass is replaced from aggregated CAD component masses. Diagonal inertia is scaled by mass ratio; inertial position and orientation are preserved because supplied CG values are component-local and need frame transforms before safe aggregation.",
        "records": records,
        "compiled": {
            "nq": int(compiled.nq),
            "nv": int(compiled.nv),
            "nu": int(compiled.nu),
            "njnt": int(compiled.njnt),
            "ngeom": int(compiled.ngeom),
        },
    }

    report_path = doc_dir / "user_mass_application_report.json"
    report_path.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")

    md_path = doc_dir / "38_user_mass_contact_model.md"
    lines = [
        "# 사용자 제공 Mass/Contact 적용 모델",
        "",
        "## 목적",
        "",
        "사용자가 제공한 mass property와 foot contact box를 반영한 새 MJCF variant를 만든다. 기존 모델은 덮어쓰지 않는다.",
        "",
        "## 실행 명령",
        "",
        "```bash",
        "python3 scripts/apply_body_mass_config.py --source envs/robots/urdf_f_link/URDF_F_link_user_contact.xml --mass-csv configs/cad_body_mass_aggregate_user.csv --out envs/robots/urdf_f_link/URDF_F_link_user_mass_contact.xml",
        "```",
        "",
        "## 산출물",
        "",
        f"- 새 모델: `{out}`",
        f"- mass CSV: `{mass_csv}`",
        f"- 리포트 JSON: `{report_path}`",
        "",
        "## 질량 요약",
        "",
        f"- 기존 MJCF body mass 합계: `{old_total:.6f} kg`",
        f"- 사용자 제공 CAD 합산 mass: `{new_total:.6f} kg`",
        f"- MuJoCo compile 후 body mass 합계: `{compiled.body_mass.sum():.6f} kg`",
        "",
        "## Body별 적용값",
        "",
        "| body | 기존 kg | 적용 kg | ratio | components |",
        "| --- | ---: | ---: | ---: | --- |",
    ]
    for record in records:
        lines.append(
            f"| `{record['body']}` | {record['old_mass_kg']:.6f} | {record['new_mass_kg']:.6f} | {record['mass_ratio']:.6f} | `{record['components']}` |"
        )
    lines.extend(
        [
            "",
            "## 주의",
            "",
            "제공된 CG는 각 컴포넌트 local frame 기준으로 보인다. 현재 MJCF는 rigid component를 MuJoCo body 하나로 합친 구조이므로, 정확한 aggregate COM/inertia를 만들려면 각 컴포넌트 CG를 해당 body frame으로 변환해야 한다.",
            "",
            "따라서 이번 variant에서는 mass를 CAD 합산값으로 교체하고, 기존 diagonal inertia를 mass ratio로 스케일했다. 최종 설계 검증 전에는 aggregate COM/inertia 계산을 별도 확정해야 한다.",
            "",
        ]
    )
    md_path.write_text("\n".join(lines), encoding="utf-8")

    print(f"Wrote {out}")
    print(f"Wrote {report_path}")
    print(f"Wrote {md_path}")
    print(json.dumps({"old_total_kg": old_total, "new_total_kg": new_total, "compiled_total_kg": float(compiled.body_mass.sum())}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
