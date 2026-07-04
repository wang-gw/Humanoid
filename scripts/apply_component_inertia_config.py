from __future__ import annotations

import argparse
import csv
import json
import xml.etree.ElementTree as ET
from collections import defaultdict
from pathlib import Path

import mujoco
import numpy as np


INERTIA_KEYS = ("Ixx", "Iyy", "Izz", "Ixy", "Ixz", "Iyz")


def fmt(values: list[float] | np.ndarray) -> str:
    return " ".join(f"{float(value):.8g}" for value in values)


def inertia_matrix(row: dict[str, str]) -> np.ndarray:
    ixx, iyy, izz, ixy, ixz, iyz = (float(row[key]) for key in INERTIA_KEYS)
    return np.array([[ixx, ixy, ixz], [ixy, iyy, iyz], [ixz, iyz, izz]], dtype=np.float64)


def inertia_to_full(I: np.ndarray) -> list[float]:
    return [float(I[0, 0]), float(I[1, 1]), float(I[2, 2]), float(I[0, 1]), float(I[0, 2]), float(I[1, 2])]


def read_components(path: Path) -> dict[str, list[dict[str, object]]]:
    groups: dict[str, list[dict[str, object]]] = defaultdict(list)
    with path.open(newline="", encoding="utf-8") as handle:
        for row in csv.DictReader(handle):
            groups[row["body_name"]].append(
                {
                    "component": row["component"],
                    "mass": float(row["mass_kg"]),
                    "cg": np.array([float(row["cg_x_m"]), float(row["cg_y_m"]), float(row["cg_z_m"])], dtype=np.float64),
                    "inertia_com": inertia_matrix(row),
                }
            )
    return dict(groups)


def aggregate(components: list[dict[str, object]]) -> dict[str, object]:
    masses = np.array([float(component["mass"]) for component in components], dtype=np.float64)
    cgs = np.array([component["cg"] for component in components], dtype=np.float64)
    total_mass = float(masses.sum())
    com = (masses[:, None] * cgs).sum(axis=0) / total_mass

    inertia = np.zeros((3, 3), dtype=np.float64)
    eye = np.eye(3)
    for component in components:
        mass = float(component["mass"])
        d = np.asarray(component["cg"], dtype=np.float64) - com
        inertia += np.asarray(component["inertia_com"], dtype=np.float64)
        inertia += mass * ((float(d.dot(d)) * eye) - np.outer(d, d))

    eigvals = np.linalg.eigvalsh(inertia)
    return {
        "mass": total_mass,
        "com": com,
        "fullinertia": np.array(inertia_to_full(inertia), dtype=np.float64),
        "eigenvalues": eigvals,
    }


def find_body(root: ET.Element, name: str) -> ET.Element:
    for body in root.iter("body"):
        if body.attrib.get("name") == name:
            return body
    raise ValueError(f"Body not found: {name}")


def apply_inertials(root: ET.Element, groups: dict[str, list[dict[str, object]]], skip_bodies: set[str]) -> list[dict[str, object]]:
    records = []
    for body_name, components in groups.items():
        if body_name in skip_bodies:
            records.append({"body_name": body_name, "skipped": True, "reason": "explicitly skipped"})
            continue
        agg = aggregate(components)
        if not np.all(np.asarray(agg["eigenvalues"]) > 0.0):
            raise ValueError(f"Aggregated inertia is not positive definite for {body_name}: {agg['eigenvalues']}")

        body = find_body(root, body_name)
        inertial = body.find("inertial")
        if inertial is None:
            raise ValueError(f"Body has no inertial element: {body_name}")

        old = dict(inertial.attrib)
        inertial.attrib["pos"] = fmt(agg["com"])
        inertial.attrib["mass"] = f"{float(agg['mass']):.8g}"
        inertial.attrib["fullinertia"] = fmt(agg["fullinertia"])
        inertial.attrib.pop("quat", None)
        inertial.attrib.pop("diaginertia", None)
        records.append(
            {
                "body_name": body_name,
                "skipped": False,
                "old": old,
                "new": dict(inertial.attrib),
                "components": [str(component["component"]) for component in components],
                "eigenvalues": [float(v) for v in agg["eigenvalues"]],
            }
        )
    return records


def main() -> int:
    parser = argparse.ArgumentParser(description="Apply user component aggregate full inertia to an experimental MJCF variant.")
    parser.add_argument("--source", type=Path, default=Path("envs/robots/urdf_f_link/URDF_F_link_user_size_mass_contact.xml"))
    parser.add_argument("--component-csv", type=Path, default=Path("configs/cad_component_inertia_user.csv"))
    parser.add_argument("--out", type=Path, default=Path("envs/robots/urdf_f_link/URDF_F_link_user_size_mass_inertia_direct_nobase_contact.xml"))
    parser.add_argument("--skip-body", action="append", default=["base_link"])
    parser.add_argument("--doc-dir", type=Path, default=Path("docs/hardware_validation"))
    args = parser.parse_args()

    source = args.source.resolve()
    component_csv = args.component_csv.resolve()
    out = args.out.resolve()
    doc_dir = args.doc_dir.resolve()
    doc_dir.mkdir(parents=True, exist_ok=True)

    groups = read_components(component_csv)
    root = ET.parse(source).getroot()
    root.attrib["model"] = out.stem
    records = apply_inertials(root, groups, set(args.skip_body))

    ET.indent(root, space="  ")
    ET.ElementTree(root).write(out, encoding="utf-8", xml_declaration=False)
    model = mujoco.MjModel.from_xml_path(str(out))

    payload = {
        "source": str(source),
        "component_csv": str(component_csv),
        "out": str(out),
        "method": "Experimental direct body-frame aggregation from user component CG/inertia. base_link is skipped by default because its frame is not yet trusted.",
        "skip_bodies": list(args.skip_body),
        "compiled_total_mass_kg": float(model.body_mass.sum()),
        "compiled": {"nq": int(model.nq), "nv": int(model.nv), "nu": int(model.nu), "njnt": int(model.njnt), "ngeom": int(model.ngeom)},
        "records": records,
    }
    report_path = doc_dir / "component_inertia_application_report.json"
    report_path.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")

    md_path = doc_dir / "51_component_inertia_experimental_variant.md"
    lines = [
        "# Component Inertia Experimental Variant",
        "",
        "## 목적",
        "",
        "사용자가 제공한 component별 full inertia를 실험용 MJCF variant에 적용한다. 단, frame 불확실성이 가장 큰 `base_link`는 유지한다.",
        "",
        "## 실행 명령",
        "",
        "```bash",
        "python3 scripts/apply_component_inertia_config.py",
        "```",
        "",
        "## 산출물",
        "",
        f"- 새 모델: `{out}`",
        f"- 적용 리포트: `{report_path}`",
        "",
        "## 적용 방식",
        "",
        "- component별 inertia는 해당 body frame 기준이라고 가정하고 body별로 합산했다.",
        "- 합산에는 parallel-axis theorem을 사용했다.",
        "- MJCF에는 `fullinertia`로 기록했다.",
        "- `base_link`는 기존 inertial을 유지했다.",
        "",
        "## Compile 결과",
        "",
        f"- total mass: `{model.body_mass.sum():.6f} kg`",
        f"- nq: `{model.nq}`",
        f"- nv: `{model.nv}`",
        f"- nu: `{model.nu}`",
        "",
        "## 적용 Body",
        "",
        "| body | 상태 | components |",
        "| --- | --- | --- |",
    ]
    for record in records:
        status = "skipped" if record["skipped"] else "applied"
        components = "" if record["skipped"] else ", ".join(record["components"])
        lines.append(f"| `{record['body_name']}` | `{status}` | `{components}` |")
    lines.extend(
        [
            "",
            "## 판단",
            "",
            "이 모델은 최종 설계 검증 모델이 아니라 frame 가정 검증용 실험 모델이다. 다음 단계에서 geometry sanity check와 neutral PD standing probe를 기존 모델과 비교해, inertia 적용이 물리적으로 납득되는 방향인지 확인한다.",
            "",
        ]
    )
    md_path.write_text("\n".join(lines), encoding="utf-8")

    print(f"Wrote {out}")
    print(f"Wrote {report_path}")
    print(f"Wrote {md_path}")
    print(json.dumps({"compiled_total_mass_kg": float(model.body_mass.sum()), "applied": sum(not r["skipped"] for r in records)}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
