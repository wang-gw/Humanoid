from __future__ import annotations

import argparse
import csv
import json
import xml.etree.ElementTree as ET
from collections import defaultdict
from pathlib import Path

import numpy as np


INERTIA_KEYS = ("Ixx", "Iyy", "Izz", "Ixy", "Ixz", "Iyz")


def parse_vec(text: str) -> np.ndarray:
    return np.array([float(v) for v in text.split()], dtype=np.float64)


def inertia_matrix(row: dict[str, str]) -> np.ndarray:
    ixx, iyy, izz, ixy, ixz, iyz = (float(row[key]) for key in INERTIA_KEYS)
    return np.array(
        [
            [ixx, ixy, ixz],
            [ixy, iyy, iyz],
            [ixz, iyz, izz],
        ],
        dtype=np.float64,
    )


def inertia_to_full(I: np.ndarray) -> list[float]:
    return [float(I[0, 0]), float(I[1, 1]), float(I[2, 2]), float(I[0, 1]), float(I[0, 2]), float(I[1, 2])]


def read_components(path: Path) -> dict[str, list[dict[str, object]]]:
    groups: dict[str, list[dict[str, object]]] = defaultdict(list)
    with path.open(newline="", encoding="utf-8") as handle:
        for row in csv.DictReader(handle):
            mass = float(row["mass_kg"])
            cg = np.array([float(row["cg_x_m"]), float(row["cg_y_m"]), float(row["cg_z_m"])], dtype=np.float64)
            groups[row["body_name"]].append(
                {
                    "component": row["component"],
                    "mass": mass,
                    "cg": cg,
                    "inertia_com": inertia_matrix(row),
                    "notes": row.get("notes", ""),
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
        cg = np.asarray(component["cg"], dtype=np.float64)
        d = cg - com
        inertia += np.asarray(component["inertia_com"], dtype=np.float64)
        inertia += mass * ((float(d.dot(d)) * eye) - np.outer(d, d))

    eigvals = np.linalg.eigvalsh(inertia)
    return {
        "mass": total_mass,
        "com": com,
        "fullinertia": inertia_to_full(inertia),
        "eigenvalues": [float(v) for v in eigvals],
        "positive_definite": bool(np.all(eigvals > 0.0)),
    }


def mjcf_inertials(path: Path) -> dict[str, dict[str, object]]:
    root = ET.parse(path).getroot()
    inertials: dict[str, dict[str, object]] = {}
    for body in root.iter("body"):
        name = body.attrib.get("name")
        inertial = body.find("inertial")
        if not name or inertial is None:
            continue
        inertials[name] = {
            "mass": float(inertial.attrib["mass"]),
            "pos": parse_vec(inertial.attrib.get("pos", "0 0 0")),
            "diaginertia": parse_vec(inertial.attrib.get("diaginertia", "0 0 0")),
            "quat": parse_vec(inertial.attrib.get("quat", "1 0 0 0")),
        }
    return inertials


def main() -> int:
    parser = argparse.ArgumentParser(description="Audit user-supplied component COM/inertia aggregation by MJCF body.")
    parser.add_argument("--component-csv", type=Path, default=Path("configs/cad_component_inertia_user.csv"))
    parser.add_argument("--model", type=Path, default=Path("envs/robots/urdf_f_link/URDF_F_link_user_size_mass_contact.xml"))
    parser.add_argument("--doc-dir", type=Path, default=Path("docs/hardware_validation"))
    args = parser.parse_args()

    component_csv = args.component_csv.resolve()
    model_path = args.model.resolve()
    doc_dir = args.doc_dir.resolve()
    doc_dir.mkdir(parents=True, exist_ok=True)

    groups = read_components(component_csv)
    current = mjcf_inertials(model_path)
    records = []
    for body_name, components in groups.items():
        agg = aggregate(components)
        current_spec = current.get(body_name)
        record = {
            "body_name": body_name,
            "component_count": len(components),
            "components": [str(component["component"]) for component in components],
            "aggregated_mass_kg": agg["mass"],
            "aggregated_com": [float(v) for v in agg["com"]],
            "aggregated_fullinertia": agg["fullinertia"],
            "inertia_eigenvalues": agg["eigenvalues"],
            "positive_definite": agg["positive_definite"],
            "current_mjcf_mass_kg": current_spec["mass"] if current_spec else None,
            "current_mjcf_inertial_pos": [float(v) for v in current_spec["pos"]] if current_spec else None,
            "com_delta_from_current": [float(v) for v in (agg["com"] - current_spec["pos"])] if current_spec else None,
        }
        records.append(record)

    payload = {
        "component_csv": str(component_csv),
        "model": str(model_path),
        "assumption": "Component CG and inertia are treated as already expressed in each target MJCF body frame. This must be confirmed before applying to a simulation model.",
        "records": records,
    }
    json_path = doc_dir / "component_inertia_aggregation_audit.json"
    json_path.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")

    md_path = doc_dir / "49_component_inertia_aggregation_audit.md"
    lines = [
        "# Component Inertia Aggregation Audit",
        "",
        "## 목적",
        "",
        "사용자가 제공한 component별 mass, COM, inertia tensor를 현재 MJCF body 단위로 합산할 수 있는지 확인한다. 이 단계는 바로 모델에 적용하는 단계가 아니라, frame 호환성과 수치 안정성을 먼저 검증하는 감사 단계다.",
        "",
        "## 입력",
        "",
        f"- component inertia CSV: `{component_csv}`",
        f"- 기준 MJCF: `{model_path}`",
        "",
        "## 가정",
        "",
        "아래 합산은 각 component의 CG와 inertia가 이미 해당 target MJCF body frame에 표현되어 있다고 가정한다. 이 가정이 틀리면 aggregate COM/inertia도 틀리므로, 이상치가 보이면 모델에 바로 적용하지 않는다.",
        "",
        "## Body별 합산 결과",
        "",
        "| body | components | mass kg | aggregate COM | current MJCF COM | COM delta | inertia PD |",
        "| --- | ---: | ---: | --- | --- | --- | --- |",
    ]
    for record in records:
        lines.append(
            "| `{body}` | {count} | {mass:.6f} | `{com}` | `{current}` | `{delta}` | `{pd}` |".format(
                body=record["body_name"],
                count=record["component_count"],
                mass=record["aggregated_mass_kg"],
                com=[round(v, 6) for v in record["aggregated_com"]],
                current=[round(v, 6) for v in record["current_mjcf_inertial_pos"]] if record["current_mjcf_inertial_pos"] else None,
                delta=[round(v, 6) for v in record["com_delta_from_current"]] if record["com_delta_from_current"] else None,
                pd=record["positive_definite"],
            )
        )
    lines.extend(
        [
            "",
            "## 판단 기준",
            "",
            "- `inertia PD=True`는 inertia tensor 자체가 물리적으로 가능한 양의 정부호임을 의미한다.",
            "- `COM delta`가 매우 크면 제공된 CG frame과 현재 MJCF body frame이 다를 가능성이 있다.",
            "- 특히 `base_link`는 제공된 local CG z가 `-0.467 m`라서 현재 MJCF inertial pos와 직접 호환되는지 별도 확인이 필요하다.",
            "",
            "## 다음 조치",
            "",
            "합산 inertia tensor는 대부분 양의 정부호인지 확인한 뒤, frame 호환성이 납득되는 body부터 새 MJCF variant에 적용한다. frame이 불확실한 body는 기존 inertial을 유지하거나, CAD assembly transform을 사용해 target MJCF body frame으로 다시 변환해야 한다.",
            "",
        ]
    )
    md_path.write_text("\n".join(lines), encoding="utf-8")

    print(f"Wrote {json_path}")
    print(f"Wrote {md_path}")
    print(json.dumps({"records": len(records), "all_positive_definite": all(r["positive_definite"] for r in records)}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
