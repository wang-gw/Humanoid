from __future__ import annotations

import argparse
import csv
import json
import xml.etree.ElementTree as ET
from collections import defaultdict
from pathlib import Path

import numpy as np


COMPONENT_MESH = {
    "base_link": "base_link",
    "AK45-10_frR": "AK45-10_R_1",
    "foot_R": "foot_R_v1_1",
}


def component_to_mesh(component: str) -> str:
    return COMPONENT_MESH.get(component, f"{component}_1")


def parse_vec(text: str) -> np.ndarray:
    return np.array([float(v) for v in text.split()], dtype=np.float64)


def quat_to_mat(q: np.ndarray) -> np.ndarray:
    w, x, y, z = [float(v) for v in q]
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def inertia_matrix(row: dict[str, str]) -> np.ndarray:
    ixx = float(row["Ixx"])
    iyy = float(row["Iyy"])
    izz = float(row["Izz"])
    ixy = float(row["Ixy"])
    ixz = float(row["Ixz"])
    iyz = float(row["Iyz"])
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
                    "mesh": component_to_mesh(row["component"]),
                    "mass": float(row["mass_kg"]),
                    "cg_local": np.array([float(row["cg_x_m"]), float(row["cg_y_m"]), float(row["cg_z_m"])], dtype=np.float64),
                    "inertia_local": inertia_matrix(row),
                }
            )
    return dict(groups)


def mesh_transforms(root: ET.Element) -> dict[tuple[str, str], dict[str, np.ndarray]]:
    transforms: dict[tuple[str, str], dict[str, np.ndarray]] = {}
    for body in root.iter("body"):
        body_name = body.attrib.get("name")
        if not body_name:
            continue
        for geom in body.findall("geom"):
            mesh = geom.attrib.get("mesh")
            if not mesh:
                continue
            pos = parse_vec(geom.attrib.get("pos", "0 0 0"))
            quat = parse_vec(geom.attrib.get("quat", "1 0 0 0"))
            transforms[(body_name, mesh)] = {"pos": pos, "rot": quat_to_mat(quat)}
    return transforms


def current_inertials(root: ET.Element) -> dict[str, dict[str, object]]:
    output = {}
    for body in root.iter("body"):
        body_name = body.attrib.get("name")
        inertial = body.find("inertial")
        if body_name and inertial is not None:
            output[body_name] = {
                "mass": float(inertial.attrib["mass"]),
                "pos": parse_vec(inertial.attrib.get("pos", "0 0 0")),
            }
    return output


def aggregate(components: list[dict[str, object]], transforms: dict[tuple[str, str], dict[str, np.ndarray]], body_name: str) -> dict[str, object]:
    transformed = []
    for component in components:
        mesh = str(component["mesh"])
        transform = transforms.get((body_name, mesh))
        if transform is None:
            raise ValueError(f"Missing mesh transform for body={body_name} mesh={mesh}")
        rot = np.asarray(transform["rot"], dtype=np.float64)
        pos = np.asarray(transform["pos"], dtype=np.float64)
        cg_body = pos + rot @ np.asarray(component["cg_local"], dtype=np.float64)
        inertia_body = rot @ np.asarray(component["inertia_local"], dtype=np.float64) @ rot.T
        transformed.append({**component, "cg_body": cg_body, "inertia_body": inertia_body, "geom_pos": pos})

    masses = np.array([float(component["mass"]) for component in transformed], dtype=np.float64)
    cgs = np.array([component["cg_body"] for component in transformed], dtype=np.float64)
    total_mass = float(masses.sum())
    com = (masses[:, None] * cgs).sum(axis=0) / total_mass

    inertia = np.zeros((3, 3), dtype=np.float64)
    eye = np.eye(3)
    for component in transformed:
        mass = float(component["mass"])
        d = np.asarray(component["cg_body"], dtype=np.float64) - com
        inertia += np.asarray(component["inertia_body"], dtype=np.float64)
        inertia += mass * ((float(d.dot(d)) * eye) - np.outer(d, d))

    eigenvalues = np.linalg.eigvalsh(inertia)
    return {
        "mass": total_mass,
        "com": com,
        "fullinertia": inertia_to_full(inertia),
        "eigenvalues": [float(v) for v in eigenvalues],
        "positive_definite": bool(np.all(eigenvalues > 0)),
        "components": [
            {
                "component": str(component["component"]),
                "mesh": str(component["mesh"]),
                "geom_pos": [float(v) for v in component["geom_pos"]],
                "cg_body": [float(v) for v in component["cg_body"]],
            }
            for component in transformed
        ],
    }


def main() -> int:
    parser = argparse.ArgumentParser(description="Audit component inertia aggregation after mesh geom transforms.")
    parser.add_argument("--component-csv", type=Path, default=Path("configs/cad_component_inertia_user.csv"))
    parser.add_argument("--model", type=Path, default=Path("envs/robots/urdf_f_link/URDF_F_link_user_size_mass_contact.xml"))
    parser.add_argument("--doc-dir", type=Path, default=Path("docs/hardware_validation"))
    args = parser.parse_args()

    component_csv = args.component_csv.resolve()
    model_path = args.model.resolve()
    doc_dir = args.doc_dir.resolve()
    doc_dir.mkdir(parents=True, exist_ok=True)

    root = ET.parse(model_path).getroot()
    transforms = mesh_transforms(root)
    current = current_inertials(root)
    groups = read_components(component_csv)

    records = []
    for body_name, components in groups.items():
        agg = aggregate(components, transforms, body_name)
        current_pos = np.asarray(current[body_name]["pos"], dtype=np.float64)
        records.append(
            {
                "body_name": body_name,
                "component_count": len(components),
                "aggregated_mass_kg": agg["mass"],
                "aggregated_com": [float(v) for v in agg["com"]],
                "aggregated_fullinertia": agg["fullinertia"],
                "inertia_eigenvalues": agg["eigenvalues"],
                "positive_definite": agg["positive_definite"],
                "current_mjcf_mass_kg": current[body_name]["mass"],
                "current_mjcf_inertial_pos": [float(v) for v in current_pos],
                "com_delta_from_current": [float(v) for v in (agg["com"] - current_pos)],
                "component_details": agg["components"],
            }
        )

    payload = {
        "component_csv": str(component_csv),
        "model": str(model_path),
        "method": "component body-frame CG/inertia -> mesh geom pos/quat -> target MJCF body frame -> parallel-axis aggregation",
        "records": records,
    }
    json_path = doc_dir / "component_inertia_transformed_audit.json"
    json_path.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")

    md_path = doc_dir / "50_component_inertia_transformed_audit.md"
    lines = [
        "# Component Inertia Transformed Audit",
        "",
        "## 목적",
        "",
        "component local frame 기준으로 받은 CG/inertia를 현재 MJCF의 mesh geom `pos/quat`로 target body frame에 변환한 뒤, body별 aggregate COM/inertia를 계산한다.",
        "",
        "## 입력",
        "",
        f"- component CSV: `{component_csv}`",
        f"- 기준 MJCF: `{model_path}`",
        "",
        "## Body별 결과",
        "",
        "| body | mass kg | aggregate COM | current MJCF COM | delta norm | inertia PD |",
        "| --- | ---: | --- | --- | ---: | --- |",
    ]
    for record in records:
        delta = np.asarray(record["com_delta_from_current"], dtype=np.float64)
        lines.append(
            "| `{body}` | {mass:.6f} | `{com}` | `{current}` | {delta_norm:.6f} | `{pd}` |".format(
                body=record["body_name"],
                mass=record["aggregated_mass_kg"],
                com=[round(v, 6) for v in record["aggregated_com"]],
                current=[round(v, 6) for v in record["current_mjcf_inertial_pos"]],
                delta_norm=float(np.linalg.norm(delta)),
                pd=record["positive_definite"],
            )
        )
    lines.extend(
        [
            "",
            "## 판단",
            "",
            "이 결과는 49번 문서의 단순 합산보다 현재 MJCF 구조에 더 가까운 해석이다. 다만 base body 안에 collapse된 `thighJR_*`, `AK45-36_tr*` mesh geom은 현재 MJCF에서 `pos=0`으로 들어가 있어, base 쪽 aggregate COM은 여전히 신뢰도가 낮다.",
            "",
            "다리 하위 body도 delta가 남아 있으므로, 이 값을 바로 최종 설계 검증용 inertial로 확정하기 전에 MuJoCo compile 및 geometry/standing probe로 sanity check를 거쳐야 한다.",
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
