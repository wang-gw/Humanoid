from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import mujoco
import numpy as np


PAIR_NAMES = [
    ("thighJ_L_1", "thighJ_R_1"),
    ("thigh_L_1", "thigh_R_1"),
    ("calf_L_1", "calf_R_1"),
    ("footJ_L_1", "footJ_R_1"),
    ("foot_L_1", "foot_R_v1_1"),
]


def body_name(model: mujoco.MjModel, body_id: int) -> str:
    return mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, body_id) or f"body_{body_id}"


def body_record(model: mujoco.MjModel, body_id: int) -> dict[str, object]:
    parent_id = int(model.body_parentid[body_id])
    return {
        "body_id": body_id,
        "body_name": body_name(model, body_id),
        "parent": body_name(model, parent_id) if body_id != 0 else "",
        "mass_kg": float(model.body_mass[body_id]),
        "body_pos": [float(v) for v in model.body_pos[body_id]],
        "inertial_pos": [float(v) for v in model.body_ipos[body_id]],
        "diagonal_inertia": [float(v) for v in model.body_inertia[body_id]],
    }


def pair_record(model: mujoco.MjModel, left_name: str, right_name: str) -> dict[str, object]:
    left_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, left_name)
    right_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, right_name)
    if left_id < 0 or right_id < 0:
        return {"left": left_name, "right": right_name, "missing": True}
    left_mass = float(model.body_mass[left_id])
    right_mass = float(model.body_mass[right_id])
    denom = max(abs(left_mass), abs(right_mass), 1e-9)
    rel_diff = abs(left_mass - right_mass) / denom
    return {
        "left": left_name,
        "right": right_name,
        "left_mass_kg": left_mass,
        "right_mass_kg": right_mass,
        "mass_diff_kg": right_mass - left_mass,
        "relative_diff": rel_diff,
        "needs_cad_check": bool(rel_diff > 0.02 or abs(right_mass - left_mass) > 0.02),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description="MJCF body mass/inertia 값을 감사하고 CAD 확인 자료를 생성한다.")
    parser.add_argument("--model", type=Path, default=Path("envs/robots/urdf_f/URDF_F_named.xml"))
    parser.add_argument("--out-dir", type=Path, default=Path("outputs/analysis/mass_properties"))
    parser.add_argument("--doc-dir", type=Path, default=Path("docs/hardware_validation"))
    args = parser.parse_args()

    model_path = args.model.resolve()
    out_dir = args.out_dir.resolve()
    doc_dir = args.doc_dir.resolve()
    out_dir.mkdir(parents=True, exist_ok=True)
    doc_dir.mkdir(parents=True, exist_ok=True)

    model = mujoco.MjModel.from_xml_path(str(model_path))
    body_rows = [body_record(model, body_id) for body_id in range(model.nbody)]
    pair_rows = [pair_record(model, left, right) for left, right in PAIR_NAMES]

    body_csv = out_dir / "mjcf_body_mass_properties.csv"
    with body_csv.open("w", newline="", encoding="utf-8") as handle:
        fieldnames = ["body_id", "body_name", "parent", "mass_kg", "body_pos", "inertial_pos", "diagonal_inertia"]
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(body_rows)

    pair_csv = out_dir / "mjcf_left_right_mass_pairs.csv"
    with pair_csv.open("w", newline="", encoding="utf-8") as handle:
        fieldnames = ["left", "right", "left_mass_kg", "right_mass_kg", "mass_diff_kg", "relative_diff", "needs_cad_check"]
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for row in pair_rows:
            writer.writerow(row)

    total_mass = float(np.sum(model.body_mass))
    payload = {
        "model_path": str(model_path),
        "total_mass_kg": total_mass,
        "body_count": int(model.nbody),
        "bodies": body_rows,
        "left_right_pairs": pair_rows,
    }
    json_path = out_dir / "mjcf_mass_properties_summary.json"
    json_path.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")

    md_path = doc_dir / "17_mjcf_mass_property_audit.md"
    lines = [
        "# MJCF 질량/관성 감사",
        "",
        "## 목적",
        "",
        "standing 실패 원인 중 하나가 잘못된 질량/관성 또는 좌우 비대칭일 수 있으므로, 현재 named MJCF의 body mass와 inertia를 정리한다.",
        "",
        "이 리포트는 CAD mass property와 비교하기 위한 기준 자료다.",
        "",
        "## 실행 명령",
        "",
        "```bash",
        "python3 scripts/audit_mjcf_mass_properties.py",
        "```",
        "",
        "## 요약",
        "",
        f"- 모델: `{model_path}`",
        f"- 총 질량: `{total_mass:.6f} kg`",
        f"- body 수: `{model.nbody}`",
        "",
        "## 산출물",
        "",
        f"- body mass CSV: `{body_csv}`",
        f"- 좌우 mass pair CSV: `{pair_csv}`",
        f"- JSON: `{json_path}`",
        "",
        "## 좌우 Mass Pair",
        "",
        "| 왼쪽 body | 오른쪽 body | 왼쪽 kg | 오른쪽 kg | 차이 kg | 상대 차이 | CAD 확인 필요 |",
        "| --- | --- | ---: | ---: | ---: | ---: | --- |",
    ]
    for row in pair_rows:
        if row.get("missing"):
            lines.append(f"| `{row['left']}` | `{row['right']}` |  |  |  |  | missing |")
            continue
        lines.append(
            f"| `{row['left']}` | `{row['right']}` | {float(row['left_mass_kg']):.6f} | "
            f"{float(row['right_mass_kg']):.6f} | {float(row['mass_diff_kg']):.6f} | "
            f"{float(row['relative_diff']):.4f} | `{row['needs_cad_check']}` |"
        )
    lines.extend(
        [
            "",
            "## 해석",
            "",
            "좌우 mass pair 차이가 큰 body는 CAD에서 의도된 차이인지 확인해야 한다. 특히 thigh/calf 질량 차이는 보행 안정성과 torque 요구량에 직접 영향을 줄 수 있다.",
            "",
        ]
    )
    md_path.write_text("\n".join(lines), encoding="utf-8")

    print(f"Wrote {body_csv}")
    print(f"Wrote {pair_csv}")
    print(f"Wrote {json_path}")
    print(f"Wrote {md_path}")
    print(json.dumps({"total_mass_kg": total_mass, "body_count": model.nbody}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
