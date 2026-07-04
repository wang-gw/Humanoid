from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import mujoco
import numpy as np


CANDIDATES = {
    "user_direct": {
        "left": [-0.07675, 0.06, -0.005],
        "right": [-0.07725, 0.06, 0.045],
        "halfsize": [0.035, 0.06, 0.02],
        "description": "사용자 제공 Fusion/local center를 MJCF foot body local로 그대로 해석",
    },
    "body_centered_user_size_bottom_40mm": {
        "left": [0.0, 0.0, -0.02],
        "right": [0.0, 0.0, -0.02],
        "halfsize": [0.035, 0.06, 0.02],
        "description": "사용자 제공 foot size는 유지하고, MJCF foot body 중심 기준으로 좌우 동일하게 배치",
    },
    "body_centered_thin_contact": {
        "left": [0.0, 0.0, -0.035],
        "right": [0.0, 0.0, -0.035],
        "halfsize": [0.035, 0.06, 0.005],
        "description": "이전 진단 모델에서 사용한 얇은 foot contact",
    },
}


def total_com(model: mujoco.MjModel, data: mujoco.MjData) -> np.ndarray:
    mass = np.asarray(model.body_mass)
    return (np.asarray(data.xipos) * mass[:, None]).sum(axis=0) / mass.sum()


def support_aabb(centers: list[np.ndarray], halfsize: np.ndarray) -> dict[str, float]:
    xs: list[float] = []
    ys: list[float] = []
    for center in centers:
        xs.extend([float(center[0] - halfsize[0]), float(center[0] + halfsize[0])])
        ys.extend([float(center[1] - halfsize[1]), float(center[1] + halfsize[1])])
    return {"x_min": min(xs), "x_max": max(xs), "y_min": min(ys), "y_max": max(ys)}


def margin(point: np.ndarray, aabb: dict[str, float]) -> dict[str, float | bool]:
    x, y = float(point[0]), float(point[1])
    return {
        "inside_x": aabb["x_min"] <= x <= aabb["x_max"],
        "inside_y": aabb["y_min"] <= y <= aabb["y_max"],
        "x_margin_min": x - aabb["x_min"],
        "x_margin_max": aabb["x_max"] - x,
        "y_margin_min": y - aabb["y_min"],
        "y_margin_max": aabb["y_max"] - y,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description="Audit candidate transforms for user-provided foot contact boxes.")
    parser.add_argument("--model", type=Path, default=Path("envs/robots/urdf_f_link/URDF_F_link_user_mass_contact.xml"))
    parser.add_argument("--out-dir", type=Path, default=Path("outputs/analysis/foot_contact_frame_candidates"))
    parser.add_argument("--doc-dir", type=Path, default=Path("docs/hardware_validation"))
    args = parser.parse_args()

    out_dir = args.out_dir.resolve()
    doc_dir = args.doc_dir.resolve()
    out_dir.mkdir(parents=True, exist_ok=True)
    doc_dir.mkdir(parents=True, exist_ok=True)

    model = mujoco.MjModel.from_xml_path(str(args.model.resolve()))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    left_body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "foot_L_1")
    right_body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "foot_R_v1_1")
    body_pos = {
        "left": np.asarray(data.xpos[left_body], dtype=float),
        "right": np.asarray(data.xpos[right_body], dtype=float),
    }
    com = total_com(model, data)

    rows: list[dict[str, object]] = []
    for name, candidate in CANDIDATES.items():
        halfsize = np.asarray(candidate["halfsize"], dtype=float)
        left_world = body_pos["left"] + np.asarray(candidate["left"], dtype=float)
        right_world = body_pos["right"] + np.asarray(candidate["right"], dtype=float)
        aabb = support_aabb([left_world, right_world], halfsize)
        support_margin = margin(com, aabb)
        rows.append(
            {
                "candidate": name,
                "description": candidate["description"],
                "left_center_body": candidate["left"],
                "right_center_body": candidate["right"],
                "halfsize": candidate["halfsize"],
                "left_center_world": [float(v) for v in left_world],
                "right_center_world": [float(v) for v in right_world],
                "world_z_delta_abs": float(abs(left_world[2] - right_world[2])),
                "support_aabb": aabb,
                "com_world": [float(v) for v in com],
                "com_inside_x": bool(support_margin["inside_x"]),
                "com_inside_y": bool(support_margin["inside_y"]),
                "support_margin": support_margin,
            }
        )

    json_path = out_dir / "foot_contact_frame_candidates.json"
    json_path.write_text(json.dumps({"model": str(args.model.resolve()), "rows": rows}, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    csv_path = out_dir / "foot_contact_frame_candidates.csv"
    with csv_path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)

    md_path = doc_dir / "41_foot_contact_frame_candidates.md"
    lines = [
        "# Foot contact frame 후보 감사",
        "",
        "## 목적",
        "",
        "사용자 제공 foot contact box가 Fusion local frame 기준이므로, 현재 MJCF foot body frame에 어떻게 넣어야 하는지 후보를 비교한다.",
        "",
        "## 실행 명령",
        "",
        "```bash",
        "python3 scripts/audit_foot_contact_frame_candidates.py",
        "```",
        "",
        "## 후보 비교",
        "",
        "| 후보 | 좌우 z 차이 m | COM inside x | COM inside y | left world | right world |",
        "| --- | ---: | --- | --- | --- | --- |",
    ]
    for row in rows:
        lines.append(
            f"| `{row['candidate']}` | {row['world_z_delta_abs']:.6f} | `{row['com_inside_x']}` | `{row['com_inside_y']}` | `{row['left_center_world']}` | `{row['right_center_world']}` |"
        )
    lines.extend(
        [
            "",
            "## 산출물",
            "",
            f"- JSON: `{json_path}`",
            f"- CSV: `{csv_path}`",
            "",
            "## 판단",
            "",
            "`user_direct` 해석은 좌우 contact center 높이가 `0.05 m` 차이 나므로 현재 MJCF body frame에는 직접 사용할 수 없다.",
            "",
            "`body_centered_user_size_bottom_40mm`는 사용자 제공 foot size를 유지하면서 좌우 contact 높이를 맞추고 COM projection도 support AABB 안에 둔다. 따라서 다음 동역학 검증용 변환 후보로 사용한다.",
            "",
        ]
    )
    md_path.write_text("\n".join(lines), encoding="utf-8")

    print(f"Wrote {json_path}")
    print(f"Wrote {csv_path}")
    print(f"Wrote {md_path}")
    print(json.dumps(rows, indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
