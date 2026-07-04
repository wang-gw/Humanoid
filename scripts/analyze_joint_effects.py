from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import mujoco
import numpy as np


TARGET_GEOMS = ("foot_L_1_sole_collision", "foot_R_v1_1_sole_collision")


def hinge_joints(model: mujoco.MjModel) -> list[dict[str, object]]:
    joints: list[dict[str, object]] = []
    for joint_id in range(model.njnt):
        if model.jnt_type[joint_id] != mujoco.mjtJoint.mjJNT_HINGE:
            continue
        joints.append(
            {
                "id": joint_id,
                "name": mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, joint_id) or f"joint_{joint_id}",
                "qposadr": int(model.jnt_qposadr[joint_id]),
                "axis": [float(v) for v in model.jnt_axis[joint_id]],
                "range": [float(v) for v in model.jnt_range[joint_id]],
            }
        )
    return joints


def set_pose(model: mujoco.MjModel, data: mujoco.MjData, base_z: float, joint_targets: dict[str, float]) -> None:
    data.qpos[:] = model.qpos0
    data.qvel[:] = 0.0
    data.qpos[0:3] = np.array([0.0, 0.0, base_z])
    data.qpos[3:7] = np.array([1.0, 0.0, 0.0, 0.0])
    for joint_id in range(model.njnt):
        if model.jnt_type[joint_id] != mujoco.mjtJoint.mjJNT_HINGE:
            continue
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, joint_id) or f"joint_{joint_id}"
        if name in joint_targets:
            data.qpos[model.jnt_qposadr[joint_id]] = float(joint_targets[name])
    mujoco.mj_forward(model, data)


def geom_positions(model: mujoco.MjModel, data: mujoco.MjData) -> dict[str, np.ndarray]:
    positions: dict[str, np.ndarray] = {}
    for name in TARGET_GEOMS:
        geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name)
        if geom_id >= 0:
            positions[name] = np.asarray(data.geom_xpos[geom_id]).copy()
    return positions


def main() -> int:
    parser = argparse.ArgumentParser(description="Analyze kinematic effect of each joint on simplified foot geoms.")
    parser.add_argument("--model", type=Path, default=Path("envs/robots/urdf_f/URDF_F_contact.xml"))
    parser.add_argument("--pose-json", type=Path, default=Path("configs/standing_pose_candidate.json"))
    parser.add_argument("--delta", type=float, default=0.1)
    parser.add_argument("--out-dir", type=Path, default=Path("outputs/analysis/joint_effects"))
    parser.add_argument("--doc-dir", type=Path, default=Path("docs/hardware_validation"))
    args = parser.parse_args()

    model_path = args.model.resolve()
    pose_path = args.pose_json.resolve()
    out_dir = args.out_dir.resolve()
    doc_dir = args.doc_dir.resolve()
    out_dir.mkdir(parents=True, exist_ok=True)
    doc_dir.mkdir(parents=True, exist_ok=True)

    model = mujoco.MjModel.from_xml_path(str(model_path))
    data = mujoco.MjData(model)
    joints = hinge_joints(model)
    pose = json.loads(pose_path.read_text(encoding="utf-8"))
    joint_targets = {str(k): float(v) for k, v in pose["joint_targets"].items()}
    base_z = float(pose["base_z"])

    set_pose(model, data, base_z, joint_targets)
    baseline = geom_positions(model, data)

    rows: list[dict[str, object]] = []
    for joint in joints:
        name = str(joint["name"])
        target = dict(joint_targets)
        target[name] = target.get(name, 0.0) + args.delta
        set_pose(model, data, base_z, target)
        plus = geom_positions(model, data)

        target = dict(joint_targets)
        target[name] = target.get(name, 0.0) - args.delta
        set_pose(model, data, base_z, target)
        minus = geom_positions(model, data)

        for geom_name in sorted(baseline):
            plus_delta = plus[geom_name] - baseline[geom_name]
            minus_delta = minus[geom_name] - baseline[geom_name]
            central = (plus[geom_name] - minus[geom_name]) / (2.0 * args.delta)
            rows.append(
                {
                    "joint": name,
                    "axis": " ".join(f"{v:g}" for v in joint["axis"]),
                    "target_geom": geom_name,
                    "plus_dx": float(plus_delta[0]),
                    "plus_dy": float(plus_delta[1]),
                    "plus_dz": float(plus_delta[2]),
                    "minus_dx": float(minus_delta[0]),
                    "minus_dy": float(minus_delta[1]),
                    "minus_dz": float(minus_delta[2]),
                    "sensitivity_dx_per_rad": float(central[0]),
                    "sensitivity_dy_per_rad": float(central[1]),
                    "sensitivity_dz_per_rad": float(central[2]),
                    "sensitivity_norm_per_rad": float(np.linalg.norm(central)),
                }
            )

    csv_path = out_dir / "joint_effects.csv"
    with csv_path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)

    grouped: dict[str, dict[str, object]] = {}
    for joint in joints:
        name = str(joint["name"])
        joint_rows = [row for row in rows if row["joint"] == name]
        strongest = max(joint_rows, key=lambda row: float(row["sensitivity_norm_per_rad"]))
        grouped[name] = {
            "axis": joint["axis"],
            "strongest_target": strongest["target_geom"],
            "strongest_norm_per_rad": strongest["sensitivity_norm_per_rad"],
            "rows": joint_rows,
        }

    json_path = out_dir / "joint_effects_summary.json"
    json_path.write_text(
        json.dumps(
            {
                "model_path": str(model_path),
                "pose_path": str(pose_path),
                "delta_rad": args.delta,
                "baseline_geom_positions": {k: [float(vv) for vv in v] for k, v in baseline.items()},
                "summary": grouped,
            },
            indent=2,
            ensure_ascii=False,
        )
        + "\n",
        encoding="utf-8",
    )

    md_path = doc_dir / "12_joint_effect_analysis.md"
    lines = [
        "# 조인트 영향 분석",
        "",
        "## 목적",
        "",
        "CAD export 조인트 이름과 positive direction이 아직 확정되지 않았기 때문에, 각 조인트를 작은 각도만큼 움직였을 때 좌우 발 collision 중심이 어떻게 이동하는지 수치로 기록한다.",
        "",
        "이 분석은 동역학 검증이 아니라 순수 기구학적 영향 분석이다. 최종 매핑 확정에는 CAD 축 방향 확인이 필요하다.",
        "",
        "## 실행 명령",
        "",
        "```bash",
        "python3 scripts/analyze_joint_effects.py",
        "```",
        "",
        "## 입력",
        "",
        f"- 모델: `{model_path}`",
        f"- pose 후보: `{pose_path}`",
        f"- perturbation: `+/-{args.delta}` rad",
        "",
        "## 산출물",
        "",
        f"- CSV: `{csv_path}`",
        f"- JSON: `{json_path}`",
        "",
        "## 요약",
        "",
        "| 조인트 | 축 | 가장 크게 영향받은 대상 | 민감도 norm / rad |",
        "| --- | --- | --- | ---: |",
    ]
    for joint_name, item in grouped.items():
        axis = " ".join(f"{float(v):g}" for v in item["axis"])
        lines.append(
            f"| `{joint_name}` | `{axis}` | `{item['strongest_target']}` | {float(item['strongest_norm_per_rad']):.6f} |"
        )
    lines.extend(
        [
            "",
            "## 해석",
            "",
            "각 조인트가 어느 발에 더 큰 영향을 주는지 확인하면 좌우 branch와 action order를 검증하는 데 도움이 된다. 다만 현재 pose 후보 자체가 동역학 standing을 통과하지 못했으므로, 이 결과는 조인트 매핑 보조 자료로만 사용한다.",
            "",
            "다음 단계에서는 CAD에서 joint 이름, 실제 회전축, positive direction을 확인한 뒤 이 표와 비교해야 한다.",
            "",
        ]
    )
    md_path.write_text("\n".join(lines), encoding="utf-8")

    print(f"Wrote {csv_path}")
    print(f"Wrote {json_path}")
    print(f"Wrote {md_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
