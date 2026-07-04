from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import mujoco
import numpy as np


FOOT_GEOMS = {
    "left": "foot_L_1_sole_collision",
    "right": "foot_R_v1_1_sole_collision",
}
TARGET_JOINTS = (
    "left_hip_roll",
    "left_hip_pitch",
    "left_ankle_pitch",
    "left_ankle_roll",
    "right_hip_roll",
    "right_hip_pitch",
    "right_ankle_pitch",
    "right_ankle_roll",
)


def hinge_joints(model: mujoco.MjModel) -> list[dict[str, object]]:
    joints = []
    for joint_id in range(model.njnt):
        if model.jnt_type[joint_id] != mujoco.mjtJoint.mjJNT_HINGE:
            continue
        joints.append(
            {
                "id": joint_id,
                "name": mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, joint_id) or f"joint_{joint_id}",
                "qposadr": int(model.jnt_qposadr[joint_id]),
                "dofadr": int(model.jnt_dofadr[joint_id]),
                "axis_local": np.array(model.jnt_axis[joint_id], dtype=np.float64),
                "body_id": int(model.jnt_bodyid[joint_id]),
            }
        )
    return joints


def load_pose(model: mujoco.MjModel, joints: list[dict[str, object]], pose_path: Path) -> tuple[float, dict[str, float]]:
    payload = json.loads(pose_path.read_text(encoding="utf-8"))
    values = {str(joint["name"]): float(model.qpos0[int(joint["qposadr"])]) for joint in joints}
    values.update({str(key): float(value) for key, value in payload["joint_targets"].items()})
    return float(payload["base_z"]), values


def set_pose(model: mujoco.MjModel, data: mujoco.MjData, joints: list[dict[str, object]], base_z: float, values: dict[str, float]) -> None:
    data.qpos[:] = model.qpos0
    data.qvel[:] = 0.0
    data.qpos[0:3] = np.array([0.0, 0.0, base_z])
    data.qpos[3:7] = np.array([1.0, 0.0, 0.0, 0.0])
    for joint in joints:
        data.qpos[int(joint["qposadr"])] = float(values[str(joint["name"])])
    mujoco.mj_forward(model, data)


def total_com(model: mujoco.MjModel, data: mujoco.MjData) -> np.ndarray:
    mass = np.asarray(model.body_mass)
    return (np.asarray(data.xipos) * mass[:, None]).sum(axis=0) / mass.sum()


def support_aabb(model: mujoco.MjModel, data: mujoco.MjData) -> dict[str, float]:
    xs: list[float] = []
    ys: list[float] = []
    for geom_name in FOOT_GEOMS.values():
        geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, geom_name)
        center = np.asarray(data.geom_xpos[geom_id])
        xmat = np.asarray(data.geom_xmat[geom_id]).reshape(3, 3)
        half = np.asarray(model.geom_size[geom_id])
        extent = np.abs(xmat) @ half
        xs.extend([float(center[0] - extent[0]), float(center[0] + extent[0])])
        ys.extend([float(center[1] - extent[1]), float(center[1] + extent[1])])
    return {"x_min": min(xs), "x_max": max(xs), "y_min": min(ys), "y_max": max(ys)}


def foot_corners(model: mujoco.MjModel, data: mujoco.MjData, side: str) -> dict[str, list[float]]:
    geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, FOOT_GEOMS[side])
    center = np.asarray(data.geom_xpos[geom_id], dtype=np.float64)
    xmat = np.asarray(data.geom_xmat[geom_id], dtype=np.float64).reshape(3, 3)
    half = np.asarray(model.geom_size[geom_id], dtype=np.float64)
    corners: dict[str, list[float]] = {}
    for sx, x_label in ((-1.0, "back"), (1.0, "front")):
        for sy, y_label in ((-1.0, "right"), (1.0, "left")):
            bottom = center + sx * half[0] * xmat[:, 0] + sy * half[1] * xmat[:, 1] - half[2] * xmat[:, 2]
            corners[f"{x_label}_{y_label}_bottom"] = [float(v) for v in bottom]
    return corners


def snapshot(model: mujoco.MjModel, data: mujoco.MjData) -> dict[str, object]:
    foot = {}
    for side, geom_name in FOOT_GEOMS.items():
        geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, geom_name)
        xmat = np.asarray(data.geom_xmat[geom_id], dtype=np.float64).reshape(3, 3)
        foot[side] = {
            "center": [float(v) for v in data.geom_xpos[geom_id]],
            "x_axis": [float(v) for v in xmat[:, 0]],
            "y_axis": [float(v) for v in xmat[:, 1]],
            "z_axis": [float(v) for v in xmat[:, 2]],
            "corners": foot_corners(model, data, side),
        }
    com = total_com(model, data)
    aabb = support_aabb(model, data)
    return {
        "com": [float(v) for v in com],
        "support_aabb": aabb,
        "foot": foot,
    }


def side_for_joint(joint_name: str) -> str:
    return "left" if joint_name.startswith("left_") else "right"


def role_from_world_axis(axis: np.ndarray) -> str:
    labels = ("roll_about_X_forward", "pitch_about_Y_lateral", "yaw_about_Z_vertical")
    idx = int(np.argmax(np.abs(axis)))
    return labels[idx]


def main() -> int:
    parser = argparse.ArgumentParser(description="Audit ankle/hip pitch-roll kinematic response around the selected contact standing pose.")
    parser.add_argument("--model", type=Path, required=True)
    parser.add_argument("--pose-json", type=Path, required=True)
    parser.add_argument("--delta", type=float, default=0.05)
    parser.add_argument("--out-dir", type=Path, default=Path("outputs/analysis/ankle_axis_response"))
    parser.add_argument("--doc-dir", type=Path, default=Path("docs/hardware_validation"))
    args = parser.parse_args()

    out_dir = args.out_dir.resolve()
    doc_dir = args.doc_dir.resolve()
    out_dir.mkdir(parents=True, exist_ok=True)
    doc_dir.mkdir(parents=True, exist_ok=True)

    model = mujoco.MjModel.from_xml_path(str(args.model.resolve()))
    data = mujoco.MjData(model)
    joints = hinge_joints(model)
    joint_by_name = {str(joint["name"]): joint for joint in joints}
    base_z, pose_values = load_pose(model, joints, args.pose_json.resolve())
    set_pose(model, data, joints, base_z, pose_values)
    baseline = snapshot(model, data)

    rows = []
    pair_summary = []
    for joint_name in TARGET_JOINTS:
        joint = joint_by_name[joint_name]
        side = side_for_joint(joint_name)
        body_rot = np.asarray(data.xmat[int(joint["body_id"])], dtype=np.float64).reshape(3, 3)
        axis_world = body_rot @ np.asarray(joint["axis_local"], dtype=np.float64)

        signed_rows = {}
        for sign in (1.0, -1.0):
            values = dict(pose_values)
            values[joint_name] = values[joint_name] + sign * args.delta
            set_pose(model, data, joints, base_z, values)
            snap = snapshot(model, data)
            base_foot = baseline["foot"][side]
            new_foot = snap["foot"][side]
            center_delta = np.array(new_foot["center"], dtype=np.float64) - np.array(base_foot["center"], dtype=np.float64)
            corner_dz = {
                key: float(new_foot["corners"][key][2] - base_foot["corners"][key][2])
                for key in base_foot["corners"]
            }
            front_avg = 0.5 * (corner_dz["front_left_bottom"] + corner_dz["front_right_bottom"])
            back_avg = 0.5 * (corner_dz["back_left_bottom"] + corner_dz["back_right_bottom"])
            left_avg = 0.5 * (corner_dz["front_left_bottom"] + corner_dz["back_left_bottom"])
            right_avg = 0.5 * (corner_dz["front_right_bottom"] + corner_dz["back_right_bottom"])
            row = {
                "joint": joint_name,
                "side": side,
                "sign": int(sign),
                "angle_delta_rad": sign * args.delta,
                "axis_local": " ".join(f"{float(v):.6g}" for v in joint["axis_local"]),
                "axis_world": " ".join(f"{float(v):.6g}" for v in axis_world),
                "dominant_world_role": role_from_world_axis(axis_world),
                "foot_center_dx": float(center_delta[0]),
                "foot_center_dy": float(center_delta[1]),
                "foot_center_dz": float(center_delta[2]),
                "front_minus_back_dz": float(front_avg - back_avg),
                "left_minus_right_dz": float(left_avg - right_avg),
                **{f"dz_{key}": value for key, value in corner_dz.items()},
            }
            rows.append(row)
            signed_rows[int(sign)] = row

        plus = signed_rows[1]
        minus = signed_rows[-1]
        pair_summary.append(
            {
                "joint": joint_name,
                "side": side,
                "axis_world": [float(v) for v in axis_world],
                "dominant_world_role": role_from_world_axis(axis_world),
                "front_back_sensitivity_per_rad": float((plus["front_minus_back_dz"] - minus["front_minus_back_dz"]) / (2.0 * args.delta)),
                "left_right_sensitivity_per_rad": float((plus["left_minus_right_dz"] - minus["left_minus_right_dz"]) / (2.0 * args.delta)),
                "center_sensitivity_per_rad": [
                    float((plus["foot_center_dx"] - minus["foot_center_dx"]) / (2.0 * args.delta)),
                    float((plus["foot_center_dy"] - minus["foot_center_dy"]) / (2.0 * args.delta)),
                    float((plus["foot_center_dz"] - minus["foot_center_dz"]) / (2.0 * args.delta)),
                ],
            }
        )

    csv_path = out_dir / "ankle_axis_response.csv"
    with csv_path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)

    payload = {
        "model": str(args.model.resolve()),
        "pose_json": str(args.pose_json.resolve()),
        "base_z": base_z,
        "delta": args.delta,
        "coordinate_note": "MuJoCo X=forward, Y=lateral, Z=up in the current contact/visual convention.",
        "baseline": baseline,
        "rows": rows,
        "summary": pair_summary,
    }
    json_path = out_dir / "ankle_axis_response_summary.json"
    json_path.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")

    md_path = doc_dir / "70_ankle_axis_response_audit.md"
    lines = [
        "# Ankle/Hip Axis Response Audit",
        "",
        "## 목적",
        "",
        "서기 실패가 단순 pose/gain 문제가 아닌지 확인하기 위해, 현재 기준 pose에서 hip/ankle pitch-roll joint의 실제 회전축과 발바닥 모서리 높이 변화를 감사한다.",
        "",
        "## 입력",
        "",
        f"- 모델: `{args.model.resolve()}`",
        f"- pose: `{args.pose_json.resolve()}`",
        f"- delta: `+/-{args.delta}` rad",
        "- 좌표 기준: MuJoCo X=전후, Y=좌우, Z=위",
        "",
        "## 핵심 요약",
        "",
        "| joint | side | world axis | dominant role | front-back dz/rad | left-right dz/rad | center sensitivity/rad |",
        "| --- | --- | --- | --- | ---: | ---: | --- |",
    ]
    for item in pair_summary:
        axis = " ".join(f"{float(v):+.3f}" for v in item["axis_world"])
        lines.append(
            f"| `{item['joint']}` | `{item['side']}` | `{axis}` | `{item['dominant_world_role']}` | {item['front_back_sensitivity_per_rad']:.6f} | {item['left_right_sensitivity_per_rad']:.6f} | `{item['center_sensitivity_per_rad']}` |"
        )
    lines.extend(
        [
            "",
            "## 해석 기준",
            "",
            "- MuJoCo 기준으로 X축 회전은 roll 성분이다.",
            "- MuJoCo 기준으로 Y축 회전은 pitch 성분이다.",
            "- `front-back dz/rad`가 크면 발 앞뒤 높이를 바꾸는 pitch-like 효과가 크다.",
            "- `left-right dz/rad`가 크면 발 좌우 높이를 바꾸는 roll-like 효과가 크다.",
            "",
            "## 산출물",
            "",
            f"- CSV: `{csv_path}`",
            f"- JSON: `{json_path}`",
            "",
        ]
    )
    md_path.write_text("\n".join(lines), encoding="utf-8")

    print(f"Wrote {csv_path}")
    print(f"Wrote {json_path}")
    print(f"Wrote {md_path}")
    print(json.dumps({"summary": pair_summary}, indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
