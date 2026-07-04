from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path

import mujoco
import numpy as np

from render_pd_standing import hinge_joint_info, load_targets
from sweep_stabilized_standing import initialize


OBSERVED_BODIES = ("base_link", "foot_L_1", "foot_R_v1_1", "thighJ_L_1", "thighJ_R_1")


def quat_to_roll_pitch_yaw(q: np.ndarray) -> tuple[float, float, float]:
    w, x, y, z = [float(v) for v in q]
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = 2.0 * (w * y - z * x)
    pitch = math.copysign(math.pi / 2.0, sinp) if abs(sinp) >= 1.0 else math.asin(sinp)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def body_rpy(model: mujoco.MjModel, data: mujoco.MjData, body_name: str) -> tuple[float, float, float]:
    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body_name)
    return quat_to_roll_pitch_yaw(np.asarray(data.xquat[body_id]))


def joint_axis_world(model: mujoco.MjModel, data: mujoco.MjData, joint_id: int) -> np.ndarray:
    body_id = int(model.jnt_bodyid[joint_id])
    xmat = np.asarray(data.xmat[body_id], dtype=np.float64).reshape(3, 3)
    axis = np.asarray(model.jnt_axis[joint_id], dtype=np.float64)
    world = xmat @ axis
    norm = np.linalg.norm(world)
    return world / norm if norm > 0.0 else world


def dominant_axis_role(axis: np.ndarray) -> str:
    idx = int(np.argmax(np.abs(axis)))
    if idx == 0:
        return "roll_about_X_forward"
    if idx == 1:
        return "pitch_about_Y_lateral"
    return "yaw_about_Z_vertical"


def simulate_case(
    model: mujoco.MjModel,
    joints: list[dict[str, int | str]],
    actuator_names: list[str],
    base_z: float,
    target_q: np.ndarray,
    actuator_id: int,
    sign: float,
    args: argparse.Namespace,
) -> tuple[dict[str, object], list[dict[str, float]]]:
    data = mujoco.MjData(model)
    initialize(model, data, joints, base_z, target_q)
    data.qvel[:] = 0.0
    mujoco.mj_forward(model, data)

    joint_id = int(model.actuator_trnid[actuator_id, 0])
    joint_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, joint_id) or f"joint_{joint_id}"
    actuator_name = actuator_names[actuator_id]
    qposadr = int(model.jnt_qposadr[joint_id])
    dofadr = int(model.jnt_dofadr[joint_id])
    axis_world = joint_axis_world(model, data, joint_id)

    initial_base = quat_to_roll_pitch_yaw(data.qpos[3:7])
    initial_body = {name: body_rpy(model, data, name) for name in OBSERVED_BODIES}
    initial_q = float(data.qpos[qposadr])

    steps = int(args.duration / model.opt.timestep)
    impulse_start_step = int(args.impulse_start / model.opt.timestep)
    impulse_end_step = int((args.impulse_start + args.impulse_duration) / model.opt.timestep)
    rows: list[dict[str, float]] = []

    for step in range(steps + 1):
        tau = np.zeros(model.nu, dtype=np.float64)
        if impulse_start_step <= step < impulse_end_step:
            tau[actuator_id] = sign * args.impulse_torque
        data.ctrl[:] = np.clip(tau, -args.torque_limit, args.torque_limit)

        if step % max(args.log_every, 1) == 0 or step == steps:
            base_roll, base_pitch, base_yaw = quat_to_roll_pitch_yaw(data.qpos[3:7])
            row = {
                "step": float(step),
                "time": float(data.time),
                "actuator": float(actuator_id),
                "sign": sign,
                "base_x": float(data.qpos[0]),
                "base_y": float(data.qpos[1]),
                "base_z": float(data.qpos[2]),
                "base_roll": base_roll,
                "base_pitch": base_pitch,
                "base_yaw": base_yaw,
                "joint_q": float(data.qpos[qposadr]),
                "joint_qd": float(data.qvel[dofadr]),
                "qvel_norm": float(np.linalg.norm(data.qvel)),
            }
            for body_name in OBSERVED_BODIES:
                roll, pitch, yaw = body_rpy(model, data, body_name)
                row[f"{body_name}_roll"] = roll
                row[f"{body_name}_pitch"] = pitch
                row[f"{body_name}_yaw"] = yaw
            rows.append(row)

        if step < steps:
            mujoco.mj_step(model, data)

    final_base = quat_to_roll_pitch_yaw(data.qpos[3:7])
    final_body = {name: body_rpy(model, data, name) for name in OBSERVED_BODIES}
    summary = {
        "actuator": actuator_name,
        "joint": joint_name,
        "sign": sign,
        "axis_world": [float(v) for v in axis_world],
        "dominant_axis_role": dominant_axis_role(axis_world),
        "initial_joint_q": initial_q,
        "final_joint_q": float(data.qpos[qposadr]),
        "delta_joint_q": float(data.qpos[qposadr] - initial_q),
        "delta_base_roll": float(final_base[0] - initial_base[0]),
        "delta_base_pitch": float(final_base[1] - initial_base[1]),
        "delta_base_yaw": float(final_base[2] - initial_base[2]),
        "max_qvel_norm": max(float(row["qvel_norm"]) for row in rows),
        "left_foot_delta_roll": float(final_body["foot_L_1"][0] - initial_body["foot_L_1"][0]),
        "left_foot_delta_pitch": float(final_body["foot_L_1"][1] - initial_body["foot_L_1"][1]),
        "right_foot_delta_roll": float(final_body["foot_R_v1_1"][0] - initial_body["foot_R_v1_1"][0]),
        "right_foot_delta_pitch": float(final_body["foot_R_v1_1"][1] - initial_body["foot_R_v1_1"][1]),
    }
    return summary, rows


def main() -> int:
    parser = argparse.ArgumentParser(description="Apply small free-space torque impulses to every actuator.")
    parser.add_argument("--model", type=Path, required=True)
    parser.add_argument("--pose-json", type=Path, required=True)
    parser.add_argument("--base-z", type=float, default=1.0)
    parser.add_argument("--duration", type=float, default=0.12)
    parser.add_argument("--impulse-start", type=float, default=0.02)
    parser.add_argument("--impulse-duration", type=float, default=0.03)
    parser.add_argument("--impulse-torque", type=float, default=2.0)
    parser.add_argument("--torque-limit", type=float, default=100.0)
    parser.add_argument("--log-every", type=int, default=2)
    parser.add_argument("--out-dir", type=Path, default=Path("outputs/analysis/all_joint_free_impulse_response"))
    args = parser.parse_args()

    out_dir = args.out_dir.resolve()
    out_dir.mkdir(parents=True, exist_ok=True)

    model = mujoco.MjModel.from_xml_path(str(args.model.resolve()))
    model.opt.gravity[:] = 0.0
    joints = hinge_joint_info(model)
    actuator_names = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, idx) or f"actuator_{idx}" for idx in range(model.nu)]
    _, target_q = load_targets(model, joints, args.pose_json.resolve())

    summaries = []
    all_rows = []
    for actuator_id in range(model.nu):
        for sign in (1.0, -1.0):
            summary, rows = simulate_case(model, joints, actuator_names, args.base_z, target_q, actuator_id, sign, args)
            case_id = len(summaries)
            for row in rows:
                row["case"] = float(case_id)
            summaries.append(summary)
            all_rows.extend(rows)

    csv_path = out_dir / "all_joint_free_impulse_response.csv"
    with csv_path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(all_rows[0].keys()))
        writer.writeheader()
        writer.writerows(all_rows)

    summary_csv = out_dir / "all_joint_free_impulse_response_summary.csv"
    with summary_csv.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(summaries[0].keys()))
        writer.writeheader()
        writer.writerows(summaries)

    payload = {
        "model": str(args.model.resolve()),
        "pose_json": str(args.pose_json.resolve()),
        "gravity": [float(v) for v in model.opt.gravity],
        "base_z": args.base_z,
        "duration": args.duration,
        "impulse_start": args.impulse_start,
        "impulse_duration": args.impulse_duration,
        "impulse_torque": args.impulse_torque,
        "summaries": summaries,
        "csv": str(csv_path),
        "summary_csv": str(summary_csv),
    }
    json_path = out_dir / "all_joint_free_impulse_response_summary.json"
    json_path.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(json.dumps(payload, indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
