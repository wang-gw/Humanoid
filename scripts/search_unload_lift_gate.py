from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import mujoco
import numpy as np

from render_axis_aware_standing import PITCH_ROLE_ACTUATORS, ROLL_ROLE_ACTUATORS
from render_pd_standing import hinge_joint_info, load_targets, quat_to_roll_pitch_yaw
from render_right_foot_lift_sequence import right_foot_clearance
from render_weight_shift import contact_stats, foot_geom_ids
from search_right_foot_lift_trajectory import trajectory_alpha
from sweep_stabilized_standing import initialize, support_center, total_com


def simulate(
    model: mujoco.MjModel,
    joints: list[dict[str, int | str]],
    actuator_names: list[str],
    base_z: float,
    start_q: np.ndarray,
    lift_q: np.ndarray,
    args: argparse.Namespace,
    target_left_ratio: float,
    kforce: float,
) -> dict[str, float]:
    data = mujoco.MjData(model)
    initialize(model, data, joints, base_z, start_q)
    geom_to_side = foot_geom_ids(model)
    steps = int(args.duration / model.opt.timestep)
    dt = float(model.opt.timestep)

    one_leg_gate = 0.0
    clearance_gate = 0.0
    combined_gate = 0.0
    clearance_over_0 = 0.0
    clearance_over_2mm = 0.0
    clearance_over_5mm = 0.0
    right_force_low = 0.0
    no_contact = 0.0
    max_clearance = -np.inf
    final_clearance = 0.0
    max_abs_roll = 0.0
    max_abs_pitch = 0.0
    max_abs_tau = 0.0
    max_qvel = 0.0
    sat_samples = 0

    for step in range(steps + 1):
        alpha = trajectory_alpha(float(data.time), args.lift_ramp, args.lift_hold, args.return_ramp)
        target_q = (1.0 - alpha) * start_q + alpha * lift_q
        q = np.array([data.qpos[int(joint["qposadr"])] for joint in joints], dtype=np.float64)
        qd = np.array([data.qvel[int(joint["dofadr"])] for joint in joints], dtype=np.float64)
        roll, pitch, _yaw = quat_to_roll_pitch_yaw(data.qpos[3:7])
        com = total_com(model, data)
        center, half = support_center(model, data)
        com_err = (com[:2] - center) / np.maximum(half, 1e-6)
        wx = float(data.qvel[3]) if model.nv >= 6 else 0.0
        wy = float(data.qvel[4]) if model.nv >= 6 else 0.0
        roll_term = args.kp_att * roll + args.kd_att * wx + args.kcom * float(com_err[1])
        pitch_term = args.kp_att * pitch + args.kd_att * wy + args.kcom * float(com_err[0])
        stats = contact_stats(model, data, geom_to_side)
        ratio_error = target_left_ratio - stats["left_force_ratio"]
        force_term = args.force_sign * kforce * ratio_error

        tau = args.joint_kp * (target_q - q) - args.joint_kd * qd
        for idx, name in enumerate(actuator_names):
            if name in ROLL_ROLE_ACTUATORS:
                tau[idx] += args.roll_sign * roll_term
            if name in PITCH_ROLE_ACTUATORS:
                tau[idx] += args.pitch_sign * pitch_term
            tau[idx] += force_term
        tau = np.clip(tau, -args.torque_limit, args.torque_limit)
        data.ctrl[:] = tau

        clearance = right_foot_clearance(model, data)
        final_clearance = clearance
        max_clearance = max(max_clearance, clearance)
        abs_roll = abs(float(roll))
        max_abs_roll = max(max_abs_roll, abs_roll)
        max_abs_pitch = max(max_abs_pitch, abs(float(pitch)))
        max_abs_tau = max(max_abs_tau, float(np.max(np.abs(data.actuator_force))) if model.nu else 0.0)
        max_qvel = max(max_qvel, float(np.linalg.norm(data.qvel)))
        sat_samples += int(np.any(np.abs(tau) >= args.torque_limit * 0.999))

        one_leg_ok = stats["right_force"] <= args.right_force_gate and stats["right_contacts"] <= args.right_contact_gate and abs_roll <= args.max_roll_gate
        clearance_ok = clearance >= args.target_clearance and abs_roll <= args.max_roll_gate
        if one_leg_ok:
            one_leg_gate += dt
        if clearance_ok:
            clearance_gate += dt
        if one_leg_ok and clearance >= args.target_clearance:
            combined_gate += dt
        if clearance > 0.0:
            clearance_over_0 += dt
        if clearance >= 0.002:
            clearance_over_2mm += dt
        if clearance >= 0.005:
            clearance_over_5mm += dt
        if stats["right_force"] <= args.right_force_gate:
            right_force_low += dt
        if stats["right_contacts"] <= args.right_contact_gate:
            no_contact += dt

        if step < steps:
            mujoco.mj_step(model, data)

    final_stats = contact_stats(model, data, geom_to_side)
    final_roll, final_pitch, final_yaw = quat_to_roll_pitch_yaw(data.qpos[3:7])
    return {
        "target_left_ratio": float(target_left_ratio),
        "kforce": float(kforce),
        "one_leg_gate_s": float(one_leg_gate),
        "clearance_gate_s": float(clearance_gate),
        "combined_gate_s": float(combined_gate),
        "clearance_over_0_s": float(clearance_over_0),
        "clearance_over_2mm_s": float(clearance_over_2mm),
        "clearance_over_5mm_s": float(clearance_over_5mm),
        "right_force_low_s": float(right_force_low),
        "no_contact_s": float(no_contact),
        "max_clearance_m": float(max_clearance),
        "final_clearance_m": float(final_clearance),
        "max_abs_roll": float(max_abs_roll),
        "max_abs_pitch": float(max_abs_pitch),
        "max_abs_tau": float(max_abs_tau),
        "max_qvel_norm": float(max_qvel),
        "saturation_fraction": sat_samples / max(steps + 1, 1),
        "final_left_ratio": float(final_stats["left_force_ratio"]),
        "final_right_force": float(final_stats["right_force"]),
        "final_right_contacts": float(final_stats["right_contacts"]),
        "final_roll": float(final_roll),
        "final_pitch": float(final_pitch),
        "final_yaw": float(final_yaw),
    }


def score(row: dict[str, float], args: argparse.Namespace) -> float:
    target_deficit = max(0.0, args.target_gate_time - row["combined_gate_s"])
    return (
        500.0 * target_deficit
        - 60.0 * row["combined_gate_s"]
        - 20.0 * row["clearance_gate_s"]
        - 10.0 * row["one_leg_gate_s"]
        + 180.0 * max(0.0, row["max_abs_roll"] - args.max_roll_gate)
        + 100.0 * row["saturation_fraction"]
        + 0.05 * row["max_abs_tau"]
    )


def main() -> int:
    parser = argparse.ArgumentParser(description="Search unload plus lift gate controller parameters.")
    parser.add_argument("--model", type=Path, required=True)
    parser.add_argument("--start-pose", type=Path, required=True)
    parser.add_argument("--lift-poses", nargs="+", type=Path, required=True)
    parser.add_argument("--out-dir", type=Path, required=True)
    parser.add_argument("--duration", type=float, default=8.0)
    parser.add_argument("--lift-ramp", type=float, default=2.0)
    parser.add_argument("--lift-hold", type=float, default=2.0)
    parser.add_argument("--return-ramp", type=float, default=2.0)
    parser.add_argument("--target-gate-time", type=float, default=0.5)
    parser.add_argument("--target-clearance", type=float, default=0.002)
    parser.add_argument("--right-force-gate", type=float, default=5.0)
    parser.add_argument("--right-contact-gate", type=float, default=0.0)
    parser.add_argument("--max-roll-gate", type=float, default=0.12)
    parser.add_argument("--joint-kp", type=float, default=20.0)
    parser.add_argument("--joint-kd", type=float, default=12.0)
    parser.add_argument("--torque-limit", type=float, default=30.0)
    parser.add_argument("--kp-att", type=float, default=0.0)
    parser.add_argument("--kd-att", type=float, default=1.0)
    parser.add_argument("--kcom", type=float, default=1.0)
    parser.add_argument("--roll-sign", type=float, default=-1.0)
    parser.add_argument("--pitch-sign", type=float, default=1.0)
    parser.add_argument("--force-sign", type=float, default=1.0)
    args = parser.parse_args()

    out_dir = args.out_dir.resolve()
    out_dir.mkdir(parents=True, exist_ok=True)
    model = mujoco.MjModel.from_xml_path(str(args.model.resolve()))
    joints = hinge_joint_info(model)
    actuator_names = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, idx) or f"actuator_{idx}" for idx in range(model.nu)]
    base_z, start_q = load_targets(model, joints, args.start_pose.resolve())

    rows: list[dict[str, float | str]] = []
    for lift_pose in args.lift_poses:
        _lift_base_z, lift_q = load_targets(model, joints, lift_pose.resolve())
        for target_left_ratio in (0.88, 0.90, 0.92, 0.94, 0.96):
            for kforce in (8.0, 10.0, 12.0, 14.0, 16.0):
                metrics = simulate(model, joints, actuator_names, base_z, start_q, lift_q, args, target_left_ratio, kforce)
                row = {
                    "lift_pose": str(lift_pose),
                    **metrics,
                }
                row["score"] = score(metrics, args)
                rows.append(row)
    rows.sort(key=lambda item: float(item["score"]))
    csv_path = out_dir / "unload_lift_gate_candidates.csv"
    with csv_path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)
    summary = {
        "model": str(args.model.resolve()),
        "start_pose": str(args.start_pose.resolve()),
        "lift_poses": [str(path.resolve()) for path in args.lift_poses],
        "duration": args.duration,
        "target_gate_time": args.target_gate_time,
        "target_clearance": args.target_clearance,
        "best": rows[0],
        "csv": str(csv_path),
        "num_candidates": len(rows),
    }
    summary_path = out_dir / "unload_lift_gate_search_summary.json"
    summary_path.write_text(json.dumps(summary, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(json.dumps(summary, indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
