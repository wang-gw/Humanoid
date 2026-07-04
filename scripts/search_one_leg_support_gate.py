from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import mujoco
import numpy as np

from render_axis_aware_standing import PITCH_ROLE_ACTUATORS, ROLL_ROLE_ACTUATORS
from render_pd_standing import hinge_joint_info, load_targets, quat_to_roll_pitch_yaw
from render_weight_shift import contact_stats, foot_geom_ids
from sweep_stabilized_standing import initialize, support_center, total_com


def simulate(
    model: mujoco.MjModel,
    joints: list[dict[str, int | str]],
    actuator_names: list[str],
    base_z: float,
    target_q: np.ndarray,
    args: argparse.Namespace,
    target_left_ratio: float,
    kforce: float,
    force_role: str,
    force_on: str,
    force_side_mode: str,
) -> dict[str, float | str]:
    data = mujoco.MjData(model)
    initialize(model, data, joints, base_z, target_q)
    geom_to_side = foot_geom_ids(model)
    steps = int(args.duration / model.opt.timestep)
    dt = float(model.opt.timestep)

    gate_time = 0.0
    low_force_time = 0.0
    no_contact_time = 0.0
    low_roll_time = 0.0
    max_abs_roll = 0.0
    max_abs_pitch = 0.0
    max_abs_tau = 0.0
    max_qvel = 0.0
    sat_samples = 0

    for step in range(steps + 1):
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
            role_match = (force_role in ("roll", "both") and name in ROLL_ROLE_ACTUATORS) or (
                force_role in ("pitch", "both") and name in PITCH_ROLE_ACTUATORS
            )
            group_match = force_on == "all" or (force_on == "hip" and "hip" in name) or (force_on == "ankle" and "ankle" in name)
            if role_match and group_match:
                side_scale = 1.0
                if force_side_mode == "opposite":
                    side_scale = 1.0 if "_left_" in name else -1.0
                tau[idx] += side_scale * force_term

        tau = np.clip(tau, -args.torque_limit, args.torque_limit)
        data.ctrl[:] = tau

        abs_roll = abs(float(roll))
        abs_pitch = abs(float(pitch))
        max_abs_roll = max(max_abs_roll, abs_roll)
        max_abs_pitch = max(max_abs_pitch, abs_pitch)
        max_qvel = max(max_qvel, float(np.linalg.norm(data.qvel)))
        max_abs_tau = max(max_abs_tau, float(np.max(np.abs(data.actuator_force))) if model.nu else 0.0)
        sat_samples += int(np.any(np.abs(tau) >= args.torque_limit * 0.999))

        right_force_ok = stats["right_force"] <= args.right_force_gate
        no_contact_ok = stats["right_contacts"] <= args.right_contact_gate
        roll_ok = abs_roll <= args.max_roll_gate
        if right_force_ok:
            low_force_time += dt
        if no_contact_ok:
            no_contact_time += dt
        if roll_ok:
            low_roll_time += dt
        if right_force_ok and no_contact_ok and roll_ok:
            gate_time += dt

        if step < steps:
            mujoco.mj_step(model, data)

    final_stats = contact_stats(model, data, geom_to_side)
    final_roll, final_pitch, final_yaw = quat_to_roll_pitch_yaw(data.qpos[3:7])
    return {
        "target_left_ratio": float(target_left_ratio),
        "kforce": float(kforce),
        "force_role": force_role,
        "force_on": force_on,
        "force_side_mode": force_side_mode,
        "gate_time_s": float(gate_time),
        "low_force_time_s": float(low_force_time),
        "no_contact_time_s": float(no_contact_time),
        "low_roll_time_s": float(low_roll_time),
        "max_abs_roll": float(max_abs_roll),
        "max_abs_pitch": float(max_abs_pitch),
        "max_abs_tau": float(max_abs_tau),
        "max_qvel_norm": float(max_qvel),
        "saturation_fraction": sat_samples / max(steps + 1, 1),
        "final_left_ratio": float(final_stats["left_force_ratio"]),
        "final_right_force": float(final_stats["right_force"]),
        "final_right_contacts": float(final_stats["right_contacts"]),
        "final_left_contacts": float(final_stats["left_contacts"]),
        "final_roll": float(final_roll),
        "final_pitch": float(final_pitch),
        "final_yaw": float(final_yaw),
    }


def score(row: dict[str, float | str], args: argparse.Namespace) -> float:
    gate_deficit = max(0.0, args.target_gate_time - float(row["gate_time_s"]))
    return (
        500.0 * gate_deficit
        - 20.0 * float(row["gate_time_s"])
        - 5.0 * float(row["low_force_time_s"])
        - 3.0 * float(row["no_contact_time_s"])
        + 160.0 * max(0.0, float(row["max_abs_roll"]) - args.max_roll_gate)
        + 100.0 * float(row["saturation_fraction"])
        + 0.05 * float(row["max_abs_tau"])
        + 0.2 * float(row["max_qvel_norm"])
    )


def main() -> int:
    parser = argparse.ArgumentParser(description="Search one-leg support gate using force-ratio feedback.")
    parser.add_argument("--model", type=Path, required=True)
    parser.add_argument("--pose", type=Path, required=True)
    parser.add_argument("--out-dir", type=Path, required=True)
    parser.add_argument("--duration", type=float, default=6.0)
    parser.add_argument("--target-gate-time", type=float, default=0.5)
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
    parser.add_argument("--fast", action="store_true", help="Evaluate a small coarse candidate set first.")
    parser.add_argument("--narrow", action="store_true", help="Evaluate candidates around the current best coarse result.")
    args = parser.parse_args()

    out_dir = args.out_dir.resolve()
    out_dir.mkdir(parents=True, exist_ok=True)
    model = mujoco.MjModel.from_xml_path(str(args.model.resolve()))
    joints = hinge_joint_info(model)
    actuator_names = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, idx) or f"actuator_{idx}" for idx in range(model.nu)]
    base_z, target_q = load_targets(model, joints, args.pose.resolve())

    rows: list[dict[str, float | str]] = []
    if args.narrow:
        target_left_ratios = (0.88, 0.90, 0.92, 0.94)
        kforces = (10.0, 12.0, 14.0, 16.0)
        force_roles = ("both",)
        force_ons = ("all",)
        force_side_modes = ("opposite",)
    elif args.fast:
        target_left_ratios = (0.82, 0.90, 0.98)
        kforces = (5.0, 12.0)
        force_roles = ("roll", "both")
        force_ons = ("all",)
        force_side_modes = ("same", "opposite")
    else:
        target_left_ratios = (0.78, 0.82, 0.86, 0.90, 0.94, 0.98)
        kforces = (3.0, 5.0, 8.0, 12.0, 16.0)
        force_roles = ("roll", "both")
        force_ons = ("all", "hip", "ankle")
        force_side_modes = ("same", "opposite")

    for target_left_ratio in target_left_ratios:
        for kforce in kforces:
            for force_role in force_roles:
                for force_on in force_ons:
                    for force_side_mode in force_side_modes:
                        row = simulate(
                            model,
                            joints,
                            actuator_names,
                            base_z,
                            target_q,
                            args,
                            target_left_ratio,
                            kforce,
                            force_role,
                            force_on,
                            force_side_mode,
                        )
                        row["score"] = score(row, args)
                        rows.append(row)

    rows.sort(key=lambda item: float(item["score"]))
    csv_path = out_dir / "one_leg_support_gate_candidates.csv"
    with csv_path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)
    summary = {
        "model": str(args.model.resolve()),
        "pose": str(args.pose.resolve()),
        "duration": args.duration,
        "gate": {
            "target_gate_time_s": args.target_gate_time,
            "right_force_gate_n": args.right_force_gate,
            "right_contact_gate": args.right_contact_gate,
            "max_roll_gate_rad": args.max_roll_gate,
        },
        "best": rows[0],
        "csv": str(csv_path),
        "num_candidates": len(rows),
    }
    summary_path = out_dir / "one_leg_support_gate_summary.json"
    summary_path.write_text(json.dumps(summary, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(json.dumps(summary, indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
