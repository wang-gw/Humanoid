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
from search_right_foot_lift_pose import RIGHT_LIFT_JOINTS, right_foot_clearance
from sweep_stabilized_standing import initialize, support_center, total_com


def smoothstep(value: float) -> float:
    x = min(max(value, 0.0), 1.0)
    return x * x * (3.0 - 2.0 * x)


def trajectory_alpha(time: float, lift_ramp: float, lift_hold: float, return_ramp: float) -> float:
    if time < lift_ramp:
        return smoothstep(time / max(lift_ramp, 1e-9))
    if time < lift_ramp + lift_hold:
        return 1.0
    return 1.0 - smoothstep((time - lift_ramp - lift_hold) / max(return_ramp, 1e-9))


def simulate(
    model: mujoco.MjModel,
    joints: list[dict[str, int | str]],
    actuator_names: list[str],
    base_z: float,
    start_q: np.ndarray,
    lift_q: np.ndarray,
    args: argparse.Namespace,
) -> dict[str, float]:
    data = mujoco.MjData(model)
    initialize(model, data, joints, base_z, start_q)
    geom_to_side = foot_geom_ids(model)
    steps = int(args.duration / model.opt.timestep)
    dt = float(model.opt.timestep)

    clearance_time = 0.0
    low_force_time = 0.0
    low_contact_time = 0.0
    stable_clearance_time = 0.0
    max_clearance = -np.inf
    final_clearance = 0.0
    max_abs_roll = 0.0
    max_qvel = 0.0
    max_abs_tau = 0.0
    max_contact = 0.0
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
        ratio_error = args.target_left_ratio - stats["left_force_ratio"]
        force_term = args.force_sign * args.kforce * ratio_error

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
        qvel = float(np.linalg.norm(data.qvel))
        max_qvel = max(max_qvel, qvel)
        max_contact = max(max_contact, stats["normal_force"])
        max_abs_tau = max(max_abs_tau, float(np.max(np.abs(data.actuator_force))) if model.nu else 0.0)
        sat_samples += int(np.any(np.abs(tau) >= args.torque_limit * 0.999))

        if clearance >= args.target_clearance:
            clearance_time += dt
            if abs_roll <= args.max_roll_gate:
                stable_clearance_time += dt
        if stats["right_force"] <= args.right_force_gate:
            low_force_time += dt
        if stats["right_contacts"] <= args.right_contact_gate:
            low_contact_time += dt

        if step < steps:
            mujoco.mj_step(model, data)

    final_stats = contact_stats(model, data, geom_to_side)
    final_roll, final_pitch, final_yaw = quat_to_roll_pitch_yaw(data.qpos[3:7])
    return {
        "clearance_time_s": float(clearance_time),
        "stable_clearance_time_s": float(stable_clearance_time),
        "low_force_time_s": float(low_force_time),
        "low_contact_time_s": float(low_contact_time),
        "max_clearance_m": float(max_clearance),
        "final_clearance_m": float(final_clearance),
        "final_left_ratio": final_stats["left_force_ratio"],
        "final_right_force": final_stats["right_force"],
        "final_right_contacts": final_stats["right_contacts"],
        "final_left_contacts": final_stats["left_contacts"],
        "final_roll": float(final_roll),
        "final_pitch": float(final_pitch),
        "final_yaw": float(final_yaw),
        "max_abs_roll": float(max_abs_roll),
        "max_qvel_norm": float(max_qvel),
        "max_abs_tau": float(max_abs_tau),
        "max_contact_force": float(max_contact),
        "saturation_fraction": sat_samples / max(steps + 1, 1),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description="Search a right-foot lift trajectory optimized for clearance duration.")
    parser.add_argument("--model", type=Path, required=True)
    parser.add_argument("--seed-pose", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--out-dir", type=Path, required=True)
    parser.add_argument("--samples", type=int, default=240)
    parser.add_argument("--seed", type=int, default=601)
    parser.add_argument("--duration", type=float, default=8.0)
    parser.add_argument("--lift-ramp", type=float, default=2.0)
    parser.add_argument("--lift-hold", type=float, default=2.0)
    parser.add_argument("--return-ramp", type=float, default=2.0)
    parser.add_argument("--target-clearance", type=float, default=0.002)
    parser.add_argument("--target-clearance-time", type=float, default=0.5)
    parser.add_argument("--target-left-ratio", type=float, default=0.78)
    parser.add_argument("--right-force-gate", type=float, default=15.0)
    parser.add_argument("--right-contact-gate", type=float, default=1.0)
    parser.add_argument("--max-roll-gate", type=float, default=0.18)
    parser.add_argument("--joint-kp", type=float, default=20.0)
    parser.add_argument("--joint-kd", type=float, default=12.0)
    parser.add_argument("--torque-limit", type=float, default=30.0)
    parser.add_argument("--kp-att", type=float, default=0.0)
    parser.add_argument("--kd-att", type=float, default=1.0)
    parser.add_argument("--kcom", type=float, default=1.0)
    parser.add_argument("--roll-sign", type=float, default=-1.0)
    parser.add_argument("--pitch-sign", type=float, default=1.0)
    parser.add_argument("--kforce", type=float, default=5.0)
    parser.add_argument("--force-sign", type=float, default=1.0)
    args = parser.parse_args()

    out_dir = args.out_dir.resolve()
    out_dir.mkdir(parents=True, exist_ok=True)
    model = mujoco.MjModel.from_xml_path(str(args.model.resolve()))
    joints = hinge_joint_info(model)
    joint_names = [str(joint["name"]) for joint in joints]
    actuator_names = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, idx) or f"actuator_{idx}" for idx in range(model.nu)]
    base_z, seed_q = load_targets(model, joints, args.seed_pose.resolve())
    rng = np.random.default_rng(args.seed)

    seed_patterns = [
        {"right_hip_roll": -0.10, "right_hip_pitch": -0.04, "right_knee_pitch": -0.08, "right_ankle_pitch": 0.07, "right_ankle_roll": 0.14},
        {"right_hip_roll": -0.12, "right_hip_pitch": -0.01, "right_knee_pitch": -0.06, "right_ankle_pitch": 0.05, "right_ankle_roll": 0.18},
        {"right_hip_roll": -0.08, "right_hip_pitch": -0.10, "right_knee_pitch": -0.16, "right_ankle_pitch": 0.14, "right_ankle_roll": 0.10},
    ]
    candidate_offsets = [dict.fromkeys(RIGHT_LIFT_JOINTS, 0.0), *seed_patterns]
    for _ in range(args.samples):
        candidate_offsets.append({name: float(rng.uniform(lo, hi)) for name, (lo, hi) in RIGHT_LIFT_JOINTS.items()})

    rows = []
    best = None
    for offsets in candidate_offsets:
        lift_q = seed_q.copy()
        for name, value in offsets.items():
            if name in joint_names:
                lift_q[joint_names.index(name)] += float(value)
        metrics = simulate(model, joints, actuator_names, base_z, seed_q, lift_q, args)
        stable_time_error = max(0.0, args.target_clearance_time - metrics["stable_clearance_time_s"])
        score = (
            400.0 * stable_time_error
            - 40.0 * metrics["stable_clearance_time_s"]
            - 5.0 * metrics["low_force_time_s"]
            + 120.0 * max(0.0, metrics["max_abs_roll"] - args.max_roll_gate)
            + 80.0 * metrics["saturation_fraction"]
            + 0.04 * metrics["max_abs_tau"]
            + 20.0 * max(0.0, metrics["final_right_force"] - 25.0) / 90.0
        )
        row = {"score": float(score), **offsets, **metrics}
        rows.append(row)
        if best is None or row["score"] < best["score"]:
            best = row

    assert best is not None
    best_q = seed_q.copy()
    for name in RIGHT_LIFT_JOINTS:
        if name in joint_names:
            best_q[joint_names.index(name)] += float(best[name])
    seed_payload = json.loads(args.seed_pose.read_text(encoding="utf-8"))
    targets = dict(seed_payload.get("joint_targets", {}))
    for idx, name in enumerate(joint_names):
        targets[name] = float(best_q[idx])
    search_args = {key: (str(value) if isinstance(value, Path) else value) for key, value in vars(args).items()}
    out_payload = {
        "base_z": float(base_z),
        "joint_targets": targets,
        "source": {
            "seed_pose": str(args.seed_pose.resolve()),
            "model": str(args.model.resolve()),
            "search": search_args,
            "best": best,
        },
    }
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(out_payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")

    csv_path = out_dir / "right_foot_lift_trajectory_candidates.csv"
    with csv_path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(sorted(rows, key=lambda row: row["score"]))
    summary = {"best": best, "out_pose": str(args.out.resolve()), "csv": str(csv_path), "samples": len(rows)}
    (out_dir / "right_foot_lift_trajectory_search_summary.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(json.dumps(summary, indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
