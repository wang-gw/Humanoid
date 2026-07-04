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


def smoothstep(value: float) -> float:
    x = min(max(value, 0.0), 1.0)
    return x * x * (3.0 - 2.0 * x)


def simulate_transition(
    model: mujoco.MjModel,
    joints: list[dict[str, int | str]],
    actuator_names: list[str],
    start_base_z: float,
    start_q: np.ndarray,
    end_base_z: float,
    end_q: np.ndarray,
    args: argparse.Namespace,
) -> dict[str, float]:
    data = mujoco.MjData(model)
    initialize(model, data, joints, start_base_z, start_q)
    geom_to_side = foot_geom_ids(model)
    steps = int(args.duration / model.opt.timestep)
    max_abs_tau = 0.0
    max_qvel = 0.0
    max_contact = 0.0
    max_abs_roll = 0.0
    max_abs_pitch = 0.0
    max_abs_yaw = 0.0
    min_left_contacts = float("inf")
    min_right_contacts = float("inf")
    min_normal_force = float("inf")
    sat_samples = 0
    contact_samples = 0
    final: dict[str, float] = {}

    for step in range(steps + 1):
        alpha = smoothstep(float(data.time) / max(args.ramp, 1e-9))
        target_q = (1.0 - alpha) * start_q + alpha * end_q
        q = np.array([data.qpos[int(joint["qposadr"])] for joint in joints], dtype=np.float64)
        qd = np.array([data.qvel[int(joint["dofadr"])] for joint in joints], dtype=np.float64)
        roll, pitch, yaw = quat_to_roll_pitch_yaw(data.qpos[3:7])
        com = total_com(model, data)
        center, half = support_center(model, data)
        com_err = (com[:2] - center) / np.maximum(half, 1e-6)
        wx = float(data.qvel[3]) if model.nv >= 6 else 0.0
        wy = float(data.qvel[4]) if model.nv >= 6 else 0.0
        roll_term = args.kp_att * roll + args.kd_att * wx + args.kcom * float(com_err[1])
        pitch_term = args.kp_att * pitch + args.kd_att * wy + args.kcom * float(com_err[0])

        tau = args.joint_kp * (target_q - q) - args.joint_kd * qd
        for idx, name in enumerate(actuator_names):
            if name in ROLL_ROLE_ACTUATORS:
                tau[idx] += args.roll_sign * roll_term
            if name in PITCH_ROLE_ACTUATORS:
                tau[idx] += args.pitch_sign * pitch_term
        tau = np.clip(tau, -args.torque_limit, args.torque_limit)
        data.ctrl[:] = tau

        stats = contact_stats(model, data, geom_to_side)
        qvel = float(np.linalg.norm(data.qvel))
        max_abs_tau = max(max_abs_tau, float(np.max(np.abs(data.actuator_force))) if model.nu else 0.0)
        max_qvel = max(max_qvel, qvel)
        max_contact = max(max_contact, stats["normal_force"])
        max_abs_roll = max(max_abs_roll, abs(float(roll)))
        max_abs_pitch = max(max_abs_pitch, abs(float(pitch)))
        max_abs_yaw = max(max_abs_yaw, abs(float(yaw)))
        min_left_contacts = min(min_left_contacts, float(stats["left_contacts"]))
        min_right_contacts = min(min_right_contacts, float(stats["right_contacts"]))
        min_normal_force = min(min_normal_force, float(stats["normal_force"]))
        sat_samples += int(np.any(np.abs(tau) >= args.torque_limit * 0.999))
        contact_samples += int(data.ncon > 0)
        final = {
            "final_left_force_ratio": float(stats["left_force_ratio"]),
            "final_right_force_ratio": float(stats["right_force_ratio"]),
            "final_left_force": float(stats["left_force"]),
            "final_right_force": float(stats["right_force"]),
            "final_roll_rad": float(roll),
            "final_pitch_rad": float(pitch),
            "final_yaw_rad": float(yaw),
            "final_base_z": float(data.qpos[2]),
            "final_qvel_norm": qvel,
            "final_contact_force": float(stats["normal_force"]),
            "final_left_contacts": float(stats["left_contacts"]),
            "final_right_contacts": float(stats["right_contacts"]),
            "final_com_err_x": float(com_err[0]),
            "final_com_err_y": float(com_err[1]),
        }
        if step < steps:
            mujoco.mj_step(model, data)

    contact_fraction = contact_samples / max(steps + 1, 1)
    saturation_fraction = sat_samples / max(steps + 1, 1)
    ratio_error = abs(final["final_left_force_ratio"] - args.target_left_ratio)
    score = (
        args.ratio_weight * ratio_error
        + args.final_roll_weight * abs(final["final_roll_rad"])
        + args.max_roll_weight * max_abs_roll
        + args.pitch_weight * abs(final["final_pitch_rad"])
        + args.yaw_weight * abs(final["final_yaw_rad"])
        + args.qvel_weight * final["final_qvel_norm"]
        + args.max_qvel_weight * max_qvel
        + args.contact_loss_weight * (1.0 - contact_fraction)
        + args.saturation_weight * saturation_fraction
        + args.torque_weight * max_abs_tau
        + args.left_contact_weight * max(0.0, args.min_left_contacts - final["final_left_contacts"])
        + args.right_contact_weight * max(0.0, args.min_right_contacts - final["final_right_contacts"])
        + args.min_left_contact_weight * max(0.0, args.min_left_contacts - min_left_contacts)
        + args.min_right_contact_weight * max(0.0, args.min_right_contacts - min_right_contacts)
        + args.normal_force_weight * max(0.0, args.min_normal_force - final["final_contact_force"])
        + args.right_force_weight * max(0.0, final["final_right_force"] - args.max_right_force)
        + args.left_force_weight * max(0.0, args.min_left_force - final["final_left_force"])
        + args.max_roll_limit_weight * max(0.0, max_abs_roll - args.max_roll_limit)
        + args.final_roll_limit_weight * max(0.0, abs(final["final_roll_rad"]) - args.final_roll_limit)
    )
    return {
        "score": float(score),
        "ratio_error": float(ratio_error),
        "target_left_ratio": float(args.target_left_ratio),
        **final,
        "max_abs_roll_rad": float(max_abs_roll),
        "max_abs_pitch_rad": float(max_abs_pitch),
        "max_abs_yaw_rad": float(max_abs_yaw),
        "min_left_contacts": float(min_left_contacts),
        "min_right_contacts": float(min_right_contacts),
        "min_normal_force": float(min_normal_force),
        "max_qvel_norm": float(max_qvel),
        "max_contact_force": float(max_contact),
        "max_abs_tau": float(max_abs_tau),
        "contact_fraction": float(contact_fraction),
        "saturation_fraction": float(saturation_fraction),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description="Search an end pose by scoring the full transition from a start pose.")
    parser.add_argument("--model", type=Path, required=True)
    parser.add_argument("--start-pose", type=Path, required=True)
    parser.add_argument("--seed-end-pose", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--out-dir", type=Path, required=True)
    parser.add_argument("--target-left-ratio", type=float, default=0.62)
    parser.add_argument("--samples", type=int, default=180)
    parser.add_argument("--iterations", type=int, default=3)
    parser.add_argument("--elite", type=int, default=18)
    parser.add_argument("--seed", type=int, default=211)
    parser.add_argument("--sigma", type=float, default=0.04)
    parser.add_argument("--min-sigma", type=float, default=0.006)
    parser.add_argument("--joint-limit", type=float, default=0.75)
    parser.add_argument("--duration", type=float, default=8.0)
    parser.add_argument("--ramp", type=float, default=4.0)
    parser.add_argument("--joint-kp", type=float, default=20.0)
    parser.add_argument("--joint-kd", type=float, default=12.0)
    parser.add_argument("--torque-limit", type=float, default=30.0)
    parser.add_argument("--kp-att", type=float, default=0.0)
    parser.add_argument("--kd-att", type=float, default=1.0)
    parser.add_argument("--kcom", type=float, default=1.0)
    parser.add_argument("--roll-sign", type=float, default=-1.0)
    parser.add_argument("--pitch-sign", type=float, default=1.0)
    parser.add_argument("--ratio-weight", type=float, default=45.0)
    parser.add_argument("--final-roll-weight", type=float, default=22.0)
    parser.add_argument("--max-roll-weight", type=float, default=8.0)
    parser.add_argument("--pitch-weight", type=float, default=10.0)
    parser.add_argument("--yaw-weight", type=float, default=4.0)
    parser.add_argument("--qvel-weight", type=float, default=3.0)
    parser.add_argument("--max-qvel-weight", type=float, default=0.5)
    parser.add_argument("--contact-loss-weight", type=float, default=80.0)
    parser.add_argument("--saturation-weight", type=float, default=50.0)
    parser.add_argument("--torque-weight", type=float, default=0.03)
    parser.add_argument("--min-left-contacts", type=float, default=0.0)
    parser.add_argument("--min-right-contacts", type=float, default=0.0)
    parser.add_argument("--left-contact-weight", type=float, default=0.0)
    parser.add_argument("--right-contact-weight", type=float, default=0.0)
    parser.add_argument("--min-left-contact-weight", type=float, default=0.0)
    parser.add_argument("--min-right-contact-weight", type=float, default=0.0)
    parser.add_argument("--min-normal-force", type=float, default=0.0)
    parser.add_argument("--normal-force-weight", type=float, default=0.0)
    parser.add_argument("--max-right-force", type=float, default=1e9)
    parser.add_argument("--right-force-weight", type=float, default=0.0)
    parser.add_argument("--min-left-force", type=float, default=0.0)
    parser.add_argument("--left-force-weight", type=float, default=0.0)
    parser.add_argument("--max-roll-limit", type=float, default=10.0)
    parser.add_argument("--final-roll-limit", type=float, default=10.0)
    parser.add_argument("--max-roll-limit-weight", type=float, default=0.0)
    parser.add_argument("--final-roll-limit-weight", type=float, default=0.0)
    args = parser.parse_args()

    out_dir = args.out_dir.resolve()
    out_dir.mkdir(parents=True, exist_ok=True)
    model = mujoco.MjModel.from_xml_path(str(args.model.resolve()))
    joints = hinge_joint_info(model)
    actuator_names = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, idx) or f"actuator_{idx}" for idx in range(model.nu)]
    start_base_z, start_q = load_targets(model, joints, args.start_pose.resolve())
    seed_base_z, seed_q = load_targets(model, joints, args.seed_end_pose.resolve())
    joint_ids = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, str(joint["name"])) for joint in joints]
    low = np.array([max(float(model.jnt_range[joint_id][0]), -args.joint_limit) for joint_id in joint_ids], dtype=np.float64)
    high = np.array([min(float(model.jnt_range[joint_id][1]), args.joint_limit) for joint_id in joint_ids], dtype=np.float64)

    rng = np.random.default_rng(args.seed)
    mean = seed_q.copy()
    sigma = np.full(seed_q.shape, args.sigma, dtype=np.float64)
    per_iter = max(args.samples // max(args.iterations, 1), args.elite)
    rows: list[dict[str, float]] = []
    best_summary: dict[str, float] | None = None
    best_q = seed_q.copy()

    for iteration in range(args.iterations):
        candidates = rng.normal(mean, sigma, size=(per_iter, seed_q.size))
        candidates = np.clip(candidates, low, high)
        if iteration == 0:
            candidates[0, :] = seed_q
        scored = []
        for local_idx, q in enumerate(candidates):
            summary = simulate_transition(model, joints, actuator_names, start_base_z, start_q, seed_base_z, q, args)
            row = {"iteration": float(iteration), "candidate": float(local_idx), "end_base_z": float(seed_base_z), **summary}
            for joint, value in zip(joints, q):
                row[f"q_{joint['name']}"] = float(value)
            rows.append(row)
            scored.append((summary["score"], q.copy(), summary))
            if best_summary is None or summary["score"] < best_summary["score"]:
                best_summary = summary
                best_q = q.copy()
        scored.sort(key=lambda item: item[0])
        elites = np.array([item[1] for item in scored[: args.elite]], dtype=np.float64)
        mean = elites.mean(axis=0)
        sigma = np.maximum(elites.std(axis=0), args.min_sigma)
        print(json.dumps({"iteration": iteration, "best_score": scored[0][0], "global_best_score": best_summary["score"] if best_summary else None}, ensure_ascii=False))

    if best_summary is None:
        raise RuntimeError("No candidates evaluated.")

    candidates_csv = out_dir / "weight_shift_transition_candidates.csv"
    with candidates_csv.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)

    payload = {
        "model_path": str(args.model.resolve()),
        "start_pose": str(args.start_pose.resolve()),
        "seed_end_pose": str(args.seed_end_pose.resolve()),
        "target_left_ratio": args.target_left_ratio,
        "search": {
            "samples": args.samples,
            "iterations": args.iterations,
            "per_iteration": per_iter,
            "elite": args.elite,
            "seed": args.seed,
            "sigma": args.sigma,
            "min_sigma": args.min_sigma,
            "duration": args.duration,
            "ramp": args.ramp,
            "controller": {
                "joint_kp": args.joint_kp,
                "joint_kd": args.joint_kd,
                "torque_limit": args.torque_limit,
                "kp_att": args.kp_att,
                "kd_att": args.kd_att,
                "kcom": args.kcom,
                "roll_sign": args.roll_sign,
                "pitch_sign": args.pitch_sign,
            },
        },
        "base_z": float(seed_base_z),
        "best": best_summary,
        "joint_targets": {str(joint["name"]): float(value) for joint, value in zip(joints, best_q)},
        "candidates_csv": str(candidates_csv),
    }
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    (out_dir / "weight_shift_transition_search_summary.json").write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(json.dumps(payload, indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
