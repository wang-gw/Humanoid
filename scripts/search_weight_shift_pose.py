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
from sweep_stabilized_standing import support_center, total_com


def sole_geom_ids(model: mujoco.MjModel) -> list[int]:
    ids: list[int] = []
    for geom_id in range(model.ngeom):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, geom_id) or ""
        if name in ("foot_L_1_sole_collision", "foot_R_v1_1_sole_collision"):
            ids.append(geom_id)
        elif name.startswith("foot_L_1_sole_pad_") or name.startswith("foot_R_v1_1_sole_pad_"):
            ids.append(geom_id)
    if not ids:
        raise ValueError("No sole contact geoms found.")
    return ids


def geom_low_z(model: mujoco.MjModel, data: mujoco.MjData, geom_id: int) -> float:
    xmat = np.asarray(data.geom_xmat[geom_id], dtype=np.float64).reshape(3, 3)
    half = np.asarray(model.geom_size[geom_id], dtype=np.float64)
    z_extent = float(np.abs(xmat[2, :]).dot(half))
    return float(data.geom_xpos[geom_id, 2] - z_extent)


def set_pose(model: mujoco.MjModel, data: mujoco.MjData, joints: list[dict[str, int | str]], base_z: float, q: np.ndarray) -> None:
    data.qpos[:] = model.qpos0
    data.qvel[:] = 0.0
    data.qpos[0:3] = np.array([0.0, 0.0, base_z])
    data.qpos[3:7] = np.array([1.0, 0.0, 0.0, 0.0])
    for value, joint in zip(q, joints):
        data.qpos[int(joint["qposadr"])] = float(value)
    mujoco.mj_forward(model, data)


def contact_base_z(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    joints: list[dict[str, int | str]],
    q: np.ndarray,
    penetration: float,
    sole_ids: list[int],
) -> float:
    set_pose(model, data, joints, 0.0, q)
    lowest = min(geom_low_z(model, data, geom_id) for geom_id in sole_ids)
    return -penetration - lowest


def simulate_candidate(
    model: mujoco.MjModel,
    joints: list[dict[str, int | str]],
    actuator_names: list[str],
    base_z: float,
    target_q: np.ndarray,
    args: argparse.Namespace,
) -> tuple[dict[str, float], list[dict[str, float]]]:
    data = mujoco.MjData(model)
    set_pose(model, data, joints, base_z, target_q)
    geom_to_side = foot_geom_ids(model)
    steps = int(args.duration / model.opt.timestep)
    log_every = max(1, int(args.log_every))
    max_abs_tau = 0.0
    max_qvel = 0.0
    max_contact = 0.0
    sat_samples = 0
    contact_samples = 0
    rows: list[dict[str, float]] = []

    for step in range(steps + 1):
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
        sat_samples += int(np.any(np.abs(tau) >= args.torque_limit * 0.999))
        contact_samples += int(data.ncon > 0)

        if step % log_every == 0 or step == steps:
            rows.append(
                {
                    "time": float(data.time),
                    "base_z": float(data.qpos[2]),
                    "roll": float(roll),
                    "pitch": float(pitch),
                    "yaw": float(yaw),
                    "qvel_norm": qvel,
                    "com_err_x": float(com_err[0]),
                    "com_err_y": float(com_err[1]),
                    **{key: float(value) for key, value in stats.items()},
                    "max_abs_tau_so_far": float(max_abs_tau),
                }
            )
        if step < steps:
            mujoco.mj_step(model, data)

    final = rows[-1]
    contact_fraction = contact_samples / max(steps + 1, 1)
    saturation_fraction = sat_samples / max(steps + 1, 1)
    ratio_error = abs(final["left_force_ratio"] - args.target_left_ratio)
    score = (
        args.ratio_weight * ratio_error
        + args.roll_weight * abs(final["roll"])
        + args.pitch_weight * abs(final["pitch"])
        + args.yaw_weight * abs(final["yaw"])
        + args.qvel_weight * final["qvel_norm"]
        + args.max_qvel_weight * max_qvel
        + args.saturation_weight * saturation_fraction
        + args.contact_loss_weight * (1.0 - contact_fraction)
        + args.torque_weight * max_abs_tau
    )
    summary = {
        "score": float(score),
        "ratio_error": float(ratio_error),
        "target_left_ratio": float(args.target_left_ratio),
        "final_left_force_ratio": float(final["left_force_ratio"]),
        "final_right_force_ratio": float(final["right_force_ratio"]),
        "final_roll_rad": float(final["roll"]),
        "final_pitch_rad": float(final["pitch"]),
        "final_yaw_rad": float(final["yaw"]),
        "final_base_z": float(final["base_z"]),
        "final_qvel_norm": float(final["qvel_norm"]),
        "final_contact_force": float(final["normal_force"]),
        "final_left_contacts": float(final["left_contacts"]),
        "final_right_contacts": float(final["right_contacts"]),
        "max_qvel_norm": float(max_qvel),
        "max_contact_force": float(max_contact),
        "max_abs_tau": float(max_abs_tau),
        "contact_fraction": float(contact_fraction),
        "saturation_fraction": float(saturation_fraction),
    }
    return summary, rows


def main() -> int:
    parser = argparse.ArgumentParser(description="Search a stable static pose with a target left/right foot force ratio.")
    parser.add_argument("--model", type=Path, required=True)
    parser.add_argument("--seed-pose", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--out-dir", type=Path, required=True)
    parser.add_argument("--target-left-ratio", type=float, default=0.75)
    parser.add_argument("--samples", type=int, default=360)
    parser.add_argument("--iterations", type=int, default=3)
    parser.add_argument("--elite", type=int, default=36)
    parser.add_argument("--seed", type=int, default=83)
    parser.add_argument("--sigma", type=float, default=0.06)
    parser.add_argument("--min-sigma", type=float, default=0.012)
    parser.add_argument("--joint-limit", type=float, default=0.75)
    parser.add_argument("--penetration", type=float, default=0.001)
    parser.add_argument("--duration", type=float, default=3.0)
    parser.add_argument("--log-every", type=int, default=20)
    parser.add_argument("--joint-kp", type=float, default=20.0)
    parser.add_argument("--joint-kd", type=float, default=12.0)
    parser.add_argument("--torque-limit", type=float, default=30.0)
    parser.add_argument("--kp-att", type=float, default=0.0)
    parser.add_argument("--kd-att", type=float, default=1.0)
    parser.add_argument("--kcom", type=float, default=1.0)
    parser.add_argument("--roll-sign", type=float, default=-1.0)
    parser.add_argument("--pitch-sign", type=float, default=1.0)
    parser.add_argument("--ratio-weight", type=float, default=30.0)
    parser.add_argument("--roll-weight", type=float, default=12.0)
    parser.add_argument("--pitch-weight", type=float, default=8.0)
    parser.add_argument("--yaw-weight", type=float, default=2.0)
    parser.add_argument("--qvel-weight", type=float, default=1.0)
    parser.add_argument("--max-qvel-weight", type=float, default=0.2)
    parser.add_argument("--saturation-weight", type=float, default=40.0)
    parser.add_argument("--contact-loss-weight", type=float, default=40.0)
    parser.add_argument("--torque-weight", type=float, default=0.02)
    args = parser.parse_args()

    out_dir = args.out_dir.resolve()
    out_dir.mkdir(parents=True, exist_ok=True)
    model = mujoco.MjModel.from_xml_path(str(args.model.resolve()))
    data = mujoco.MjData(model)
    joints = hinge_joint_info(model)
    actuator_names = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, idx) or f"actuator_{idx}" for idx in range(model.nu)]
    _, seed_q = load_targets(model, joints, args.seed_pose.resolve())
    sole_ids = sole_geom_ids(model)
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
    best_base_z = 0.0
    best_timeline: list[dict[str, float]] = []

    for iteration in range(args.iterations):
        candidates = rng.normal(mean, sigma, size=(per_iter, seed_q.size))
        candidates = np.clip(candidates, low, high)
        if iteration == 0:
            candidates[0, :] = seed_q
        scored = []
        for local_idx, q in enumerate(candidates):
            base_z = contact_base_z(model, data, joints, q, args.penetration, sole_ids)
            summary, timeline = simulate_candidate(model, joints, actuator_names, base_z, q, args)
            row = {"iteration": float(iteration), "candidate": float(local_idx), "base_z": float(base_z), **summary}
            for joint, value in zip(joints, q):
                row[f"q_{joint['name']}"] = float(value)
            rows.append(row)
            scored.append((summary["score"], q.copy(), base_z, summary, timeline))
            if best_summary is None or summary["score"] < best_summary["score"]:
                best_summary = summary
                best_q = q.copy()
                best_base_z = base_z
                best_timeline = timeline
        scored.sort(key=lambda item: item[0])
        elites = np.array([item[1] for item in scored[: args.elite]], dtype=np.float64)
        mean = elites.mean(axis=0)
        sigma = np.maximum(elites.std(axis=0), args.min_sigma)
        print(json.dumps({"iteration": iteration, "best_score": scored[0][0], "global_best_score": best_summary["score"] if best_summary else None}, ensure_ascii=False))

    if best_summary is None:
        raise RuntimeError("No candidates evaluated.")

    candidates_csv = out_dir / "weight_shift_pose_candidates.csv"
    with candidates_csv.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)

    timeline_csv = out_dir / "best_weight_shift_pose_timeline.csv"
    with timeline_csv.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(best_timeline[0].keys()))
        writer.writeheader()
        writer.writerows(best_timeline)

    payload = {
        "model_path": str(args.model.resolve()),
        "seed_pose": str(args.seed_pose.resolve()),
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
        "base_z": float(best_base_z),
        "best": best_summary,
        "joint_targets": {str(joint["name"]): float(value) for joint, value in zip(joints, best_q)},
        "candidates_csv": str(candidates_csv),
        "best_timeline_csv": str(timeline_csv),
    }
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    (out_dir / "weight_shift_pose_search_summary.json").write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(json.dumps(payload, indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
