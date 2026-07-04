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


RIGHT_LIFT_JOINTS = {
    "right_hip_roll": (-0.18, 0.18),
    "right_hip_pitch": (-0.35, 0.35),
    "right_knee_pitch": (-0.45, 0.45),
    "right_ankle_pitch": (-0.45, 0.45),
    "right_ankle_roll": (-0.18, 0.18),
}


def right_foot_clearance(model: mujoco.MjModel, data: mujoco.MjData) -> float:
    lowest = np.inf
    for geom_id in range(model.ngeom):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, geom_id) or ""
        if not name.startswith("foot_R_v1_1_sole_pad_"):
            continue
        center = np.asarray(data.geom_xpos[geom_id], dtype=np.float64)
        xmat = np.asarray(data.geom_xmat[geom_id], dtype=np.float64).reshape(3, 3)
        half = np.asarray(model.geom_size[geom_id], dtype=np.float64)
        for sx in (-1.0, 1.0):
            for sy in (-1.0, 1.0):
                for sz in (-1.0, 1.0):
                    local = np.array([sx * half[0], sy * half[1], sz * half[2]], dtype=np.float64)
                    lowest = min(lowest, float((center + xmat @ local)[2]))
    return float(lowest)


def simulate(
    model: mujoco.MjModel,
    joints: list[dict[str, int | str]],
    actuator_names: list[str],
    base_z: float,
    start_q: np.ndarray,
    target_q: np.ndarray,
    args: argparse.Namespace,
) -> dict[str, float]:
    data = mujoco.MjData(model)
    initialize(model, data, joints, base_z, start_q)
    geom_to_side = foot_geom_ids(model)
    steps = int(args.duration / model.opt.timestep)
    max_abs_tau = 0.0
    max_qvel = 0.0
    max_abs_roll = 0.0
    max_contact = 0.0
    sat_samples = 0
    min_clearance = np.inf
    max_clearance = -np.inf

    for step in range(steps + 1):
        alpha = min(max(float(data.time) / max(args.ramp, 1e-9), 0.0), 1.0)
        alpha = alpha * alpha * (3.0 - 2.0 * alpha)
        q_target = (1.0 - alpha) * start_q + alpha * target_q
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
        tau = args.joint_kp * (q_target - q) - args.joint_kd * qd
        for idx, name in enumerate(actuator_names):
            if name in ROLL_ROLE_ACTUATORS:
                tau[idx] += args.roll_sign * roll_term
            if name in PITCH_ROLE_ACTUATORS:
                tau[idx] += args.pitch_sign * pitch_term
            tau[idx] += force_term
        tau = np.clip(tau, -args.torque_limit, args.torque_limit)
        data.ctrl[:] = tau

        clearance = right_foot_clearance(model, data)
        min_clearance = min(min_clearance, clearance)
        max_clearance = max(max_clearance, clearance)
        qvel = float(np.linalg.norm(data.qvel))
        max_qvel = max(max_qvel, qvel)
        max_abs_roll = max(max_abs_roll, abs(float(roll)))
        max_contact = max(max_contact, stats["normal_force"])
        max_abs_tau = max(max_abs_tau, float(np.max(np.abs(data.actuator_force))) if model.nu else 0.0)
        sat_samples += int(np.any(np.abs(tau) >= args.torque_limit * 0.999))
        if step < steps:
            mujoco.mj_step(model, data)

    final_stats = contact_stats(model, data, geom_to_side)
    final_roll, final_pitch, final_yaw = quat_to_roll_pitch_yaw(data.qpos[3:7])
    final_clearance = right_foot_clearance(model, data)
    return {
        "final_clearance_m": final_clearance,
        "max_clearance_m": float(max_clearance),
        "min_clearance_m": float(min_clearance),
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
    parser = argparse.ArgumentParser(description="Search a small right-foot lift pose from a loaded left-support pose.")
    parser.add_argument("--model", type=Path, required=True)
    parser.add_argument("--seed-pose", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--out-dir", type=Path, required=True)
    parser.add_argument("--samples", type=int, default=240)
    parser.add_argument("--seed", type=int, default=501)
    parser.add_argument("--duration", type=float, default=6.0)
    parser.add_argument("--ramp", type=float, default=3.0)
    parser.add_argument("--target-clearance", type=float, default=0.005)
    parser.add_argument("--target-left-ratio", type=float, default=0.78)
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

    candidates = []
    zero = {name: 0.0 for name in RIGHT_LIFT_JOINTS}
    candidate_offsets = [zero]
    for _ in range(args.samples):
        candidate_offsets.append({name: float(rng.uniform(lo, hi)) for name, (lo, hi) in RIGHT_LIFT_JOINTS.items()})

    best = None
    for offsets in candidate_offsets:
        target_q = seed_q.copy()
        for name, value in offsets.items():
            if name in joint_names:
                target_q[joint_names.index(name)] += value
        metrics = simulate(model, joints, actuator_names, base_z, seed_q, target_q, args)
        clearance_error = max(0.0, args.target_clearance - metrics["final_clearance_m"])
        score = (
            8000.0 * clearance_error
            + 300.0 * max(0.0, metrics["final_right_force"] - 20.0) / 90.0
            + 300.0 * max(0.0, 0.70 - metrics["final_left_ratio"])
            + 80.0 * max(0.0, metrics["max_abs_roll"] - 0.18)
            + 20.0 * max(0.0, metrics["final_right_contacts"] - 1.0)
            + 60.0 * metrics["saturation_fraction"]
            + 0.03 * metrics["max_abs_tau"]
        )
        row = {"score": float(score), **offsets, **metrics}
        candidates.append(row)
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

    csv_path = out_dir / "right_foot_lift_candidates.csv"
    with csv_path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(candidates[0].keys()))
        writer.writeheader()
        writer.writerows(sorted(candidates, key=lambda row: row["score"]))
    summary = {"best": best, "out_pose": str(args.out.resolve()), "csv": str(csv_path), "samples": len(candidates)}
    (out_dir / "right_foot_lift_search_summary.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(json.dumps(summary, indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
