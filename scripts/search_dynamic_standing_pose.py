from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import mujoco
import numpy as np

from render_pd_standing import hinge_joint_info, load_targets
from sweep_axis_aware_standing import simulate


FOOT_GEOMS = ("foot_L_1_sole_collision", "foot_R_v1_1_sole_collision")


def geom_low_z(model: mujoco.MjModel, data: mujoco.MjData, geom_id: int) -> float:
    xmat = np.asarray(data.geom_xmat[geom_id], dtype=np.float64).reshape(3, 3)
    half = np.asarray(model.geom_size[geom_id], dtype=np.float64)
    z_extent = float(np.abs(xmat[2, :]).dot(half))
    return float(data.geom_xpos[geom_id, 2] - z_extent)


def contact_base_z(model: mujoco.MjModel, data: mujoco.MjData, joints: list[dict[str, int | str]], q: np.ndarray, penetration: float) -> float:
    data.qpos[:] = model.qpos0
    data.qvel[:] = 0.0
    data.qpos[0:3] = np.array([0.0, 0.0, 0.0])
    data.qpos[3:7] = np.array([1.0, 0.0, 0.0, 0.0])
    for value, joint in zip(q, joints):
        data.qpos[int(joint["qposadr"])] = float(value)
    mujoco.mj_forward(model, data)
    lows = []
    for name in FOOT_GEOMS:
        geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name)
        lows.append(geom_low_z(model, data, geom_id))
    return -penetration - min(lows)


def support_margin(model: mujoco.MjModel, data: mujoco.MjData) -> float:
    mass = np.asarray(model.body_mass)
    com = (np.asarray(data.xipos) * mass[:, None]).sum(axis=0) / mass.sum()
    xs: list[float] = []
    ys: list[float] = []
    for name in FOOT_GEOMS:
        geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name)
        xmat = np.asarray(data.geom_xmat[geom_id], dtype=np.float64).reshape(3, 3)
        half = np.asarray(model.geom_size[geom_id], dtype=np.float64)
        extent = np.abs(xmat) @ half
        center = np.asarray(data.geom_xpos[geom_id])
        xs.extend([float(center[0] - extent[0]), float(center[0] + extent[0])])
        ys.extend([float(center[1] - extent[1]), float(center[1] + extent[1])])
    margins = [com[0] - min(xs), max(xs) - com[0], com[1] - min(ys), max(ys) - com[1]]
    return float(min(margins))


def set_kinematic_pose(model: mujoco.MjModel, data: mujoco.MjData, joints: list[dict[str, int | str]], base_z: float, q: np.ndarray) -> None:
    data.qpos[:] = model.qpos0
    data.qvel[:] = 0.0
    data.qpos[0:3] = np.array([0.0, 0.0, base_z])
    data.qpos[3:7] = np.array([1.0, 0.0, 0.0, 0.0])
    for value, joint in zip(q, joints):
        data.qpos[int(joint["qposadr"])] = float(value)
    mujoco.mj_forward(model, data)


def main() -> int:
    parser = argparse.ArgumentParser(description="Search joint targets using dynamic standing rollout score.")
    parser.add_argument("--model", type=Path, required=True)
    parser.add_argument("--seed-pose", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--out-dir", type=Path, default=Path("outputs/analysis/dynamic_standing_pose_search"))
    parser.add_argument("--samples", type=int, default=600)
    parser.add_argument("--iterations", type=int, default=4)
    parser.add_argument("--elite", type=int, default=48)
    parser.add_argument("--seed", type=int, default=23)
    parser.add_argument("--sigma", type=float, default=0.08)
    parser.add_argument("--min-sigma", type=float, default=0.01)
    parser.add_argument("--joint-limit", type=float, default=0.8)
    parser.add_argument("--penetration", type=float, default=0.001)
    parser.add_argument("--duration", type=float, default=2.0)
    parser.add_argument("--log-every", type=int, default=10)
    parser.add_argument("--joint-kp", type=float, default=20.0)
    parser.add_argument("--joint-kd", type=float, default=12.0)
    parser.add_argument("--torque-limit", type=float, default=30.0)
    parser.add_argument("--kp-att", type=float, default=0.0)
    parser.add_argument("--kd-att", type=float, default=1.0)
    parser.add_argument("--kcom", type=float, default=1.0)
    parser.add_argument("--roll-sign", type=float, default=-1.0)
    parser.add_argument("--pitch-sign", type=float, default=-1.0)
    parser.add_argument("--extra-roll-weight", type=float, default=8.0)
    parser.add_argument("--extra-pitch-weight", type=float, default=0.0)
    parser.add_argument("--extra-yaw-weight", type=float, default=2.0)
    parser.add_argument("--base-z-min", type=float, default=-0.01)
    parser.add_argument("--base-z-penalty", type=float, default=0.0)
    args = parser.parse_args()

    out_dir = args.out_dir.resolve()
    out_dir.mkdir(parents=True, exist_ok=True)
    model = mujoco.MjModel.from_xml_path(str(args.model.resolve()))
    data = mujoco.MjData(model)
    joints = hinge_joint_info(model)
    actuator_names = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, idx) or f"actuator_{idx}" for idx in range(model.nu)]
    _, seed_q = load_targets(model, joints, args.seed_pose.resolve())
    joint_ids = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, str(joint["name"])) for joint in joints]
    low = np.array([max(float(model.jnt_range[joint_id][0]), -args.joint_limit) for joint_id in joint_ids], dtype=np.float64)
    high = np.array([min(float(model.jnt_range[joint_id][1]), args.joint_limit) for joint_id in joint_ids], dtype=np.float64)

    rng = np.random.default_rng(args.seed)
    mean = seed_q.copy()
    sigma = np.full(seed_q.shape, args.sigma, dtype=np.float64)
    rows = []
    best: dict[str, float] | None = None
    best_q = seed_q.copy()
    best_base_z = 0.0

    base_cfg = {
        "joint_kp": args.joint_kp,
        "joint_kd": args.joint_kd,
        "torque_limit": args.torque_limit,
        "kp_att": args.kp_att,
        "kd_att": args.kd_att,
        "kcom": args.kcom,
        "roll_sign": args.roll_sign,
        "pitch_sign": args.pitch_sign,
    }
    per_iter = max(args.samples // max(args.iterations, 1), args.elite)
    for iteration in range(args.iterations):
        candidates = rng.normal(mean, sigma, size=(per_iter, seed_q.size))
        candidates = np.clip(candidates, low, high)
        if iteration == 0:
            candidates[0, :] = seed_q
        scored = []
        for local_idx, q in enumerate(candidates):
            base_z = contact_base_z(model, data, joints, q, args.penetration)
            set_kinematic_pose(model, data, joints, base_z, q)
            margin = support_margin(model, data)
            if margin < 0.0:
                summary = {**base_cfg, "score": 1e6 + abs(margin) * 1e5, "final_roll_rad": 999.0, "final_pitch_rad": 999.0, "max_qvel_norm": 999.0, "contact_fraction": 0.0, "saturation_fraction": 1.0}
            else:
                summary, _ = simulate(model, joints, actuator_names, base_z, q, base_cfg, args.duration, args.log_every, False)
                # Dynamic pose search can prioritize longer-horizon drift terms more strongly than the stabilizer sweep.
                summary["score"] += (
                    args.extra_roll_weight * abs(summary["final_roll_rad"])
                    + args.extra_pitch_weight * abs(summary["final_pitch_rad"])
                    + args.extra_yaw_weight * abs(summary["final_yaw_rad"])
                    + args.base_z_penalty * max(0.0, args.base_z_min - summary["final_base_z"])
                )
            row = {
                "iteration": float(iteration),
                "candidate": float(local_idx),
                "base_z": float(base_z),
                "support_margin": float(margin),
                **summary,
            }
            for joint, value in zip(joints, q):
                row[f"q_{joint['name']}"] = float(value)
            rows.append(row)
            scored.append((float(summary["score"]), q.copy(), base_z, row))
            if best is None or float(summary["score"]) < float(best["score"]):
                best = row
                best_q = q.copy()
                best_base_z = base_z
        scored.sort(key=lambda item: item[0])
        elites = np.array([item[1] for item in scored[: args.elite]], dtype=np.float64)
        mean = elites.mean(axis=0)
        sigma = np.maximum(elites.std(axis=0), args.min_sigma)
        print(json.dumps({"iteration": iteration, "best_score": scored[0][0], "global_best_score": best["score"] if best else None}, ensure_ascii=False))

    if best is None:
        raise RuntimeError("No candidates evaluated.")

    csv_path = out_dir / "dynamic_pose_search_candidates.csv"
    with csv_path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)

    payload = {
        "model_path": str(args.model.resolve()),
        "seed_pose": str(args.seed_pose.resolve()),
        "search": {
            "samples": args.samples,
            "iterations": args.iterations,
            "per_iteration": per_iter,
            "elite": args.elite,
            "seed": args.seed,
            "sigma": args.sigma,
            "min_sigma": args.min_sigma,
            "duration": args.duration,
            "penetration": args.penetration,
            "controller": base_cfg,
            "score_weights": {
                "extra_roll_weight": args.extra_roll_weight,
                "extra_pitch_weight": args.extra_pitch_weight,
                "extra_yaw_weight": args.extra_yaw_weight,
                "base_z_min": args.base_z_min,
                "base_z_penalty": args.base_z_penalty,
            },
        },
        "base_z": float(best_base_z),
        "best": {k: v for k, v in best.items() if not k.startswith("q_")},
        "joint_targets": {str(joint["name"]): float(value) for joint, value in zip(joints, best_q)},
        "csv": str(csv_path),
    }
    out_path = args.out.resolve()
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    json_path = out_dir / "dynamic_pose_search_summary.json"
    json_path.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(json.dumps(payload, indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
