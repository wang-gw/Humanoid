#!/usr/bin/env python3
"""Quasi-static standing-pose search for the urdf_f_v2 model.

Adapted from search_quasistatic_standing_pose.py. The v2 footprint-contact model
senses the ground through a 2x2 grid of `<foot>_sole_pad_*` box geoms per foot
(no single `_sole_collision` geom), so this variant aggregates all pad geoms of a
foot into one support region and seats the model on the floor accordingly.

CEM optimizes the 10 joint angles so the whole-body CoM projects inside the
two-foot support polygon with both soles flat and level — a statically balanced
pose to seed RL training.

Usage:
  python3 scripts/search_quasistatic_standing_pose_v2.py \
      --model envs/robots/urdf_f_v2/URDF_F_v2_footprint_contact.xml \
      --left-foot foot_L_1 --right-foot foot_R_1 \
      --out configs/urdf_f_v2/quasistatic_standing_pose.json
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import mujoco
import numpy as np


def hinge_joints(model: mujoco.MjModel) -> list[dict[str, object]]:
    joints = []
    for joint_id in range(model.njnt):
        if model.jnt_type[joint_id] != mujoco.mjtJoint.mjJNT_HINGE:
            continue
        joints.append(
            {
                "name": mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, joint_id) or f"joint_{joint_id}",
                "qposadr": int(model.jnt_qposadr[joint_id]),
                "range": np.asarray(model.jnt_range[joint_id], dtype=np.float64),
            }
        )
    return joints


def pad_geom_ids(model: mujoco.MjModel, foot_body: str) -> list[int]:
    prefix = f"{foot_body}_sole_pad_"
    ids = []
    for gid in range(model.ngeom):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, gid) or ""
        if name.startswith(prefix):
            ids.append(gid)
    if not ids:
        raise ValueError(f"no sole pad geoms with prefix {prefix!r}")
    return ids


def total_com(model: mujoco.MjModel, data: mujoco.MjData) -> np.ndarray:
    mass = np.asarray(model.body_mass)
    return (np.asarray(data.xipos) * mass[:, None]).sum(axis=0) / mass.sum()


def foot_region(model, data, pad_ids):
    """Aggregate a foot's pads: return (x_lo,x_hi,y_lo,y_hi, low_z, center_xy)."""
    xs, ys, lows = [], [], []
    for gid in pad_ids:
        center = np.asarray(data.geom_xpos[gid])
        xmat = np.asarray(data.geom_xmat[gid]).reshape(3, 3)
        ext = np.abs(xmat) @ np.asarray(model.geom_size[gid])
        xs += [float(center[0] - ext[0]), float(center[0] + ext[0])]
        ys += [float(center[1] - ext[1]), float(center[1] + ext[1])]
        lows.append(float(center[2] - ext[2]))
    cx = 0.5 * (min(xs) + max(xs))
    cy = 0.5 * (min(ys) + max(ys))
    return min(xs), max(xs), min(ys), max(ys), min(lows), np.array([cx, cy])


def foot_normal_penalty(model, data, foot_body: str) -> float:
    bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, foot_body)
    zaxis = np.asarray(data.xmat[bid]).reshape(3, 3)[:, 2]
    return float((1.0 - abs(zaxis[2])) ** 2)


def set_pose(model, data, joints, q, clearance, feet_pads) -> float:
    data.qpos[:] = model.qpos0
    data.qvel[:] = 0.0
    data.qpos[0:3] = np.array([0.0, 0.0, 0.0])
    data.qpos[3:7] = np.array([1.0, 0.0, 0.0, 0.0])
    for value, joint in zip(q, joints):
        data.qpos[int(joint["qposadr"])] = float(value)
    mujoco.mj_forward(model, data)

    lows = [foot_region(model, data, pads)[4] for pads in feet_pads]
    base_z = clearance - min(lows)
    data.qpos[2] = base_z
    mujoco.mj_forward(model, data)
    return float(base_z)


def evaluate(model, data, joints, q, clearance, feet_pads, feet_bodies) -> dict[str, object]:
    base_z = set_pose(model, data, joints, q, clearance, feet_pads)
    com = total_com(model, data)

    regions = [foot_region(model, data, pads) for pads in feet_pads]
    xs = [r[0] for r in regions] + [r[1] for r in regions]
    ys = [r[2] for r in regions] + [r[3] for r in regions]
    lows = [r[4] for r in regions]
    centers = [r[5] for r in regions]
    normal_penalty = sum(foot_normal_penalty(model, data, b) for b in feet_bodies)

    aabb = {"x_min": min(xs), "x_max": max(xs), "y_min": min(ys), "y_max": max(ys)}
    margins = np.array(
        [com[0] - aabb["x_min"], aabb["x_max"] - com[0], com[1] - aabb["y_min"], aabb["y_max"] - com[1]],
        dtype=np.float64,
    )
    support_center = np.array([(aabb["x_min"] + aabb["x_max"]) * 0.5, (aabb["y_min"] + aabb["y_max"]) * 0.5])
    support_half = np.array([(aabb["x_max"] - aabb["x_min"]) * 0.5, (aabb["y_max"] - aabb["y_min"]) * 0.5])
    normalized_com_error = (com[:2] - support_center) / np.maximum(support_half, 1e-6)

    foot_height_diff = abs(lows[0] - lows[1])
    foot_y_symmetry = abs(float(centers[0][1] - centers[1][1]))
    outside_penalty = float(np.square(np.minimum(margins, 0.0)).sum())
    center_penalty = float(normalized_com_error.dot(normalized_com_error))
    joint_penalty = float(q.dot(q))
    base_penalty = 0.0 if 0.0 <= base_z <= 0.8 else float((base_z - np.clip(base_z, 0.0, 0.8)) ** 2)

    cost = (
        10000.0 * outside_penalty
        + 25.0 * center_penalty
        + 5000.0 * foot_height_diff * foot_height_diff
        + 100.0 * normal_penalty
        + 0.25 * joint_penalty
        + 200.0 * foot_y_symmetry * foot_y_symmetry
        + 1000.0 * base_penalty
    )
    return {
        "cost": float(cost),
        "base_z": base_z,
        "com_world": [float(v) for v in com],
        "support_aabb": aabb,
        "support_margins": [float(v) for v in margins],
        "foot_low_z": [float(v) for v in lows],
        "foot_height_diff": float(foot_height_diff),
        "normal_penalty": float(normal_penalty),
        "center_penalty": float(center_penalty),
        "joint_norm": float(np.linalg.norm(q)),
    }


def sample_candidates(rng, mean, sigma, count, low, high):
    q = rng.normal(mean, sigma, size=(count, mean.size))
    return np.clip(q, low, high)


def main() -> int:
    parser = argparse.ArgumentParser(description="Quasi-static standing pose search for urdf_f_v2.")
    parser.add_argument("--model", type=Path, default=Path("envs/robots/urdf_f_v2/URDF_F_v2_footprint_contact.xml"))
    parser.add_argument("--left-foot", default="foot_L_1")
    parser.add_argument("--right-foot", default="foot_R_1")
    parser.add_argument("--samples", type=int, default=40000)
    parser.add_argument("--elite", type=int, default=256)
    parser.add_argument("--iterations", type=int, default=8)
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--limit", type=float, default=0.8)
    parser.add_argument("--clearance", type=float, default=0.005)
    parser.add_argument("--out", type=Path, default=Path("configs/urdf_f_v2/quasistatic_standing_pose.json"))
    args = parser.parse_args()

    model_path = args.model.resolve()
    model = mujoco.MjModel.from_xml_path(str(model_path))
    data = mujoco.MjData(model)
    joints = hinge_joints(model)
    feet_bodies = [args.left_foot, args.right_foot]
    feet_pads = [pad_geom_ids(model, b) for b in feet_bodies]

    low = np.array([max(float(j["range"][0]), -args.limit) for j in joints], dtype=np.float64)
    high = np.array([min(float(j["range"][1]), args.limit) for j in joints], dtype=np.float64)

    rng = np.random.default_rng(args.seed)
    mean = np.zeros(len(joints), dtype=np.float64)
    sigma = np.full(len(joints), args.limit * 0.5, dtype=np.float64)
    best_result = None
    best_q = None
    history = []
    per_iter = max(args.samples // max(args.iterations, 1), args.elite)

    for iteration in range(args.iterations):
        candidates = sample_candidates(rng, mean, sigma, per_iter, low, high)
        if iteration == 0:
            candidates[0, :] = 0.0
        scored = []
        for q in candidates:
            result = evaluate(model, data, joints, q, args.clearance, feet_pads, feet_bodies)
            scored.append((float(result["cost"]), q.copy(), result))
            if best_result is None or float(result["cost"]) < float(best_result["cost"]):
                best_result = result
                best_q = q.copy()
        scored.sort(key=lambda item: item[0])
        elites = np.array([item[1] for item in scored[: args.elite]], dtype=np.float64)
        mean = elites.mean(axis=0)
        sigma = np.maximum(elites.std(axis=0), 0.03)
        history.append({"iteration": iteration, "best_cost": scored[0][0], "global_best_cost": float(best_result["cost"])})
        print(f"iter {iteration}: best_cost={scored[0][0]:.4f} global_best={float(best_result['cost']):.4f}")

    if best_result is None or best_q is None:
        raise RuntimeError("No candidate evaluated.")

    joint_targets = {str(j["name"]): float(v) for j, v in zip(joints, best_q)}
    payload = {
        "model_path": str(model_path),
        "_status": "quasi-static balance candidate — verify with a PD/RL rollout before trusting.",
        "search": {
            "samples": args.samples,
            "per_iteration": per_iter,
            "elite": args.elite,
            "iterations": args.iterations,
            "seed": args.seed,
            "joint_abs_limit": args.limit,
            "clearance": args.clearance,
            "left_foot": args.left_foot,
            "right_foot": args.right_foot,
            "history": history,
        },
        "base_z": best_result["base_z"],
        "cost": best_result["cost"],
        "joint_targets": joint_targets,
        "com_world": best_result["com_world"],
        "support_aabb": best_result["support_aabb"],
        "support_margins": best_result["support_margins"],
        "foot_low_z": best_result["foot_low_z"],
        "foot_height_diff": best_result["foot_height_diff"],
        "normal_penalty": best_result["normal_penalty"],
        "center_penalty": best_result["center_penalty"],
        "joint_norm": best_result["joint_norm"],
    }
    out = args.out.resolve()
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(f"\nWrote {out}")
    print(json.dumps({"cost": best_result["cost"], "base_z": best_result["base_z"],
                      "support_margins": best_result["support_margins"],
                      "joint_norm": best_result["joint_norm"]}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
