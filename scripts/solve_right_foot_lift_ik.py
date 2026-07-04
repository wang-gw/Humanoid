from __future__ import annotations

import argparse
import json
from pathlib import Path

import mujoco
import numpy as np

from render_pd_standing import hinge_joint_info, load_targets
from search_right_foot_lift_pose import right_foot_clearance
from sweep_stabilized_standing import initialize


IK_JOINT_LIMITS = {
    "right_hip_roll": (-0.20, 0.20),
    "right_hip_pitch": (-0.30, 0.20),
    "right_knee_pitch": (-0.35, 0.10),
    "right_ankle_pitch": (-0.20, 0.35),
    "right_ankle_roll": (-0.20, 0.20),
    "left_hip_roll": (-0.08, 0.08),
    "left_ankle_roll": (-0.08, 0.08),
}


def sole_bottom_corners(model: mujoco.MjModel, data: mujoco.MjData) -> np.ndarray:
    corners = []
    for geom_id in range(model.ngeom):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, geom_id) or ""
        if not name.startswith("foot_R_v1_1_sole_pad_"):
            continue
        center = np.asarray(data.geom_xpos[geom_id], dtype=np.float64)
        xmat = np.asarray(data.geom_xmat[geom_id], dtype=np.float64).reshape(3, 3)
        half = np.asarray(model.geom_size[geom_id], dtype=np.float64)
        # Use local bottom face corners only. This directly targets sole clearance.
        for sx in (-1.0, 1.0):
            for sy in (-1.0, 1.0):
                local = np.array([sx * half[0], sy * half[1], -half[2]], dtype=np.float64)
                corners.append(center + xmat @ local)
    return np.asarray(corners, dtype=np.float64)


def set_q(model: mujoco.MjModel, data: mujoco.MjData, joints: list[dict[str, int | str]], base_z: float, q: np.ndarray) -> None:
    initialize(model, data, joints, base_z, q)
    mujoco.mj_forward(model, data)


def corner_z_for_q(model: mujoco.MjModel, joints: list[dict[str, int | str]], base_z: float, q: np.ndarray) -> np.ndarray:
    data = mujoco.MjData(model)
    set_q(model, data, joints, base_z, q)
    return sole_bottom_corners(model, data)[:, 2]


def main() -> int:
    parser = argparse.ArgumentParser(description="Finite-difference IK for right sole clearance.")
    parser.add_argument("--model", type=Path, required=True)
    parser.add_argument("--seed-pose", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--out-dir", type=Path, required=True)
    parser.add_argument("--lift", type=float, default=0.002)
    parser.add_argument("--iterations", type=int, default=18)
    parser.add_argument("--step", type=float, default=1e-4)
    parser.add_argument("--gain", type=float, default=0.65)
    parser.add_argument("--damping", type=float, default=1e-4)
    parser.add_argument("--max-dq", type=float, default=0.035)
    args = parser.parse_args()

    out_dir = args.out_dir.resolve()
    out_dir.mkdir(parents=True, exist_ok=True)
    model = mujoco.MjModel.from_xml_path(str(args.model.resolve()))
    joints = hinge_joint_info(model)
    joint_names = [str(joint["name"]) for joint in joints]
    base_z, q = load_targets(model, joints, args.seed_pose.resolve())
    seed_q = q.copy()

    ik_joint_names = [name for name in IK_JOINT_LIMITS if name in joint_names]
    ik_indices = [joint_names.index(name) for name in ik_joint_names]
    lower = np.array([IK_JOINT_LIMITS[name][0] for name in ik_joint_names], dtype=np.float64)
    upper = np.array([IK_JOINT_LIMITS[name][1] for name in ik_joint_names], dtype=np.float64)

    initial_z = corner_z_for_q(model, joints, base_z, q)
    target_z = initial_z + args.lift
    trace = []
    for iteration in range(args.iterations):
        current_z = corner_z_for_q(model, joints, base_z, q)
        residual = target_z - current_z
        jac = np.zeros((len(current_z), len(ik_indices)), dtype=np.float64)
        for col, q_idx in enumerate(ik_indices):
            q_plus = q.copy()
            q_plus[q_idx] += args.step
            z_plus = corner_z_for_q(model, joints, base_z, q_plus)
            jac[:, col] = (z_plus - current_z) / args.step
        lhs = jac.T @ jac + args.damping * np.eye(len(ik_indices))
        rhs = jac.T @ residual
        try:
            dq_sel = np.linalg.solve(lhs, rhs)
        except np.linalg.LinAlgError:
            dq_sel = np.linalg.lstsq(lhs, rhs, rcond=None)[0]
        dq_sel = np.clip(args.gain * dq_sel, -args.max_dq, args.max_dq)
        for local_idx, q_idx in enumerate(ik_indices):
            q[q_idx] += dq_sel[local_idx]
            q[q_idx] = seed_q[q_idx] + float(np.clip(q[q_idx] - seed_q[q_idx], lower[local_idx], upper[local_idx]))

        data = mujoco.MjData(model)
        set_q(model, data, joints, base_z, q)
        trace.append(
            {
                "iteration": iteration,
                "min_z": float(np.min(current_z)),
                "max_z": float(np.max(current_z)),
                "mean_abs_residual": float(np.mean(np.abs(residual))),
                "clearance": right_foot_clearance(model, data),
                "dq_norm": float(np.linalg.norm(dq_sel)),
            }
        )

    data = mujoco.MjData(model)
    set_q(model, data, joints, base_z, q)
    final_z = sole_bottom_corners(model, data)[:, 2]
    targets = json.loads(args.seed_pose.read_text(encoding="utf-8")).get("joint_targets", {})
    targets = dict(targets)
    for idx, name in enumerate(joint_names):
        targets[name] = float(q[idx])
    payload = {
        "base_z": float(base_z),
        "joint_targets": targets,
        "source": {
            "seed_pose": str(args.seed_pose.resolve()),
            "model": str(args.model.resolve()),
            "lift": args.lift,
            "ik_joint_names": ik_joint_names,
            "initial_min_z": float(np.min(initial_z)),
            "final_min_z": float(np.min(final_z)),
            "initial_corner_z": [float(v) for v in initial_z],
            "final_corner_z": [float(v) for v in final_z],
            "trace": trace,
        },
    }
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    summary = {
        "out_pose": str(args.out.resolve()),
        "initial_min_z": float(np.min(initial_z)),
        "final_min_z": float(np.min(final_z)),
        "requested_lift": args.lift,
        "achieved_lift": float(np.min(final_z) - np.min(initial_z)),
        "final_clearance": right_foot_clearance(model, data),
        "ik_joint_names": ik_joint_names,
        "trace": trace,
    }
    (out_dir / "right_foot_lift_ik_summary.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(json.dumps(summary, indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
