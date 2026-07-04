from __future__ import annotations

import argparse
import json
from pathlib import Path

import mujoco
import numpy as np


FOOT_COLLISION_NAMES = ("foot_L_1_sole_collision", "foot_R_v1_1_sole_collision")


def hinge_joints(model: mujoco.MjModel) -> list[dict[str, object]]:
    joints: list[dict[str, object]] = []
    for joint_id in range(model.njnt):
        if model.jnt_type[joint_id] != mujoco.mjtJoint.mjJNT_HINGE:
            continue
        joints.append(
            {
                "id": joint_id,
                "name": mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, joint_id) or f"joint_{joint_id}",
                "qposadr": int(model.jnt_qposadr[joint_id]),
                "range": [float(v) for v in model.jnt_range[joint_id]],
            }
        )
    return joints


def evaluate_pose(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    joints: list[dict[str, object]],
    foot_geoms: list[int],
    q: np.ndarray,
    clearance: float,
) -> dict[str, object]:
    data.qpos[:] = model.qpos0
    data.qvel[:] = 0.0
    data.qpos[3:7] = np.array([1.0, 0.0, 0.0, 0.0])
    for value, joint in zip(q, joints):
        data.qpos[int(joint["qposadr"])] = float(value)
    data.qpos[2] = 0.0
    mujoco.mj_forward(model, data)
    foot_lows = [float(data.geom_xpos[geom_id, 2] - model.geom_size[geom_id, 2]) for geom_id in foot_geoms]
    base_z = clearance - min(foot_lows)
    data.qpos[2] = base_z
    mujoco.mj_forward(model, data)

    mass = np.asarray(model.body_mass)
    com = (np.asarray(data.xipos) * mass[:, None]).sum(axis=0) / mass.sum()
    xs: list[float] = []
    ys: list[float] = []
    for geom_id in foot_geoms:
        center = np.asarray(data.geom_xpos[geom_id])
        size = np.asarray(model.geom_size[geom_id])
        xs.extend([float(center[0] - size[0]), float(center[0] + size[0])])
        ys.extend([float(center[1] - size[1]), float(center[1] + size[1])])
    aabb = {"x_min": min(xs), "x_max": max(xs), "y_min": min(ys), "y_max": max(ys)}
    margins = [
        float(com[0] - aabb["x_min"]),
        float(aabb["x_max"] - com[0]),
        float(com[1] - aabb["y_min"]),
        float(aabb["y_max"] - com[1]),
    ]
    return {
        "score": min(margins),
        "base_z": float(base_z),
        "com_world": [float(v) for v in com],
        "support_aabb": aabb,
        "support_margins": margins,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description="Search for a geometry-feasible standing pose candidate.")
    parser.add_argument("--model", type=Path, default=Path("envs/robots/urdf_f/URDF_F_contact.xml"))
    parser.add_argument("--samples", type=int, default=30000)
    parser.add_argument("--seed", type=int, default=1)
    parser.add_argument("--limit", type=float, default=0.8)
    parser.add_argument("--min-base-z", type=float, default=0.03)
    parser.add_argument("--max-base-z", type=float, default=0.5)
    parser.add_argument("--clearance", type=float, default=0.005)
    parser.add_argument("--out", type=Path, default=Path("configs/standing_pose_candidate.json"))
    parser.add_argument("--doc-dir", type=Path, default=Path("docs/hardware_validation"))
    args = parser.parse_args()

    model_path = args.model.resolve()
    model = mujoco.MjModel.from_xml_path(str(model_path))
    data = mujoco.MjData(model)
    joints = hinge_joints(model)
    foot_geoms = [
        mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name) for name in FOOT_COLLISION_NAMES
    ]
    if any(geom_id < 0 for geom_id in foot_geoms):
        raise ValueError("Simplified foot collision geoms are required.")

    rng = np.random.default_rng(args.seed)
    best: dict[str, object] | None = None
    best_q: np.ndarray | None = None
    accepted = 0
    for _ in range(max(args.samples, 1)):
        q = np.array(
            [
                rng.uniform(
                    max(float(joint["range"][0]), -args.limit),
                    min(float(joint["range"][1]), args.limit),
                )
                for joint in joints
            ],
            dtype=np.float64,
        )
        result = evaluate_pose(model, data, joints, foot_geoms, q, args.clearance)
        if not (args.min_base_z <= float(result["base_z"]) <= args.max_base_z):
            continue
        accepted += 1
        if best is None or float(result["score"]) > float(best["score"]):
            best = result
            best_q = q.copy()

    if best is None or best_q is None:
        raise RuntimeError("No standing pose candidate found with the requested constraints.")

    joint_targets = {str(joint["name"]): float(value) for joint, value in zip(joints, best_q)}
    payload = {
        "model_path": str(model_path),
        "search": {
            "samples": args.samples,
            "accepted": accepted,
            "seed": args.seed,
            "joint_abs_limit": args.limit,
            "min_base_z": args.min_base_z,
            "max_base_z": args.max_base_z,
        },
        "base_z": best["base_z"],
        "score": best["score"],
        "joint_targets": joint_targets,
        "com_world": best["com_world"],
        "support_aabb": best["support_aabb"],
        "support_margins": best["support_margins"],
    }

    out = args.out.resolve()
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")

    doc_dir = args.doc_dir.resolve()
    doc_dir.mkdir(parents=True, exist_ok=True)
    md_path = doc_dir / "09_standing_pose_search.md"
    md_path.write_text(
        "\n".join(
            [
                "# Standing Pose Search",
                "",
                "## Purpose",
                "",
                "Neutral pose has its COM outside the simplified foot support area. This search finds a first geometry-feasible joint pose candidate before running another dynamic PD standing probe.",
                "",
                "## Command",
                "",
                "```bash",
                "python3 scripts/search_standing_pose.py",
                "```",
                "",
                "## Result",
                "",
                f"- Candidate config: `{out}`",
                f"- Samples: `{args.samples}`",
                f"- Accepted by base-z constraint: `{accepted}`",
                f"- Score, min COM support margin: `{float(best['score']):.6f}`",
                f"- Base z: `{float(best['base_z']):.6f}`",
                f"- COM world: `{best['com_world']}`",
                f"- Support AABB: `{best['support_aabb']}`",
                "",
                "## Joint Targets",
                "",
                *[f"- `{name}`: `{value:.6f}` rad" for name, value in joint_targets.items()],
                "",
                "## Interpretation",
                "",
                "This is a geometry candidate, not a validated standing controller. It must be tested with forward dynamics and torque logging.",
                "",
            ]
        ),
        encoding="utf-8",
    )
    print(f"Wrote {out}")
    print(f"Wrote {md_path}")
    print(json.dumps({"score": best["score"], "base_z": best["base_z"], "accepted": accepted}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
