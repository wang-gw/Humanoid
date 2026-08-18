#!/usr/bin/env python3
"""Compare original and symmetric inertials under identical PD commands."""

from __future__ import annotations

import itertools
import json
import sys
from dataclasses import replace
from pathlib import Path

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from humanoidv2 import KHR3HVEnv  # noqa: E402
from humanoidv2.khr3hv_env import KHRConfig  # noqa: E402
from scripts.search_single_support_reference import simulate  # noqa: E402

ORIGINAL = ROOT / "models/urdf_f_v2/URDF_F_v2_footprint_contact.xml"
SYMMETRIC = ROOT / "models/urdf_f_v2/URDF_F_v2_footprint_contact_symmetric_inertia.xml"


def standing_hold(model_path: Path) -> dict[str, float | bool | int]:
    config = replace(KHRConfig(), enhanced_collisions=True, kp=80.0, kd=0.32)
    env = KHR3HVEnv(model_path=model_path, config=config)
    env.reset(seed=7)
    target = env.data.qpos[env.qpos_ids].copy()
    records = []
    for step in range(100):
        for _ in range(env.frame_skip):
            error = target - env.data.qpos[env.qpos_ids]
            torque = config.kp * error - config.kd * env.data.qvel[env.dof_ids]
            env.data.ctrl[:] = np.clip(torque, -env.torque_limit, env.torque_limit)
            mujoco.mj_step(env.model, env.data)
        _, _, left_force, right_force = env._foot_contacts()
        records.append((left_force, right_force))
    recent = np.asarray(records[-50:])
    whole_com = (env.model.body_mass[:, None] * env.data.xipos).sum(axis=0) / env.model.body_mass.sum()
    feet_midpoint = 0.5 * (env.data.xpos[env.left_foot_id] + env.data.xpos[env.right_foot_id])
    result = {
        "steps": step + 1,
        "survived": bool(env.data.xipos[env.base_id, 2] >= env.fall_height),
        "left_mean_force_n": float(recent[:, 0].mean()),
        "right_mean_force_n": float(recent[:, 1].mean()),
        "absolute_force_difference_n": float(abs(recent[:, 0].mean() - recent[:, 1].mean())),
        "final_com_lateral_offset_m": float(whole_com[0] - feet_midpoint[0]),
        "final_torso_height_m": float(env.data.xipos[env.base_id, 2]),
    }
    env.close()
    return result


def common_parameter_search() -> tuple[list[dict[str, object]], list[dict[str, object]]]:
    widths = (0.040, 0.050, 0.060, 0.070, 0.080, 0.090, 0.100)
    heights = (0.035, 0.045, 0.055)
    gains = (30.0, 50.0, 65.0, 80.0)
    rows = [
        simulate(width, height, 4.0, kp, side, SYMMETRIC)
        for width, height, kp, side in itertools.product(widths, heights, gains, ("left", "right"))
    ]
    pairs = []
    for width, height, kp in itertools.product(widths, heights, gains):
        matched = [
            row for row in rows
            if row["sway_width_m"] == width and row["foot_height_m"] == height and row["kp"] == kp
        ]
        left = next(row for row in matched if row["side"] == "left")
        right = next(row for row in matched if row["side"] == "right")
        if left["success"] and right["success"]:
            symmetry_error = (
                abs(left["mean_swing_force_n"] - right["mean_swing_force_n"])
                + abs(left["mean_stance_force_n"] - right["mean_stance_force_n"])
                + 100.0 * abs(left["max_swing_height_m"] - right["max_swing_height_m"])
            )
            pairs.append(
                {
                    "sway_width_m": width,
                    "foot_height_m": height,
                    "kp": kp,
                    "kd": 0.004 * kp,
                    "symmetry_error": symmetry_error,
                    "left": left,
                    "right": right,
                }
            )
    return rows, sorted(pairs, key=lambda row: row["symmetry_error"])


def main() -> None:
    output_dir = ROOT / "results/inertia_symmetry"
    output_dir.mkdir(parents=True, exist_ok=True)
    rows, pairs = common_parameter_search()
    selected = pairs[0]
    parameters = (
        selected["sway_width_m"],
        selected["foot_height_m"],
        4.0,
        selected["kp"],
    )
    comparison = {
        "standing_hold": {
            "original": standing_hold(ORIGINAL),
            "symmetric": standing_hold(SYMMETRIC),
        },
        "common_search": {
            "evaluations": len(rows),
            "single_side_successes": sum(bool(row["success"]) for row in rows),
            "both_side_same_parameter_successes": len(pairs),
            "selected": selected,
            "all_successful_pairs": pairs,
        },
        "selected_identical_command_comparison": {
            "original": {
                side: simulate(*parameters, side, ORIGINAL) for side in ("left", "right")
            },
            "symmetric": {
                side: simulate(*parameters, side, SYMMETRIC) for side in ("left", "right")
            },
        },
    }
    (output_dir / "symmetric_common_parameter_search.json").write_text(
        json.dumps(rows, indent=2), encoding="utf-8"
    )
    (output_dir / "dynamics_comparison.json").write_text(
        json.dumps(comparison, indent=2), encoding="utf-8"
    )
    print(json.dumps(comparison, indent=2))


if __name__ == "__main__":
    main()
