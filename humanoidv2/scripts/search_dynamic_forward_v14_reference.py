#!/usr/bin/env python3
"""Search V14 reference parameters at the fixed 4.52-second step cycle."""

from __future__ import annotations

import itertools
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from humanoidv2 import DynamicForwardV14Env  # noqa: E402


def evaluate(lift_m: float, sway_width_m: float, kp: float, first_side: str):
    env = DynamicForwardV14Env(
        reference_only=True,
        fixed_first_side=first_side,
        lift_m=lift_m,
        sway_width_m=sway_width_m,
        kp=kp,
    )
    observation, _ = env.reset(seed=17)
    start_world_y = float(env.data.qpos[1])
    for step in range(env.config.max_episode_steps):
        observation, reward, terminated, truncated, info = env.step(
            np.zeros(10, dtype=np.float32)
        )
        if terminated or truncated:
            break
    row = {
        "lift_m": lift_m,
        "sway_width_m": sway_width_m,
        "kp": kp,
        "first_side": first_side,
        "steps": step + 1,
        "terminated": terminated,
        "truncated": truncated,
        "gait_success": bool(info["gait_success"]),
        "impact_limit_satisfied": bool(info["impact_limit_satisfied"]),
        "success": bool(info["is_success"]),
        "forward_displacement_m": float(info["base_forward_displacement_m"]),
        "maximum_landing_force_n": float(info["maximum_landing_force_n"]),
        "minimum_step_length_m": min(
            float(info[f"step_{index}_length_m"]) for index in range(1, 9)
        ),
        "maximum_advance_mean_force_n": max(
            float(info[f"step_{index}_advance_mean_force_n"]) for index in range(1, 9)
        ),
        "minimum_advance_under_5n_fraction": min(
            float(info[f"step_{index}_advance_under_5n_fraction"])
            for index in range(1, 9)
        ),
        "minimum_both_contact_fraction": min(
            float(info[f"step_{index}_both_contact_fraction"])
            for index in range(1, 9)
        ),
    }
    env.close()
    return row


def main() -> None:
    parameter_sets = itertools.product(
        (0.030, 0.035, 0.040, 0.045, 0.050),
        (0.090, 0.100, 0.110),
        (90.0, 110.0),
    )
    rows = [
        evaluate(lift, sway, kp, side)
        for lift, sway, kp in parameter_sets
        for side in ("left", "right")
    ]
    pairs = [rows[index:index + 2] for index in range(0, len(rows), 2)]
    selected = max(
        pairs,
        key=lambda pair: (
            sum(bool(row["success"]) for row in pair),
            sum(bool(row["gait_success"]) for row in pair),
            sum(bool(row["impact_limit_satisfied"]) for row in pair),
            min(float(row["minimum_advance_under_5n_fraction"]) for row in pair),
            -max(float(row["maximum_landing_force_n"]) for row in pair),
            min(float(row["forward_displacement_m"]) for row in pair),
        ),
    )
    output = ROOT / "results/khr3hv_v14_dynamic_forward_stage1"
    result = {
        "evaluations": len(rows),
        "rows": rows,
        "selected_parameters": {
            "lift_m": selected[0]["lift_m"],
            "sway_width_m": selected[0]["sway_width_m"],
            "kp": selected[0]["kp"],
        },
        "selected_rows": selected,
    }
    (output / "reference_search.json").write_text(
        json.dumps(result, indent=2, allow_nan=False), encoding="utf-8"
    )
    print(json.dumps(result, indent=2, allow_nan=False))


if __name__ == "__main__":
    main()
