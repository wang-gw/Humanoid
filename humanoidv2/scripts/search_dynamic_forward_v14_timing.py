#!/usr/bin/env python3
"""Search phase allocations that keep V14 at 113 control steps per footstep."""

from __future__ import annotations

import json
import sys
from dataclasses import replace
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from humanoidv2 import DynamicForwardV14Env  # noqa: E402


TIMINGS = (
    (45, 15, 23, 23, 7),
    (45, 20, 20, 21, 7),
    (45, 25, 18, 18, 7),
    (40, 25, 20, 21, 7),
    (50, 20, 18, 18, 7),
    (45, 30, 15, 16, 7),
    (50, 25, 15, 16, 7),
)


def evaluate(timing: tuple[int, int, int, int, int], first_side: str) -> dict[str, object]:
    env = DynamicForwardV14Env(reference_only=True, fixed_first_side=first_side)
    (
        env.lift_steps,
        env.lift_hold_steps,
        env.advance_steps,
        env.land_steps,
        env.settle_steps,
    ) = timing
    env.step_cycle_steps = sum(timing)
    env.config = replace(
        env.config, max_episode_steps=env.num_steps * env.step_cycle_steps
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
        "timing": {
            "lift": timing[0],
            "hold": timing[1],
            "advance": timing[2],
            "land": timing[3],
            "settle": timing[4],
            "total": sum(timing),
        },
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
    }
    env.close()
    return row


def main() -> None:
    rows = [
        evaluate(timing, side)
        for timing in TIMINGS
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
    output.mkdir(parents=True, exist_ok=True)
    result = {
        "control_dt_seconds": 0.04,
        "target_step_cycle_seconds": 4.5,
        "actual_step_cycle_seconds": 4.52,
        "rows": rows,
        "selected_timing": selected[0]["timing"],
        "selected_rows": selected,
    }
    (output / "timing_search.json").write_text(
        json.dumps(result, indent=2, allow_nan=False), encoding="utf-8"
    )
    print(json.dumps(result, indent=2, allow_nan=False))


if __name__ == "__main__":
    main()
