#!/usr/bin/env python3
"""Search timing and gain scheduling for the 64-step V16 reference."""

from __future__ import annotations

import json
import sys
from dataclasses import replace
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from humanoidv2 import DynamicForwardV16Env  # noqa: E402


TIMINGS = (
    (24, 8, 11, 16, 5),
    (22, 8, 10, 19, 5),
    (20, 8, 10, 21, 5),
    (20, 7, 9, 23, 5),
    (18, 8, 9, 24, 5),
    (20, 8, 9, 22, 5),
    (22, 8, 9, 20, 5),
    (20, 9, 9, 22, 4),
)


def evaluate(timing: tuple[int, ...], landing_kp: float, first_side: str):
    env = DynamicForwardV16Env(
        reference_only=True,
        fixed_first_side=first_side,
        kp=130.0,
        landing_kp=landing_kp,
        gain_transition_fraction=0.0,
    )
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
    for step in range(env.config.max_episode_steps):
        observation, reward, terminated, truncated, info = env.step(
            np.zeros(10, dtype=np.float32)
        )
        if terminated or truncated:
            break
    row = {
        "timing": {
            "lift": timing[0], "hold": timing[1], "advance": timing[2],
            "land": timing[3], "settle": timing[4], "total": sum(timing),
        },
        "kp": 130.0,
        "landing_kp": landing_kp,
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
    rows = [
        evaluate(timing, landing_kp, side)
        for timing in TIMINGS
        for landing_kp in (50.0, 60.0, 70.0, 80.0, 90.0, 100.0, 110.0)
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
    result = {
        "evaluations": len(rows),
        "rows": rows,
        "selected_timing": selected[0]["timing"],
        "selected_landing_kp": selected[0]["landing_kp"],
        "selected_rows": selected,
    }
    output = ROOT / "results/khr3hv_v16_dynamic_forward_stage3"
    output.mkdir(parents=True, exist_ok=True)
    (output / "reference_search.json").write_text(
        json.dumps(result, indent=2, allow_nan=False), encoding="utf-8"
    )
    print(json.dumps(result, indent=2, allow_nan=False))


if __name__ == "__main__":
    main()
