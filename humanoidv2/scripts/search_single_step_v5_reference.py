#!/usr/bin/env python3
"""Search symmetric PD/IK references for one forward step."""

from __future__ import annotations

import itertools
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from humanoidv2 import SingleStepV5Env  # noqa: E402


def simulate(stride: float, kp: float, landing_steps: int, side: str) -> dict[str, object]:
    env = SingleStepV5Env(
        reference_only=True,
        fixed_side=side,
        stride_m=stride,
        kp=kp,
        landing_steps=landing_steps,
    )
    observation, _ = env.reset(seed=7)
    records = []
    terminated = truncated = False
    for _ in range(env.config.max_episode_steps):
        observation, reward, terminated, truncated, info = env.step(np.zeros(10, dtype=np.float32))
        records.append(info)
        if terminated or truncated:
            break
    advance = [row for row in records if row["task_phase"] == "advance"]
    settle = [row for row in records if row["task_phase"] == "settle"]
    result = {
        "side": side,
        "stride_m": stride,
        "kp": kp,
        "kd": 0.004 * kp,
        "landing_steps": landing_steps,
        "steps": len(records),
        "terminated": terminated,
        "success": bool(info["is_success"]),
        "advance_mean_swing_force_n": float(np.mean([row["swing_force_n"] for row in advance])),
        "advance_under_5n_fraction": float(np.mean([row["swing_force_n"] < 5.0 for row in advance])),
        "final_left_force_n": float(np.mean([row["left_foot_force_n"] for row in settle])),
        "final_right_force_n": float(np.mean([row["right_foot_force_n"] for row in settle])),
        "final_both_contact_fraction": float(info["final_both_contact_fraction"]),
        "final_step_length_m": float(info["step_length_m"]),
        "maximum_swing_height_m": float(max(row["swing_height_m"] for row in records)),
        "peak_landing_force_n": float(info["peak_landing_force_n"]),
        "base_forward_displacement_m": float(info["base_forward_displacement_m"]),
    }
    env.close()
    return result


def main() -> None:
    output_dir = ROOT / "results/khr3hv_v5_symmetric"
    output_dir.mkdir(parents=True, exist_ok=True)
    rows = [
        simulate(stride, kp, landing_steps, side)
        for stride, kp, landing_steps, side in itertools.product(
            (0.010, 0.020, 0.030, 0.040, 0.050),
            (50.0, 65.0, 80.0),
            (50, 75, 100),
            ("left", "right"),
        )
    ]
    successful_pairs = []
    for stride, kp, landing_steps in itertools.product(
        (0.010, 0.020, 0.030, 0.040, 0.050), (50.0, 65.0, 80.0), (50, 75, 100)
    ):
        pair = [
            row for row in rows
            if row["stride_m"] == stride and row["kp"] == kp and row["landing_steps"] == landing_steps
        ]
        if all(row["success"] for row in pair):
            successful_pairs.append(
                {
                    "stride_m": stride,
                    "kp": kp,
                    "kd": 0.004 * kp,
                    "landing_steps": landing_steps,
                    "left": next(row for row in pair if row["side"] == "left"),
                    "right": next(row for row in pair if row["side"] == "right"),
                }
            )
    summary = {
        "evaluations": len(rows),
        "successes": sum(bool(row["success"]) for row in rows),
        "both_side_same_parameter_successes": len(successful_pairs),
        "selected": next(
            pair for pair in successful_pairs
            if pair["stride_m"] == 0.030 and pair["kp"] == 80.0 and pair["landing_steps"] == 50
        ),
        "successful_pairs": successful_pairs,
    }
    (output_dir / "reference_search.json").write_text(json.dumps(rows, indent=2), encoding="utf-8")
    (output_dir / "reference_search_summary.json").write_text(
        json.dumps(summary, indent=2), encoding="utf-8"
    )
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
