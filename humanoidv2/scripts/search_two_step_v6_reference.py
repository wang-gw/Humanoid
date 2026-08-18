#!/usr/bin/env python3
"""Search symmetric reference parameters for two alternating steps."""

from __future__ import annotations

import itertools
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from humanoidv2 import TwoStepV6Env  # noqa: E402


def simulate(stride: float, width: float, kp: float, first_side: str) -> dict[str, object]:
    env = TwoStepV6Env(
        reference_only=True,
        fixed_first_side=first_side,
        stride_m=stride,
        sway_width_m=width,
        kp=kp,
    )
    observation, _ = env.reset(seed=7)
    records = []
    terminated = truncated = False
    for _ in range(env.config.max_episode_steps):
        observation, reward, terminated, truncated, info = env.step(np.zeros(10, dtype=np.float32))
        records.append(info)
        if terminated or truncated:
            break
    first_advance = [row for row in records if row["task_phase"] == "first_advance"]
    second_advance = [row for row in records if row["task_phase"] == "second_advance"]
    result = {
        "first_side": first_side,
        "stride_m": stride,
        "sway_width_m": width,
        "kp": kp,
        "kd": 0.004 * kp,
        "steps": len(records),
        "terminated": terminated,
        "success": bool(info["is_success"]),
        "first_advance_mean_force_n": float(np.mean([row["swing_force_n"] for row in first_advance])),
        "second_advance_mean_force_n": float(np.mean([row["swing_force_n"] for row in second_advance])),
        "first_advance_under_5n_fraction": float(np.mean([row["swing_force_n"] < 5.0 for row in first_advance])),
        "second_advance_under_5n_fraction": float(np.mean([row["swing_force_n"] < 5.0 for row in second_advance])),
        "first_step_length_m": float(info["first_step_length_m"]),
        "second_step_length_m": float(info["active_step_length_m"]),
        "base_forward_displacement_m": float(info["base_forward_displacement_m"]),
        "first_both_contact_fraction": float(info["first_both_contact_fraction"]),
        "final_both_contact_fraction": float(info["final_both_contact_fraction"]),
        "first_peak_landing_force_n": float(info["first_peak_landing_force_n"]),
        "second_peak_landing_force_n": float(info["second_peak_landing_force_n"]),
        "maximum_swing_height_m": float(max(row["swing_height_m"] for row in records)),
    }
    env.close()
    return result


def main() -> None:
    output_dir = ROOT / "results/khr3hv_v6_symmetric"
    output_dir.mkdir(parents=True, exist_ok=True)
    rows = [
        simulate(stride, width, kp, first_side)
        for stride, width, kp, first_side in itertools.product(
            (0.020, 0.030, 0.040),
            (0.080, 0.090, 0.100),
            (65.0, 80.0),
            ("left", "right"),
        )
    ]
    pairs = []
    for stride, width, kp in itertools.product(
        (0.020, 0.030, 0.040), (0.080, 0.090, 0.100), (65.0, 80.0)
    ):
        pair = [
            row for row in rows
            if row["stride_m"] == stride and row["sway_width_m"] == width and row["kp"] == kp
        ]
        if all(row["success"] for row in pair):
            pairs.append(
                {
                    "stride_m": stride,
                    "sway_width_m": width,
                    "kp": kp,
                    "kd": 0.004 * kp,
                    "left_first": next(row for row in pair if row["first_side"] == "left"),
                    "right_first": next(row for row in pair if row["first_side"] == "right"),
                }
            )
    summary = {
        "evaluations": len(rows),
        "successes": sum(bool(row["success"]) for row in rows),
        "both_order_same_parameter_successes": len(pairs),
        "selected": next(
            pair for pair in pairs
            if pair["stride_m"] == 0.030 and pair["sway_width_m"] == 0.090 and pair["kp"] == 80.0
        ),
        "successful_pairs": pairs,
    }
    (output_dir / "reference_search.json").write_text(json.dumps(rows, indent=2), encoding="utf-8")
    (output_dir / "reference_search_summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
