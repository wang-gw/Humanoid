#!/usr/bin/env python3
"""Search symmetric reference parameters for four alternating steps."""

from __future__ import annotations

import itertools
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from humanoidv2 import FourStepV7Env  # noqa: E402


def simulate(stride: float, width: float, kp: float, first_side: str) -> dict[str, object]:
    env = FourStepV7Env(
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
    result: dict[str, object] = {
        "first_side": first_side,
        "order": info["step_order"],
        "stride_m": stride,
        "sway_width_m": width,
        "kp": kp,
        "kd": 0.004 * kp,
        "steps": len(records),
        "terminated": terminated,
        "success": bool(info["is_success"]),
        "base_forward_displacement_m": float(info["base_forward_displacement_m"]),
        "maximum_swing_height_m": float(max(row["swing_height_m"] for row in records)),
    }
    for index in range(1, 5):
        result[f"step_{index}_advance_mean_force_n"] = float(info[f"step_{index}_advance_mean_force_n"])
        result[f"step_{index}_advance_under_5n_fraction"] = float(info[f"step_{index}_advance_under_5n_fraction"])
        result[f"step_{index}_length_m"] = float(info[f"step_{index}_length_m"])
        result[f"step_{index}_both_contact_fraction"] = float(info[f"step_{index}_both_contact_fraction"])
        result[f"step_{index}_peak_landing_force_n"] = float(info[f"step_{index}_peak_landing_force_n"])
    env.close()
    return result


def main() -> None:
    output_dir = ROOT / "results/khr3hv_v7_symmetric"
    output_dir.mkdir(parents=True, exist_ok=True)
    strides = (0.020, 0.030, 0.040)
    widths = (0.080, 0.090, 0.100)
    gains = (65.0, 80.0)
    rows = [
        simulate(stride, width, kp, first_side)
        for stride, width, kp, first_side in itertools.product(
            strides, widths, gains, ("left", "right")
        )
    ]
    pairs = []
    for stride, width, kp in itertools.product(strides, widths, gains):
        pair = [
            row for row in rows
            if row["stride_m"] == stride and row["sway_width_m"] == width and row["kp"] == kp
        ]
        if all(row["success"] for row in pair):
            pairs.append({
                "stride_m": stride,
                "sway_width_m": width,
                "kp": kp,
                "kd": 0.004 * kp,
                "left_first": next(row for row in pair if row["first_side"] == "left"),
                "right_first": next(row for row in pair if row["first_side"] == "right"),
            })
    selected = next(
        pair for pair in pairs
        if pair["stride_m"] == 0.030 and pair["sway_width_m"] == 0.090 and pair["kp"] == 80.0
    )
    summary = {
        "evaluations": len(rows),
        "successes": sum(bool(row["success"]) for row in rows),
        "both_order_same_parameter_successes": len(pairs),
        "selected": selected,
        "successful_pairs": pairs,
    }
    (output_dir / "reference_search.json").write_text(json.dumps(rows, indent=2, allow_nan=False), encoding="utf-8")
    (output_dir / "reference_search_summary.json").write_text(json.dumps(summary, indent=2, allow_nan=False), encoding="utf-8")
    print(json.dumps(summary, indent=2, allow_nan=False))


if __name__ == "__main__":
    main()
