#!/usr/bin/env python3
"""Search reference parameters after correcting anatomical forward to world -Y."""

from __future__ import annotations

import itertools
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from humanoidv2 import CorrectedForwardV13Env  # noqa: E402


def simulate(stride: float, width: float, kp: float, first_side: str) -> dict[str, object]:
    env = CorrectedForwardV13Env(
        reference_only=True,
        fixed_first_side=first_side,
        stride_m=stride,
        sway_width_m=width,
        kp=kp,
    )
    observation, _ = env.reset(seed=7)
    start_world_y = float(env.data.qpos[1])
    for step in range(env.config.max_episode_steps):
        observation, reward, terminated, truncated, info = env.step(
            np.zeros(10, dtype=np.float32)
        )
        if terminated or truncated:
            break
    result: dict[str, object] = {
        "first_side": first_side,
        "forward_axis": info["forward_axis"],
        "stride_m": stride,
        "sway_width_m": width,
        "kp": kp,
        "steps": step + 1,
        "terminated": terminated,
        "truncated": truncated,
        "gait_success": bool(info["gait_success"]),
        "impact_limit_satisfied": bool(info["impact_limit_satisfied"]),
        "success": bool(info["is_success"]),
        "world_y_displacement_m": float(env.data.qpos[1] - start_world_y),
        "base_forward_displacement_m": float(info["base_forward_displacement_m"]),
        "maximum_landing_force_n": float(info["maximum_landing_force_n"]),
        "minimum_step_length_m": min(
            float(info[f"step_{index}_length_m"]) for index in range(1, 9)
        ),
        "worst_advance_mean_force_n": max(
            float(info[f"step_{index}_advance_mean_force_n"]) for index in range(1, 9)
        ),
        "minimum_advance_under_5n_fraction": min(
            float(info[f"step_{index}_advance_under_5n_fraction"])
            for index in range(1, 9)
        ),
        "minimum_both_contact_fraction": min(
            float(info[f"step_{index}_both_contact_fraction"]) for index in range(1, 9)
        ),
    }
    env.close()
    return result


def main() -> None:
    output = ROOT / "results/khr3hv_v13_corrected_forward"
    output.mkdir(parents=True, exist_ok=True)
    parameters = tuple(
        itertools.product(
            (0.025, 0.030, 0.035),
            (0.085, 0.090, 0.095, 0.100),
            (70.0, 80.0, 90.0),
        )
    )
    rows = [
        simulate(stride, width, kp, side)
        for stride, width, kp in parameters
        for side in ("left", "right")
    ]
    pairs = []
    for stride, width, kp in parameters:
        pair = [
            row
            for row in rows
            if (row["stride_m"], row["sway_width_m"], row["kp"])
            == (stride, width, kp)
        ]
        pairs.append(
            {
                "stride_m": stride,
                "sway_width_m": width,
                "kp": kp,
                "both_order_success": all(bool(row["success"]) for row in pair),
                "both_order_survive": all(not bool(row["terminated"]) for row in pair),
                "minimum_forward_m": min(float(row["base_forward_displacement_m"]) for row in pair),
                "worst_landing_force_n": max(float(row["maximum_landing_force_n"]) for row in pair),
                "worst_advance_mean_force_n": max(float(row["worst_advance_mean_force_n"]) for row in pair),
                "minimum_advance_under_5n_fraction": min(
                    float(row["minimum_advance_under_5n_fraction"]) for row in pair
                ),
            }
        )
    selected = next(
        pair
        for pair in pairs
        if (pair["stride_m"], pair["sway_width_m"], pair["kp"])
        == (0.035, 0.090, 90.0)
    )
    summary = {
        "forward_axis": "-Y",
        "evaluations": len(rows),
        "successful_references": sum(bool(row["success"]) for row in rows),
        "selection_reason": (
            "largest tested stride with both orders surviving, over 0.24 m forward progress, "
            "under-50 N landing peaks, and the lowest near-feasible swing-contact metrics"
        ),
        "selected": selected,
        "selected_orders": [
            row
            for row in rows
            if (row["stride_m"], row["sway_width_m"], row["kp"])
            == (0.035, 0.090, 90.0)
        ],
    }
    (output / "reference_search.json").write_text(
        json.dumps(rows, indent=2, allow_nan=False), encoding="utf-8"
    )
    (output / "reference_search_summary.json").write_text(
        json.dumps(summary, indent=2, allow_nan=False), encoding="utf-8"
    )
    print(json.dumps(summary, indent=2, allow_nan=False))


if __name__ == "__main__":
    main()
