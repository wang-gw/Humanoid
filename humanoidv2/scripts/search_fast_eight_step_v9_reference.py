#!/usr/bin/env python3
"""Compare temporal compression and V9 six-second reference candidates."""

from __future__ import annotations

import itertools
import json
import sys
from dataclasses import replace
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from humanoidv2 import EightStepV8Env, FastEightStepV9Env  # noqa: E402


def summarize(env: EightStepV8Env, label: str) -> dict[str, object]:
    observation, _ = env.reset(seed=7)
    terminated = truncated = False
    for _ in range(env.config.max_episode_steps):
        observation, reward, terminated, truncated, info = env.step(np.zeros(10, dtype=np.float32))
        if terminated or truncated:
            break
    result: dict[str, object] = {
        "label": label,
        "first_side": info["first_side"],
        "steps": env._step_count,
        "duration_seconds": env._step_count * env.config.control_dt,
        "terminated": terminated,
        "success": bool(info["is_success"]),
        "base_forward_displacement_m": float(info["base_forward_displacement_m"]),
    }
    for index in range(1, 9):
        result[f"step_{index}_advance_mean_force_n"] = float(info[f"step_{index}_advance_mean_force_n"])
        result[f"step_{index}_advance_under_5n_fraction"] = float(info[f"step_{index}_advance_under_5n_fraction"])
        result[f"step_{index}_length_m"] = float(info[f"step_{index}_length_m"])
        result[f"step_{index}_both_contact_fraction"] = float(info[f"step_{index}_both_contact_fraction"])
    env.close()
    return result


def temporal(scale: float, side: str) -> dict[str, object]:
    env = EightStepV8Env(reference_only=True, fixed_first_side=side)
    env.lift_steps = round(100 * scale)
    env.advance_steps = env.land_steps = env.settle_steps = round(50 * scale)
    env.step_cycle_steps = env.lift_steps + env.advance_steps + env.land_steps + env.settle_steps
    env.config = replace(env.config, max_episode_steps=8 * env.step_cycle_steps)
    return summarize(env, f"{10.0 * scale:.0f}s_per_step")


def candidate(lift: float, width: float, kp: float, side: str) -> dict[str, object]:
    env = FastEightStepV9Env(
        reference_only=True,
        fixed_first_side=side,
        lift_m=lift,
        sway_width_m=width,
        kp=kp,
    )
    result = summarize(env, "6s_spatial_candidate")
    result.update({"lift_m": lift, "sway_width_m": width, "kp": kp, "kd": 0.004 * kp})
    return result


def main() -> None:
    output = ROOT / "results/khr3hv_v9_symmetric"
    output.mkdir(parents=True, exist_ok=True)
    temporal_rows = [
        temporal(scale, side)
        for scale, side in itertools.product((0.8, 0.6, 0.4), ("left", "right"))
    ]
    spatial_rows = [
        candidate(lift, width, kp, side)
        for lift, width, kp, side in itertools.product(
            (0.030, 0.035), (0.090, 0.100), (80.0, 85.0), ("left", "right")
        )
    ]
    selected = [
        row for row in spatial_rows
        if row["lift_m"] == 0.030 and row["sway_width_m"] == 0.090 and row["kp"] == 80.0
    ]
    summary = {
        "temporal_evaluations": len(temporal_rows),
        "spatial_evaluations": len(spatial_rows),
        "strict_successes_by_temporal_scale": {
            label: sum(row["success"] for row in temporal_rows if row["label"] == label)
            for label in ("8s_per_step", "6s_per_step", "4s_per_step")
        },
        "six_second_same_parameter_both_order_successes": sum(
            all(row["success"] for row in spatial_rows if row["lift_m"] == lift and row["sway_width_m"] == width and row["kp"] == kp)
            for lift, width, kp in itertools.product((0.030, 0.035), (0.090, 0.100), (80.0, 85.0))
        ),
        "selected_learning_target": selected,
        "selection_reason": "symmetric no-fall candidate; PPO must raise under-5N fraction above 0.9",
        "temporal_comparison": temporal_rows,
        "spatial_search": spatial_rows,
    }
    (output / "reference_search.json").write_text(
        json.dumps({"temporal": temporal_rows, "spatial": spatial_rows}, indent=2, allow_nan=False), encoding="utf-8"
    )
    (output / "reference_search_summary.json").write_text(
        json.dumps(summary, indent=2, allow_nan=False), encoding="utf-8"
    )
    print(json.dumps(summary, indent=2, allow_nan=False))


if __name__ == "__main__":
    main()
