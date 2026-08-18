#!/usr/bin/env python3
"""Screen six-second phase allocations before V10 transfer learning."""

from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np
from stable_baselines3 import PPO

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from humanoidv2 import FastEightStepV9Env  # noqa: E402


TIMINGS = (
    (60, 30, 30, 30),
    (60, 30, 35, 25),
    (60, 30, 40, 20),
    (60, 30, 45, 15),
    (60, 30, 46, 14),
    (60, 30, 47, 13),
    (60, 30, 48, 12),
    (60, 30, 49, 11),
    (60, 30, 50, 10),
    (55, 30, 40, 25),
    (50, 30, 45, 25),
    (50, 25, 50, 25),
)


def evaluate(
    timing: tuple[int, int, int, int],
    first_side: str,
    policy: PPO | None,
) -> dict[str, object]:
    env = FastEightStepV9Env(reference_only=policy is None, fixed_first_side=first_side)
    env.lift_steps, env.advance_steps, env.land_steps, env.settle_steps = timing
    observation, _ = env.reset(seed=7)
    total_reward = 0.0
    impact_window_peaks = np.zeros(8, dtype=np.float64)
    for step in range(env.config.max_episode_steps):
        action = (
            np.zeros(10, dtype=np.float32)
            if policy is None
            else policy.predict(observation, deterministic=True)[0]
        )
        observation, reward, terminated, truncated, info = env.step(action)
        total_reward += reward
        phase_kind = str(info["task_phase"]).split("_", 1)[1]
        if phase_kind in {"land", "settle"}:
            step_index = int(info["active_step"]) - 1
            impact_window_peaks[step_index] = max(
                impact_window_peaks[step_index], float(info["swing_force_n"])
            )
        if terminated or truncated:
            break
    peaks = impact_window_peaks.tolist()
    result: dict[str, object] = {
        "mode": "reference" if policy is None else "v9_policy",
        "first_side": first_side,
        "lift_steps": timing[0],
        "advance_steps": timing[1],
        "land_steps": timing[2],
        "settle_steps": timing[3],
        "seconds_per_step": sum(timing) * env.config.control_dt,
        "steps": step + 1,
        "terminated": terminated,
        "gait_success": bool(info["is_success"]),
        "impact_limit_satisfied": bool(truncated and max(peaks) <= 50.0),
        "maximum_landing_force_n": max(peaks),
        "base_forward_displacement_m": float(info["base_forward_displacement_m"]),
        "total_reward": total_reward,
    }
    for index, peak in enumerate(peaks, start=1):
        result[f"step_{index}_peak_landing_force_n"] = peak
        result[f"step_{index}_advance_under_5n_fraction"] = float(
            info[f"step_{index}_advance_under_5n_fraction"]
        )
        result[f"step_{index}_length_m"] = float(info[f"step_{index}_length_m"])
        result[f"step_{index}_both_contact_fraction"] = float(
            info[f"step_{index}_both_contact_fraction"]
        )
    env.close()
    return result


def main() -> None:
    output = ROOT / "results/khr3hv_v10_symmetric"
    output.mkdir(parents=True, exist_ok=True)
    policy = PPO.load(
        ROOT / "results/khr3hv_v9_symmetric/ppo_fast_eight_step_final.zip",
        device="cpu",
    )
    rows = [
        evaluate(timing, side, candidate)
        for timing in TIMINGS
        for candidate in (None, policy)
        for side in ("left", "right")
    ]
    selected_timing = (60, 30, 50, 10)
    selected = [
        row
        for row in rows
        if tuple(row[key] for key in ("lift_steps", "advance_steps", "land_steps", "settle_steps"))
        == selected_timing
    ]
    candidate_comparison = []
    for timing in TIMINGS:
        candidates = [
            row
            for row in rows
            if row["mode"] == "v9_policy"
            and tuple(
                row[key]
                for key in ("lift_steps", "advance_steps", "land_steps", "settle_steps")
            )
            == timing
        ]
        candidate_comparison.append(
            {
                "timing_steps": list(timing),
                "both_order_gait_success": all(row["gait_success"] for row in candidates),
                "both_order_impact_success": all(
                    row["impact_limit_satisfied"] for row in candidates
                ),
                "worst_landing_force_n": max(
                    row["maximum_landing_force_n"] for row in candidates
                ),
            }
        )
    summary = {
        "impact_limit_n": 50.0,
        "selected_timing_steps": list(selected_timing),
        "selected_timing_seconds": [value * 0.04 for value in selected_timing],
        "selection_reason": (
            "keeps the V9 6 s cycle and both-order gait success while reducing the warm-start "
            "worst land+settle force from 65.82 N to 47.45 N with an unchanged 6 s cycle"
        ),
        "selected_results": selected,
        "candidate_comparison": candidate_comparison,
    }
    (output / "timing_search.json").write_text(
        json.dumps(rows, indent=2, allow_nan=False), encoding="utf-8"
    )
    (output / "timing_search_summary.json").write_text(
        json.dumps(summary, indent=2, allow_nan=False), encoding="utf-8"
    )
    print(json.dumps(summary, indent=2, allow_nan=False))


if __name__ == "__main__":
    main()
