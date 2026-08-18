#!/usr/bin/env python3
"""Locate V10 policy tolerance to left/right leg mass asymmetry."""

from __future__ import annotations

import json
import sys
from pathlib import Path

from stable_baselines3 import PPO

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from evaluate_robust_soft_landing_v11 import MILD_NOISE, run_episode  # noqa: E402


ASYMMETRIES = (-0.03, -0.02, -0.01, -0.005, 0.0, 0.005, 0.01, 0.02, 0.03)
MILD_SEEDS = (7, 17, 29)


def main() -> None:
    output = ROOT / "results/khr3hv_v11_robustness"
    output.mkdir(parents=True, exist_ok=True)
    policy = PPO.load(
        ROOT / "results/khr3hv_v10_symmetric/ppo_soft_landing_final.zip", device="cpu"
    )
    rows = []
    total = len(ASYMMETRIES) * 2 * (1 + len(MILD_SEEDS))
    completed = 0
    for asymmetry in ASYMMETRIES:
        for noise_label, noise, seeds in (
            ("none", {}, (7,)),
            ("mild", MILD_NOISE, MILD_SEEDS),
        ):
            params = {"leg_mass_asymmetry": asymmetry, **noise}
            for seed in seeds:
                for first_side in ("left", "right"):
                    row, _ = run_episode(
                        policy,
                        "asymmetry_sweep",
                        first_side,
                        seed,
                        params_override=params,
                    )
                    row["noise_label"] = noise_label
                    rows.append(row)
                    completed += 1
                    if completed % 12 == 0 or completed == total:
                        print(f"completed={completed}/{total}", flush=True)

    aggregates = []
    for asymmetry in ASYMMETRIES:
        for noise_label in ("none", "mild"):
            selected = [
                row
                for row in rows
                if row["leg_mass_asymmetry"] == asymmetry
                and row["noise_label"] == noise_label
            ]
            aggregates.append(
                {
                    "leg_mass_asymmetry": asymmetry,
                    "noise_label": noise_label,
                    "runs": len(selected),
                    "successes": sum(row["success"] for row in selected),
                    "success_rate": sum(row["success"] for row in selected) / len(selected),
                    "falls": sum(row["failure_reason"] == "fall" for row in selected),
                    "unload_failures": sum(
                        row["failure_reason"] == "swing_unload" for row in selected
                    ),
                    "impact_failures": sum(
                        row["failure_reason"] == "impact_limit" for row in selected
                    ),
                    "worst_landing_force_n": max(
                        row["maximum_landing_force_n"] for row in selected
                    ),
                }
            )
    summary = {
        "definition": (
            "positive asymmetry scales every left-leg mass/inertia by 1+a and every "
            "right-leg mass/inertia by 1-a; negative does the reverse"
        ),
        "asymmetries": list(ASYMMETRIES),
        "mild_initial_noise": MILD_NOISE,
        "runs": len(rows),
        "aggregates": aggregates,
    }
    (output / "asymmetry_sweep_results.json").write_text(
        json.dumps(rows, indent=2, allow_nan=False), encoding="utf-8"
    )
    (output / "asymmetry_sweep_summary.json").write_text(
        json.dumps(summary, indent=2, allow_nan=False), encoding="utf-8"
    )
    print(json.dumps(summary, indent=2, allow_nan=False))


if __name__ == "__main__":
    main()

