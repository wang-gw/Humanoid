#!/usr/bin/env python3
"""Calibrate V21 lift-gate strength without retraining its saved checkpoints."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

from stable_baselines3 import PPO

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from scripts.train_landing_residual_v19 import combined_domain  # noqa: E402
from scripts.train_phase_split_residual_v21 import (  # noqa: E402
    BASE,
    OUTPUT,
    run_episode,
    selection_key,
)


CALIBRATION_GRID = (
    ("ppo_phase_split_v21_initial.zip", 0.0),
    ("ppo_phase_split_v21_stage0.zip", 0.0),
    ("ppo_phase_split_v21_stage0.zip", 0.10),
    ("ppo_phase_split_v21_stage1.zip", 0.10),
    ("ppo_phase_split_v21_stage1.zip", 0.20),
    ("ppo_phase_split_v21_stage2.zip", 0.10),
    ("ppo_phase_split_v21_stage2.zip", 0.20),
)


def evaluate_configuration(
    model_path: Path, gate_scale: float, base_path: Path
) -> list[dict[str, object]]:
    corrector = PPO.load(model_path, device="cpu")
    base = PPO.load(base_path, device="cpu")
    configuration = f"{model_path.stem}_gate{gate_scale:.2f}"
    rows: list[dict[str, object]] = []
    for side in ("left", "right"):
        row = run_episode(
            corrector, base, side, 17, {}, conditional_lift_gate_scale=gate_scale
        )
        row.update(
            {
                "model": model_path.name,
                "configuration": configuration,
                "conditional_lift_gate_scale": gate_scale,
                "case": "nominal",
            }
        )
        rows.append(row)
    for sample, seed in enumerate(range(101, 121), start=1):
        for side in ("left", "right"):
            row = run_episode(
                corrector,
                base,
                side,
                seed,
                combined_domain(seed),
                conditional_lift_gate_scale=gate_scale,
            )
            row.update(
                {
                    "model": model_path.name,
                    "configuration": configuration,
                    "conditional_lift_gate_scale": gate_scale,
                    "case": "combined",
                    "sample": sample,
                }
            )
            rows.append(row)
    return rows


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--base-policy", type=Path, default=BASE)
    parser.add_argument("--output", type=Path, default=OUTPUT)
    args = parser.parse_args()

    all_rows: list[dict[str, object]] = []
    scores: list[dict[str, object]] = []
    for filename, gate_scale in CALIBRATION_GRID:
        model_path = args.output / filename
        print(f"evaluating {filename} gate={gate_scale:.2f}", flush=True)
        rows = evaluate_configuration(model_path, gate_scale, args.base_policy)
        all_rows.extend(rows)
        nominal = [row for row in rows if row["case"] == "nominal"]
        combined = [row for row in rows if row["case"] == "combined"]
        key = selection_key(rows)
        score = {
            "model": filename,
            "conditional_lift_gate_scale": gate_scale,
            "nominal_successes": sum(row["success"] for row in nominal),
            "combined_successes": sum(row["success"] for row in combined),
            "both_sides_successful_samples": sum(
                all(row["success"] for row in combined if row["sample"] == sample)
                for sample in range(1, 21)
            ),
            "worst_landing_force_n": max(
                float(row["maximum_landing_force_n"]) for row in combined
            ),
            "minimum_forward_displacement_m": min(
                float(row["base_forward_displacement_m"]) for row in rows
            ),
            "selection_key": list(key),
        }
        scores.append(score)
        print(json.dumps(score, allow_nan=False), flush=True)

    selected = max(scores, key=lambda score: tuple(score["selection_key"]))
    selected_path = args.output / str(selected["model"])
    PPO.load(selected_path, device="cpu").save(
        args.output / "ppo_phase_split_v21_final"
    )
    deployment = {
        "selected_model": selected["model"],
        "conditional_lift_gate_scale": selected["conditional_lift_gate_scale"],
        "early_gate_blend": 0.75,
        "score": selected,
    }
    (args.output / "calibration_results.json").write_text(
        json.dumps(all_rows, indent=2, allow_nan=False), encoding="utf-8"
    )
    (args.output / "calibration_summary.json").write_text(
        json.dumps({"scores": scores, "selected": deployment}, indent=2),
        encoding="utf-8",
    )
    (args.output / "deployment_config.json").write_text(
        json.dumps(deployment, indent=2), encoding="utf-8"
    )
    print(json.dumps(deployment, indent=2), flush=True)


if __name__ == "__main__":
    main()
