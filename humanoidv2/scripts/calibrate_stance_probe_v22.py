#!/usr/bin/env python3
"""Calibrate the symmetric first-step stance hip-roll correction on all 40 cases."""

from __future__ import annotations

import argparse
import concurrent.futures
import json
import multiprocessing as mp
import sys
from collections import Counter, defaultdict
from pathlib import Path

import torch
from stable_baselines3 import PPO

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from humanoidv2 import FirstStepStanceHipRollV22Env  # noqa: E402
from scripts.diagnose_counterfactual_v22 import (  # noqa: E402
    BASE,
    CORRECTOR,
    OUTPUT,
    step_diagnostics,
)
from scripts.train_landing_residual_v19 import combined_domain  # noqa: E402


AMPLITUDES = (0.0, 0.004, 0.005, 0.006, 0.007, 0.008, 0.010, 0.012)
_BASE_POLICY: PPO | None = None
_CORRECTOR_POLICY: PPO | None = None


def initialize_worker(base_path: str, corrector_path: str) -> None:
    global _BASE_POLICY, _CORRECTOR_POLICY
    torch.set_num_threads(1)
    _BASE_POLICY = PPO.load(base_path, device="cpu")
    _CORRECTOR_POLICY = PPO.load(corrector_path, device="cpu")


def run_case(spec: tuple[float, str, int, bool]) -> dict[str, object]:
    amplitude, side, seed, is_combined = spec
    assert _BASE_POLICY is not None and _CORRECTOR_POLICY is not None
    env = FirstStepStanceHipRollV22Env(
        base_policy=_BASE_POLICY,
        fixed_first_side=side,
        fixed_domain=combined_domain(seed) if is_combined else {},
        curriculum_stage=2,
        early_gate_blend=0.75,
        stance_hip_roll_lift_offset_rad=amplitude,
    )
    observation, _ = env.reset(seed=seed)
    terminated = truncated = False
    for count in range(1, env.config.max_episode_steps + 1):
        action = _CORRECTOR_POLICY.predict(observation, deterministic=True)[0]
        observation, _, terminated, truncated, info = env.step(action)
        if terminated or truncated:
            break
    steps = step_diagnostics(info)
    failed_criteria = []
    if terminated:
        failed_criteria.append("fall")
    if not bool(info["impact_limit_satisfied"]):
        failed_criteria.append("impact")
    if float(info["base_forward_displacement_m"]) < 0.200:
        failed_criteria.append("forward")
    if min(float(step["advance_under_5n_fraction"]) for step in steps) < 0.9:
        failed_criteria.append("swing_unload")
    if min(float(step["both_contact_fraction"]) for step in steps) < 0.9:
        failed_criteria.append("double_support")
    if min(float(step["step_length_m"]) for step in steps) < 0.020:
        failed_criteria.append("step_length")
    row: dict[str, object] = {
        "amplitude_rad": amplitude,
        "first_side": side,
        "seed": seed,
        "case": "combined" if is_combined else "nominal",
        "terminated": terminated,
        "truncated": truncated,
        "success": bool(info["is_success"]),
        "failed_criteria": failed_criteria,
        "base_forward_displacement_m": float(info["base_forward_displacement_m"]),
        "maximum_landing_force_n": float(info["maximum_landing_force_n"]),
        "minimum_step_length_m": min(float(step["step_length_m"]) for step in steps),
        "minimum_advance_under_5n_fraction": min(
            float(step["advance_under_5n_fraction"]) for step in steps
        ),
        "minimum_both_contact_fraction": min(
            float(step["both_contact_fraction"]) for step in steps
        ),
        "first_step": steps[0],
        "maximum_applied_probe_rad": float(
            info["applied_stance_hip_roll_max_abs_rad"]
        ),
    }
    env.close()
    return row


def aggregate(amplitude: float, rows: list[dict[str, object]]) -> dict[str, object]:
    nominal = [row for row in rows if row["case"] == "nominal"]
    combined = [row for row in rows if row["case"] == "combined"]
    failures = Counter(
        reason for row in combined for reason in row["failed_criteria"]
    )
    combined_seeds = sorted({int(row["seed"]) for row in combined})
    paired = sum(
        all(row["success"] for row in combined if row["seed"] == seed)
        for seed in combined_seeds
    )
    key = (
        sum(row["success"] for row in nominal),
        sum(row["success"] for row in combined),
        paired,
        -max(float(row["maximum_landing_force_n"]) for row in combined),
        min(float(row["base_forward_displacement_m"]) for row in rows),
    )
    return {
        "amplitude_rad": amplitude,
        "nominal_successes": key[0],
        "combined_successes": key[1],
        "combined_runs": len(combined),
        "both_sides_successful_samples": paired,
        "falls": failures["fall"],
        "failure_counts": dict(sorted(failures.items())),
        "worst_landing_force_n": -key[3],
        "minimum_forward_displacement_m": key[4],
        "selection_key": list(key),
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--workers", type=int, default=4)
    parser.add_argument("--output", type=Path, default=OUTPUT)
    parser.add_argument("--amplitudes", type=float, nargs="+", default=AMPLITUDES)
    parser.add_argument("--label", default="stance_probe_calibration")
    parser.add_argument("--seed-start", type=int, default=101)
    parser.add_argument("--seed-end", type=int, default=120)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    amplitudes = tuple(args.amplitudes)
    if args.seed_end < args.seed_start:
        parser.error("seed-end must be greater than or equal to seed-start")
    combined_seeds = range(args.seed_start, args.seed_end + 1)
    cases_per_configuration = 2 + 2 * len(combined_seeds)
    specs = []
    for amplitude in amplitudes:
        specs.extend((amplitude, side, 17, False) for side in ("left", "right"))
        specs.extend(
            (amplitude, side, seed, True)
            for seed in combined_seeds
            for side in ("left", "right")
        )

    rows = []
    context = mp.get_context("spawn")
    with concurrent.futures.ProcessPoolExecutor(
        max_workers=args.workers,
        mp_context=context,
        initializer=initialize_worker,
        initargs=(str(BASE), str(CORRECTOR)),
    ) as executor:
        for completed, row in enumerate(executor.map(run_case, specs), start=1):
            rows.append(row)
            if completed % cases_per_configuration == 0:
                print(
                    f"completed configurations={completed // cases_per_configuration}/{len(amplitudes)}",
                    flush=True,
                )

    grouped: dict[float, list[dict[str, object]]] = defaultdict(list)
    for row in rows:
        grouped[float(row["amplitude_rad"])].append(row)
    scores = [aggregate(amplitude, grouped[amplitude]) for amplitude in amplitudes]
    selected = max(scores, key=lambda row: tuple(row["selection_key"]))
    summary = {
        "controller": "V20 frozen corrector + deterministic first-step stance hip-roll lift offset",
        "amplitudes_rad": list(amplitudes),
        "combined_seed_range": [args.seed_start, args.seed_end],
        "scores": scores,
        "selected": selected,
    }
    (args.output / f"{args.label}_results.json").write_text(
        json.dumps(rows, indent=2, allow_nan=False), encoding="utf-8"
    )
    (args.output / f"{args.label}_summary.json").write_text(
        json.dumps(summary, indent=2, allow_nan=False), encoding="utf-8"
    )
    print(json.dumps(summary, indent=2, allow_nan=False))


if __name__ == "__main__":
    main()
