#!/usr/bin/env python3
"""Sweep one small first-step probe over every remaining V20 failure."""

from __future__ import annotations

import argparse
import concurrent.futures
import json
import multiprocessing as mp
import sys
from collections import defaultdict
from pathlib import Path

import torch
from stable_baselines3 import PPO

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from humanoidv2 import CounterfactualProbeV22Env  # noqa: E402
from humanoidv2.khr3hv_env import JOINT_NAMES  # noqa: E402
from scripts.diagnose_counterfactual_v22 import (  # noqa: E402
    BASE,
    CORRECTOR,
    OUTPUT,
    step_diagnostics,
)
from scripts.train_landing_residual_v19 import combined_domain  # noqa: E402


PHASES = ("lift", "hold", "advance", "land")
OFFSETS = (-0.006, 0.006)
_BASE_POLICY: PPO | None = None
_CORRECTOR_POLICY: PPO | None = None


def initialize_worker(base_path: str, corrector_path: str) -> None:
    global _BASE_POLICY, _CORRECTOR_POLICY
    torch.set_num_threads(1)
    _BASE_POLICY = PPO.load(base_path, device="cpu")
    _CORRECTOR_POLICY = PPO.load(corrector_path, device="cpu")


def violation(row: dict[str, object]) -> float:
    value = 0.0
    for step in row["steps"]:
        value += max(float(step["peak_landing_force_n"]) - 50.0, 0.0) / 30.0
        value += max(0.9 - float(step["advance_under_5n_fraction"]), 0.0)
        value += max(0.9 - float(step["both_contact_fraction"]), 0.0)
        value += max(0.020 - float(step["step_length_m"]), 0.0) / 0.020
    value += max(0.200 - float(row["base_forward_displacement_m"]), 0.0) / 0.050
    if bool(row["terminated"]):
        value += 10.0
    return float(value)


def failure_count(row: dict[str, object]) -> int:
    count = sum(len(step["failures"]) for step in row["steps"])
    count += int(float(row["base_forward_displacement_m"]) < 0.200)
    count += int(bool(row["terminated"]))
    return count


def run_candidate(spec: tuple[int, str, str, int, str, float]) -> dict[str, object]:
    index, side, seed, joint_index, phase, offset = spec
    assert _BASE_POLICY is not None and _CORRECTOR_POLICY is not None
    env = CounterfactualProbeV22Env(
        base_policy=_BASE_POLICY,
        fixed_first_side=side,
        fixed_domain=combined_domain(seed),
        curriculum_stage=2,
        early_gate_blend=0.75,
        probe_step=1,
        probe_phase=phase,
        probe_joint=joint_index,
        probe_offset_rad=offset,
    )
    observation, _ = env.reset(seed=seed)
    terminated = truncated = False
    for count in range(1, env.config.max_episode_steps + 1):
        action = _CORRECTOR_POLICY.predict(observation, deterministic=True)[0]
        observation, _, terminated, truncated, info = env.step(action)
        if terminated or truncated:
            break
    steps = step_diagnostics(info)
    row: dict[str, object] = {
        "baseline_index": index,
        "seed": seed,
        "first_side": side,
        "probe_step": 1,
        "probe_phase": phase,
        "probe_joint": JOINT_NAMES[joint_index],
        "probe_joint_index": joint_index,
        "probe_offset_rad": offset,
        "applied_probe_max_abs_rad": float(info["applied_probe_max_abs_rad"]),
        "terminated": terminated,
        "truncated": truncated,
        "success": bool(info["is_success"]),
        "base_forward_displacement_m": float(info["base_forward_displacement_m"]),
        "maximum_landing_force_n": float(info["maximum_landing_force_n"]),
        "steps": steps,
    }
    row["failure_count"] = failure_count(row)
    row["violation"] = violation(row)
    row["failing_steps"] = [step["step"] for step in steps if step["failures"]]
    env.close()
    return row


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--workers", type=int, default=4)
    parser.add_argument(
        "--baseline",
        type=Path,
        default=OUTPUT / "baseline_failure_diagnostics.json",
    )
    parser.add_argument("--output", type=Path, default=OUTPUT)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    baseline = json.loads(args.baseline.read_text(encoding="utf-8"))["rows"]
    for row in baseline:
        row["failure_count"] = failure_count(row)
        row["violation"] = violation(row)

    specs = [
        (index, str(row["first_side"]), int(row["seed"]), joint, phase, offset)
        for index, row in enumerate(baseline)
        for phase in PHASES
        for joint in range(len(JOINT_NAMES))
        for offset in OFFSETS
    ]
    results: list[dict[str, object]] = []
    context = mp.get_context("spawn")
    with concurrent.futures.ProcessPoolExecutor(
        max_workers=args.workers,
        mp_context=context,
        initializer=initialize_worker,
        initargs=(str(BASE), str(CORRECTOR)),
    ) as executor:
        for completed, row in enumerate(executor.map(run_candidate, specs), start=1):
            baseline_row = baseline[int(row["baseline_index"])]
            row["violation_improvement"] = float(
                baseline_row["violation"] - row["violation"]
            )
            row["new_later_failure"] = any(
                step["failures"] for step in row["steps"][1:]
            )
            results.append(row)
            if completed % 50 == 0 or completed == len(specs):
                print(f"completed={completed}/{len(specs)}", flush=True)

    by_run: dict[int, list[dict[str, object]]] = defaultdict(list)
    by_probe: dict[tuple[str, str, float], list[dict[str, object]]] = defaultdict(list)
    for row in results:
        by_run[int(row["baseline_index"])].append(row)
        by_probe[
            (
                str(row["probe_phase"]),
                str(row["probe_joint"]),
                float(row["probe_offset_rad"]),
            )
        ].append(row)

    run_summaries = []
    for index, baseline_row in enumerate(baseline):
        candidates = sorted(
            by_run[index],
            key=lambda row: (
                not bool(row["success"]),
                bool(row["new_later_failure"]),
                int(row["failure_count"]),
                float(row["violation"]),
                -float(row["base_forward_displacement_m"]),
            ),
        )
        run_summaries.append(
            {
                "seed": baseline_row["seed"],
                "first_side": baseline_row["first_side"],
                "baseline_failure_count": baseline_row["failure_count"],
                "baseline_violation": baseline_row["violation"],
                "successful_probes": sum(row["success"] for row in candidates),
                "improving_without_later_failure": sum(
                    float(row["violation_improvement"]) > 0.0
                    and not row["new_later_failure"]
                    for row in candidates
                ),
                "top_candidates": candidates[:10],
            }
        )

    probe_summaries = []
    for (phase, joint, offset), rows in by_probe.items():
        probe_summaries.append(
            {
                "probe_phase": phase,
                "probe_joint": joint,
                "probe_offset_rad": offset,
                "recovered_failures": sum(row["success"] for row in rows),
                "runs_with_lower_violation": sum(
                    float(row["violation_improvement"]) > 0.0 for row in rows
                ),
                "new_later_failures": sum(row["new_later_failure"] for row in rows),
                "mean_violation_improvement": sum(
                    float(row["violation_improvement"]) for row in rows
                )
                / len(rows),
                "minimum_violation_improvement": min(
                    float(row["violation_improvement"]) for row in rows
                ),
            }
        )
    probe_summaries.sort(
        key=lambda row: (
            -int(row["recovered_failures"]),
            -int(row["runs_with_lower_violation"]),
            int(row["new_later_failures"]),
            -float(row["mean_violation_improvement"]),
        )
    )
    summary = {
        "probe_offset_rad": list(OFFSETS),
        "probe_phases": list(PHASES),
        "runs": len(baseline),
        "experiments": len(results),
        "total_successful_counterfactuals": sum(row["success"] for row in results),
        "runs_with_at_least_one_successful_probe": sum(
            item["successful_probes"] > 0 for item in run_summaries
        ),
        "run_summaries": run_summaries,
        "universal_probe_ranking": probe_summaries,
    }
    (args.output / "first_pass_results.json").write_text(
        json.dumps(results, indent=2, allow_nan=False), encoding="utf-8"
    )
    (args.output / "first_pass_summary.json").write_text(
        json.dumps(summary, indent=2, allow_nan=False), encoding="utf-8"
    )
    print(
        json.dumps(
            {
                "experiments": len(results),
                "total_successful_counterfactuals": summary[
                    "total_successful_counterfactuals"
                ],
                "runs_with_at_least_one_successful_probe": summary[
                    "runs_with_at_least_one_successful_probe"
                ],
                "top_universal_probes": probe_summaries[:10],
            },
            indent=2,
            allow_nan=False,
        )
    )


if __name__ == "__main__":
    main()
