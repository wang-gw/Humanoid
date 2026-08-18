#!/usr/bin/env python3
"""Stabilize V20 at the calibrated 75% early gate over the full uniform domain."""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

import numpy as np
from stable_baselines3 import PPO
from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3.common.monitor import Monitor

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from humanoidv2 import ContactAwareResidualV20Env  # noqa: E402
from scripts.train_contact_aware_residual_v20 import run_episode  # noqa: E402
from scripts.train_landing_residual_v19 import Progress, combined_domain  # noqa: E402


BASE = ROOT / "results/khr3hv_v17_forward_margin/ppo_forward_margin_v17_final.zip"
SOURCE = ROOT / "results/khr3hv_v20_contact_aware/ppo_contact_aware_v20_stage2.zip"
OUTPUT = ROOT / "results/khr3hv_v20_contact_aware"
DEPLOYMENT_BLEND = 0.75


def evaluate(path: Path, base_path: Path) -> list[dict[str, object]]:
    corrector = PPO.load(path, device="cpu")
    base = PPO.load(base_path, device="cpu")
    rows: list[dict[str, object]] = []
    for side in ("left", "right"):
        row = run_episode_with_blend(corrector, base, side, 17, {})
        row.update({"model": path.name, "case": "nominal"})
        rows.append(row)
    for sample, seed in enumerate(range(101, 121), start=1):
        for side in ("left", "right"):
            row = run_episode_with_blend(
                corrector, base, side, seed, combined_domain(seed)
            )
            row.update({"model": path.name, "case": "combined", "sample": sample})
            rows.append(row)
    return rows


def run_episode_with_blend(corrector, base, side, seed, domain):
    env = ContactAwareResidualV20Env(
        base_policy=base,
        fixed_first_side=side,
        fixed_domain=domain,
        curriculum_stage=2,
        early_gate_blend=DEPLOYMENT_BLEND,
    )
    observation, _ = env.reset(seed=seed)
    maximum_correction = 0.0
    for step in range(env.config.max_episode_steps):
        action = corrector.predict(observation, deterministic=True)[0]
        observation, _, terminated, truncated, info = env.step(action)
        maximum_correction = max(
            maximum_correction, float(info["applied_correction_max_abs"])
        )
        if terminated or truncated:
            break
    row = {
        "first_side": side,
        "seed": seed,
        **domain,
        "steps": step + 1,
        "terminated": terminated,
        "truncated": truncated,
        "success": bool(info["is_success"]),
        "impact_limit_satisfied": bool(info["impact_limit_satisfied"]),
        "maximum_landing_force_n": float(info["maximum_landing_force_n"]),
        "base_forward_displacement_m": float(info["base_forward_displacement_m"]),
        "minimum_advance_under_5n_fraction": min(
            float(info[f"step_{index}_advance_under_5n_fraction"])
            for index in range(1, 9)
        ),
        "minimum_both_contact_fraction": min(
            float(info[f"step_{index}_both_contact_fraction"])
            for index in range(1, 9)
        ),
        "maximum_applied_correction_rad": maximum_correction,
    }
    env.close()
    return row


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--base-policy", type=Path, default=BASE)
    parser.add_argument("--source", type=Path, default=SOURCE)
    parser.add_argument("--output", type=Path, default=OUTPUT)
    parser.add_argument("--learning-rate", type=float, default=5.0e-5)
    parser.add_argument("--n-envs", type=int, default=4)
    parser.add_argument("--seed", type=int, default=47)
    parser.add_argument("--chunk-offset", type=int, default=0)
    parser.add_argument("--chunks", type=int, default=2)
    args = parser.parse_args()

    def build_env():
        base = PPO.load(args.base_policy, device="cpu")
        return Monitor(
            ContactAwareResidualV20Env(
                base_policy=base,
                curriculum_stage=2,
                early_gate_blend=DEPLOYMENT_BLEND,
                landing_force_reward_scale=1.25,
                downward_velocity_reward_scale=0.20,
                correction_penalty_scale=0.002,
                correction_rate_penalty_scale=0.002,
                terminal_success_bonus=50.0,
            )
        )

    env = make_vec_env(build_env, n_envs=args.n_envs, seed=args.seed)
    model = PPO.load(args.source, env=env, device="cpu")
    model.learning_rate = args.learning_rate
    model.lr_schedule = lambda _: args.learning_rate
    progress = Progress()
    candidates = [args.source]
    records = []
    started = time.time()
    for index in range(1, args.chunks + 1):
        chunk = args.chunk_offset + index * 40_000
        model.learn(
            40_000, callback=progress, progress_bar=False, reset_num_timesteps=True
        )
        path = args.output / f"ppo_contact_aware_v20_stable_{chunk}_steps.zip"
        model.save(path)
        candidates.append(path)
        recent = progress.episodes[-20:]
        record = {
            "added_timesteps": chunk,
            "recent_reward": float(np.mean([row["r"] for row in recent])),
            "recent_length": float(np.mean([row["l"] for row in recent])),
            "model": path.name,
        }
        records.append(record)
        print(json.dumps(record, allow_nan=False), flush=True)

    comparison = []
    for path in candidates:
        print(f"evaluating {path.name}", flush=True)
        comparison.extend(evaluate(path, args.base_policy))
    grouped = {
        path: [row for row in comparison if row["model"] == path.name]
        for path in candidates
    }

    def key(path):
        rows = grouped[path]
        nominal = [row for row in rows if row["case"] == "nominal"]
        combined = [row for row in rows if row["case"] == "combined"]
        paired = sum(
            all(row["success"] for row in combined if row["sample"] == sample)
            for sample in range(1, 21)
        )
        return (
            sum(row["success"] for row in nominal),
            sum(row["success"] for row in combined),
            paired,
            -max(row["maximum_landing_force_n"] for row in combined),
            min(row["base_forward_displacement_m"] for row in rows),
        )

    selected = max(candidates, key=key)
    PPO.load(selected, device="cpu").save(args.output / "ppo_contact_aware_v20_final")
    scores = []
    for path in candidates:
        rows = grouped[path]
        nominal = [row for row in rows if row["case"] == "nominal"]
        combined = [row for row in rows if row["case"] == "combined"]
        scores.append(
            {
                "model": path.name,
                "nominal_successes": sum(row["success"] for row in nominal),
                "combined_successes": sum(row["success"] for row in combined),
                "both_sides_successful_samples": sum(
                    all(row["success"] for row in combined if row["sample"] == sample)
                    for sample in range(1, 21)
                ),
                "worst_landing_force_n": max(
                    row["maximum_landing_force_n"] for row in combined
                ),
                "minimum_forward_displacement_m": min(
                    row["base_forward_displacement_m"] for row in rows
                ),
                "selection_key": list(key(path)),
            }
        )
    summary = {
        "source": str(args.source),
        "early_gate_blend": DEPLOYMENT_BLEND,
        "domain": "uniform V18 full range",
        "learning_rate": args.learning_rate,
        "elapsed_seconds": time.time() - started,
        "records": records,
        "candidate_scores": scores,
        "selected_model": selected.name,
    }
    suffix = (
        "" if args.chunk_offset == 0
        else f"_{args.chunk_offset}_{args.chunk_offset + args.chunks * 40_000}"
    )
    (args.output / f"stabilization_comparison{suffix}.json").write_text(
        json.dumps(comparison, indent=2, allow_nan=False), encoding="utf-8"
    )
    (args.output / f"stabilization_summary{suffix}.json").write_text(
        json.dumps(summary, indent=2, allow_nan=False), encoding="utf-8"
    )
    (args.output / "deployment_config.json").write_text(
        json.dumps({"early_gate_blend": DEPLOYMENT_BLEND}, indent=2), encoding="utf-8"
    )
    env.close()
    print(json.dumps(summary, indent=2, allow_nan=False), flush=True)


if __name__ == "__main__":
    main()
