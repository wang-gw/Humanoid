#!/usr/bin/env python3
"""Continue V19 training with a high-friction-biased hard-domain curriculum."""

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

from humanoidv2 import LandingResidualV19Env  # noqa: E402
from scripts.train_landing_residual_v19 import Progress, combined_domain, run_episode  # noqa: E402


BASE = ROOT / "results/khr3hv_v17_forward_margin/ppo_forward_margin_v17_final.zip"
SOURCE = ROOT / "results/khr3hv_v19_landing_residual_joint/ppo_landing_residual_v19_stage2.zip"
OUTPUT = ROOT / "results/khr3hv_v19_landing_residual_joint"


def evaluate(path: Path, base_path: Path) -> list[dict[str, object]]:
    corrector = PPO.load(path, device="cpu")
    base = PPO.load(base_path, device="cpu")
    rows: list[dict[str, object]] = []
    for side in ("left", "right"):
        row = run_episode(corrector, base, side, 17, {})
        row.update({"model": path.name, "case": "nominal"})
        rows.append(row)
    for sample, seed in enumerate(range(101, 121), start=1):
        domain = combined_domain(seed)
        for side in ("left", "right"):
            row = run_episode(corrector, base, side, seed, domain)
            row.update({"model": path.name, "case": "combined", "sample": sample})
            rows.append(row)
    return rows


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--base-policy", type=Path, default=BASE)
    parser.add_argument("--source", type=Path, default=SOURCE)
    parser.add_argument("--output", type=Path, default=OUTPUT)
    parser.add_argument("--n-envs", type=int, default=4)
    parser.add_argument("--seed", type=int, default=29)
    parser.add_argument("--chunks", type=int, default=4)
    parser.add_argument("--chunk-timesteps", type=int, default=40_000)
    parser.add_argument("--learning-rate", type=float, default=1.0e-4)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)

    def build_env():
        base = PPO.load(args.base_policy, device="cpu")
        return Monitor(
            LandingResidualV19Env(
                base_policy=base,
                curriculum_stage=3,
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
    for chunk in range(1, args.chunks + 1):
        model.learn(
            args.chunk_timesteps,
            callback=progress,
            progress_bar=False,
            reset_num_timesteps=True,
        )
        path = args.output / f"ppo_landing_residual_v19_hard_{chunk * args.chunk_timesteps}_steps.zip"
        model.save(path)
        candidates.append(path)
        recent = progress.episodes[-20:]
        record = {
            "chunk": chunk,
            "added_timesteps": chunk * args.chunk_timesteps,
            "recent_reward": float(np.mean([row["r"] for row in recent])) if recent else None,
            "recent_length": float(np.mean([row["l"] for row in recent])) if recent else None,
            "model": path.name,
        }
        records.append(record)
        print(json.dumps(record, allow_nan=False), flush=True)

    comparison: list[dict[str, object]] = []
    for path in candidates:
        print(f"evaluating {path.name}", flush=True)
        comparison.extend(evaluate(path, args.base_policy))
    grouped = {
        path: [row for row in comparison if row["model"] == path.name] for path in candidates
    }

    def key(path: Path) -> tuple[float, ...]:
        rows = grouped[path]
        nominal = [row for row in rows if row["case"] == "nominal"]
        combined = [row for row in rows if row["case"] == "combined"]
        paired = sum(
            all(row["success"] for row in combined if row["sample"] == sample)
            for sample in range(1, 21)
        )
        return (
            float(sum(row["success"] for row in nominal)),
            float(sum(row["success"] for row in combined)),
            float(paired),
            -max(float(row["maximum_landing_force_n"]) for row in combined),
            min(float(row["base_forward_displacement_m"]) for row in rows),
        )

    selected = max(candidates, key=key)
    PPO.load(selected, device="cpu").save(args.output / "ppo_landing_residual_v19_final")
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
                "combined_runs": len(combined),
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
                "selection_key": list(key(path)),
            }
        )
    summary = {
        "source": str(args.source),
        "hard_domain": "75% friction [1.05, 1.20], 25% friction [0.80, 1.05]",
        "learning_rate": args.learning_rate,
        "elapsed_seconds": time.time() - started,
        "records": records,
        "candidate_scores": scores,
        "selected_model": selected.name,
    }
    (args.output / "hard_training_comparison.json").write_text(
        json.dumps(comparison, indent=2, allow_nan=False), encoding="utf-8"
    )
    (args.output / "hard_training_summary.json").write_text(
        json.dumps(summary, indent=2, allow_nan=False), encoding="utf-8"
    )
    env.close()
    print(json.dumps(summary, indent=2, allow_nan=False), flush=True)


if __name__ == "__main__":
    main()
