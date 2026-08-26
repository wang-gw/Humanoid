#!/usr/bin/env python3
"""Curriculum V2: refine the successful V1 gait with natural swing costs."""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

from stable_baselines3 import PPO
from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3.common.monitor import Monitor

PROJECT_DIR = Path(__file__).resolve().parent
REPO_ROOT = PROJECT_DIR.parent
sys.path.insert(0, str(Path(__file__).resolve().parent))

from periodic_gaits import NaturalGaitConfig, PeriodicGaitEnv  # noqa: E402


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", type=Path, default=PROJECT_DIR / "results/v1_baseline/ppo_periodic_walk_v1.zip")
    parser.add_argument("--timesteps", type=int, default=500_000)
    parser.add_argument("--n-envs", type=int, default=4)
    parser.add_argument("--seed", type=int, default=31)
    parser.add_argument("--learning-rate", type=float, default=5e-5)
    parser.add_argument("--output", type=Path, default=REPO_ROOT / "outputs/periodic_gaits_research/v2_finetune")
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    env = make_vec_env(
        lambda: Monitor(PeriodicGaitEnv(config=NaturalGaitConfig())),
        n_envs=args.n_envs,
        seed=args.seed,
    )
    model = PPO.load(args.source, env=env, device="cpu")
    model.learning_rate = args.learning_rate
    model.lr_schedule = lambda _: args.learning_rate
    started = time.time()
    model.learn(total_timesteps=args.timesteps, reset_num_timesteps=False, progress_bar=False)
    model.save(args.output / "ppo_periodic_walk_v2")
    summary = {
        "version": "v2_natural_swing_curriculum",
        "source_policy": str(args.source.resolve()),
        "additional_timesteps": args.timesteps,
        "final_timesteps": model.num_timesteps,
        "learning_rate": args.learning_rate,
        "seed": args.seed,
        "elapsed_seconds": time.time() - started,
        "config": NaturalGaitConfig().__dict__,
    }
    (args.output / "training_summary.json").write_text(json.dumps(summary, indent=2))
    env.close()
    print(json.dumps(summary, indent=2), flush=True)


if __name__ == "__main__":
    main()
