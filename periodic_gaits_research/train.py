#!/usr/bin/env python3
"""Train the first reference-free periodic walking policy."""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

import numpy as np
import torch
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import BaseCallback
from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3.common.monitor import Monitor

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(Path(__file__).resolve().parent))

from periodic_gaits import PeriodicGaitEnv  # noqa: E402


class Progress(BaseCallback):
    def __init__(self, interval: int = 25_000) -> None:
        super().__init__()
        self.interval = interval
        self.next_report = interval
        self.episodes: list[dict[str, float]] = []

    def _on_step(self) -> bool:
        for info in self.locals.get("infos", []):
            if "episode" in info:
                self.episodes.append(info["episode"])
        if self.num_timesteps >= self.next_report:
            recent = self.episodes[-20:]
            reward = np.mean([x["r"] for x in recent]) if recent else float("nan")
            length = np.mean([x["l"] for x in recent]) if recent else float("nan")
            print(f"steps={self.num_timesteps} reward={reward:.2f} length={length:.1f}", flush=True)
            self.next_report += self.interval
        return True


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--timesteps", type=int, default=200_000)
    parser.add_argument("--n-envs", type=int, default=4)
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--output", type=Path, default=REPO_ROOT / "outputs/periodic_gaits_research/v1")
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)

    env = make_vec_env(lambda: Monitor(PeriodicGaitEnv()), n_envs=args.n_envs, seed=args.seed)
    model = PPO(
        "MlpPolicy",
        env,
        learning_rate=3e-4,
        n_steps=512,
        batch_size=256,
        n_epochs=5,
        gamma=0.99,
        gae_lambda=0.95,
        clip_range=0.2,
        ent_coef=0.005,
        policy_kwargs={"net_arch": {"pi": [256, 256], "vf": [256, 256]}, "activation_fn": torch.nn.ELU},
        seed=args.seed,
        device="cpu",
        verbose=0,
    )
    callback = Progress()
    started = time.time()
    model.learn(total_timesteps=args.timesteps, callback=callback)
    model.save(args.output / "ppo_periodic_walk_v1")
    summary = {
        "timesteps": model.num_timesteps,
        "n_envs": args.n_envs,
        "seed": args.seed,
        "elapsed_seconds": time.time() - started,
        "episodes": len(callback.episodes),
    }
    (args.output / "training_summary.json").write_text(json.dumps(summary, indent=2))
    env.close()
    print(json.dumps(summary, indent=2), flush=True)


if __name__ == "__main__":
    main()
