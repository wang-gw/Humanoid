#!/usr/bin/env python3
"""Train the first KHR-3HV-style PPO baseline."""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

import numpy as np
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import BaseCallback, CheckpointCallback
from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3.common.monitor import Monitor

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from humanoidv2 import KHR3HVEnv, KHR3HVV2Env, KHR3HVV21Env, KHR3HVV22Env  # noqa: E402


class ProgressCallback(BaseCallback):
    def __init__(self, interval: int = 25_000) -> None:
        super().__init__()
        self.interval = interval
        self.next_report = interval
        self.episodes: list[dict[str, float]] = []

    def _on_step(self) -> bool:
        for info in self.locals.get("infos", []):
            episode = info.get("episode")
            if episode:
                self.episodes.append({k: float(v) for k, v in episode.items()})
        if self.num_timesteps >= self.next_report:
            recent = self.episodes[-20:]
            mean_reward = float(np.mean([item["r"] for item in recent])) if recent else float("nan")
            mean_length = float(np.mean([item["l"] for item in recent])) if recent else float("nan")
            print(
                f"steps={self.num_timesteps} episodes={len(self.episodes)} "
                f"recent_reward={mean_reward:.3f} recent_length={mean_length:.1f}",
                flush=True,
            )
            self.next_report += self.interval
        return True


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--timesteps", type=int, default=500_000)
    parser.add_argument("--n-envs", type=int, default=4)
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--version", choices=("v1", "v2", "v2_1", "v2_2"), default="v1")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    if args.output is None:
        args.output = ROOT / f"results/khr3hv_{args.version}"
    args.output.mkdir(parents=True, exist_ok=True)
    (args.output / "checkpoints").mkdir(exist_ok=True)

    def factory() -> Monitor:
        env_class = {
            "v1": KHR3HVEnv,
            "v2": KHR3HVV2Env,
            "v2_1": KHR3HVV21Env,
            "v2_2": KHR3HVV22Env,
        }[args.version]
        return Monitor(env_class())

    env = make_vec_env(factory, n_envs=args.n_envs, seed=args.seed)
    model = PPO(
        "MlpPolicy",
        env,
        learning_rate=3.0e-4,
        n_steps=2048,
        batch_size=64,
        n_epochs=10,
        gamma=0.99,
        gae_lambda=0.95,
        clip_range=0.2,
        ent_coef=0.0,
        vf_coef=0.5,
        policy_kwargs={"net_arch": {"pi": [64, 64], "vf": [64, 64]}, "activation_fn": __import__("torch").nn.Tanh},
        verbose=0,
        seed=args.seed,
        device="cpu",
    )
    progress = ProgressCallback()
    checkpoint = CheckpointCallback(
        save_freq=max(50_000 // args.n_envs, 1),
        save_path=str(args.output / "checkpoints"),
        name_prefix="ppo_khr3hv",
    )
    started = time.time()
    model.learn(total_timesteps=args.timesteps, callback=[progress, checkpoint], progress_bar=False)
    elapsed = time.time() - started
    model.save(args.output / "ppo_khr3hv_final")
    metadata = {
        "requested_timesteps": args.timesteps,
        "actual_timesteps": model.num_timesteps,
        "n_envs": args.n_envs,
        "seed": args.seed,
        "version": args.version,
        "elapsed_seconds": elapsed,
        "episodes": len(progress.episodes),
        "last_20_mean_reward": float(np.mean([x["r"] for x in progress.episodes[-20:]])) if progress.episodes else None,
        "last_20_mean_length": float(np.mean([x["l"] for x in progress.episodes[-20:]])) if progress.episodes else None,
    }
    (args.output / "training_summary.json").write_text(json.dumps(metadata, indent=2), encoding="utf-8")
    env.close()
    print(json.dumps(metadata, indent=2), flush=True)


if __name__ == "__main__":
    main()
