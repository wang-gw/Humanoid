#!/usr/bin/env python3
"""Train the symmetric left/right one-step V5 task."""

from __future__ import annotations

import argparse
import json
import re
import sys
import time
from pathlib import Path

import numpy as np
import torch
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import BaseCallback, CheckpointCallback
from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3.common.monitor import Monitor

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from humanoidv2 import SingleStepV5Env  # noqa: E402


class Progress(BaseCallback):
    def __init__(self) -> None:
        super().__init__()
        self.next_report = 25_000
        self.episodes: list[dict[str, float]] = []

    def _on_step(self) -> bool:
        for info in self.locals.get("infos", []):
            if "episode" in info:
                self.episodes.append({key: float(value) for key, value in info["episode"].items()})
        if self.num_timesteps >= self.next_report:
            recent = self.episodes[-20:]
            reward = np.mean([row["r"] for row in recent]) if recent else float("nan")
            length = np.mean([row["l"] for row in recent]) if recent else float("nan")
            print(f"steps={self.num_timesteps} reward={reward:.3f} length={length:.1f}", flush=True)
            self.next_report += 25_000
        return True


def evaluate_candidate(path: Path) -> list[dict[str, object]]:
    policy = PPO.load(path, device="cpu")
    results = []
    for side in ("left", "right"):
        env = SingleStepV5Env(fixed_side=side)
        observation, _ = env.reset(seed=7)
        total_reward = 0.0
        records = []
        for step in range(env.config.max_episode_steps):
            action = policy.predict(observation, deterministic=True)[0]
            observation, reward, terminated, truncated, info = env.step(action)
            total_reward += reward
            records.append(info)
            if terminated or truncated:
                break
        advance = [row for row in records if row["task_phase"] == "advance"]
        results.append(
            {
                "model": path.name,
                "side": side,
                "steps": step + 1,
                "reward": total_reward,
                "terminated": terminated,
                "success": bool(info["is_success"]),
                "advance_mean_swing_force_n": float(np.mean([row["swing_force_n"] for row in advance])),
                "final_both_contact_fraction": float(info["final_both_contact_fraction"]),
                "final_step_length_m": float(info["step_length_m"]),
                "peak_landing_force_n": float(info["peak_landing_force_n"]),
            }
        )
        env.close()
    return results


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--timesteps", type=int, default=50_000)
    parser.add_argument("--n-envs", type=int, default=4)
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--output", type=Path, default=ROOT / "results/khr3hv_v5_symmetric")
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    (args.output / "checkpoints").mkdir(exist_ok=True)

    env = make_vec_env(lambda: Monitor(SingleStepV5Env()), n_envs=args.n_envs, seed=args.seed)
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
        policy_kwargs={"net_arch": {"pi": [64, 64], "vf": [64, 64]}, "activation_fn": torch.nn.Tanh},
        verbose=0,
        seed=args.seed,
        device="cpu",
    )
    progress = Progress()
    checkpoint = CheckpointCallback(
        save_freq=max(25_000 // args.n_envs, 1),
        save_path=str(args.output / "checkpoints"),
        name_prefix="single_step",
    )
    started = time.time()
    model.learn(args.timesteps, callback=[progress, checkpoint], progress_bar=False)
    last_path = args.output / "ppo_single_step_last.zip"
    model.save(last_path)
    candidate_paths = sorted((args.output / "checkpoints").glob("*.zip")) + [last_path]
    comparison = [row for path in candidate_paths for row in evaluate_candidate(path)]
    grouped = {path: [row for row in comparison if row["model"] == path.name] for path in candidate_paths}
    selected_path = max(
        candidate_paths,
        key=lambda path: (
            sum(row["success"] for row in grouped[path]),
            min(row["reward"] for row in grouped[path]),
        ),
    )
    PPO.load(selected_path, device="cpu").save(args.output / "ppo_single_step_final")
    (args.output / "checkpoint_comparison.json").write_text(
        json.dumps(comparison, indent=2), encoding="utf-8"
    )
    step_match = re.search(r"_(\d+)_steps", selected_path.stem)
    summary = {
        "actual_timesteps": model.num_timesteps,
        "elapsed_seconds": time.time() - started,
        "episodes": len(progress.episodes),
        "last_20_reward": float(np.mean([row["r"] for row in progress.episodes[-20:]])),
        "last_20_length": float(np.mean([row["l"] for row in progress.episodes[-20:]])),
        "selected_model": selected_path.name,
        "selected_training_steps": int(step_match.group(1)) if step_match else model.num_timesteps,
        "selection_rule": "both-side successes, then maximize the lower side reward",
    }
    (args.output / "training_summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    env.close()
    print(json.dumps(summary, indent=2), flush=True)


if __name__ == "__main__":
    main()
