#!/usr/bin/env python3
"""Transfer V14 PPO into the 3.40-second V15 speed-curriculum stage."""

from __future__ import annotations

import argparse
import json
import re
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

from humanoidv2 import DynamicForwardV15Env  # noqa: E402


class Progress(BaseCallback):
    def __init__(self) -> None:
        super().__init__()
        self.next_report = 25_000
        self.episodes: list[dict[str, float]] = []

    def _on_step(self) -> bool:
        for info in self.locals.get("infos", []):
            if "episode" in info:
                self.episodes.append(
                    {key: float(value) for key, value in info["episode"].items()}
                )
        if self.num_timesteps >= self.next_report:
            recent = self.episodes[-20:]
            reward = np.mean([row["r"] for row in recent]) if recent else float("nan")
            length = np.mean([row["l"] for row in recent]) if recent else float("nan")
            print(
                f"steps={self.num_timesteps} reward={reward:.3f} length={length:.1f}",
                flush=True,
            )
            self.next_report += 25_000
        return True


def evaluate_candidate(path: Path, seed: int) -> list[dict[str, object]]:
    policy = PPO.load(path, device="cpu")
    rows: list[dict[str, object]] = []
    for first_side in ("left", "right"):
        env = DynamicForwardV15Env(fixed_first_side=first_side)
        observation, _ = env.reset(seed=seed)
        start_world_y = float(env.data.qpos[1])
        total_reward = 0.0
        for step in range(env.config.max_episode_steps):
            action = policy.predict(observation, deterministic=True)[0]
            observation, reward, terminated, truncated, info = env.step(action)
            total_reward += reward
            if terminated or truncated:
                break
        rows.append(
            {
                "model": path.name,
                "first_side": first_side,
                "steps": step + 1,
                "duration_seconds": (step + 1) * env.config.control_dt,
                "reward": total_reward,
                "terminated": terminated,
                "truncated": truncated,
                "gait_success": bool(info["gait_success"]),
                "impact_limit_satisfied": bool(info["impact_limit_satisfied"]),
                "success": bool(info["is_success"]),
                "forward_axis": info["forward_axis"],
                "world_y_displacement_m": float(env.data.qpos[1] - start_world_y),
                "base_forward_displacement_m": float(info["base_forward_displacement_m"]),
                "maximum_landing_force_n": float(info["maximum_landing_force_n"]),
                "minimum_step_length_m": min(
                    float(info[f"step_{index}_length_m"]) for index in range(1, 9)
                ),
                "minimum_advance_under_5n_fraction": min(
                    float(info[f"step_{index}_advance_under_5n_fraction"])
                    for index in range(1, 9)
                ),
            }
        )
        env.close()
    return rows


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--timesteps", type=int, default=50_000)
    parser.add_argument("--n-envs", type=int, default=4)
    parser.add_argument("--seed", type=int, default=17)
    parser.add_argument("--learning-rate", type=float, default=1.0e-5)
    parser.add_argument(
        "--initial-model",
        type=Path,
        default=ROOT
        / "results/khr3hv_v14_dynamic_forward_stage1/ppo_dynamic_forward_v14_final.zip",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=ROOT / "results/khr3hv_v15_dynamic_forward_stage2",
    )
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    (args.output / "checkpoints").mkdir(exist_ok=True)

    env = make_vec_env(
        lambda: Monitor(DynamicForwardV15Env()), n_envs=args.n_envs, seed=args.seed
    )
    initial_path = args.output / "ppo_dynamic_forward_v15_initial.zip"
    PPO.load(args.initial_model, device="cpu").save(initial_path)
    model = PPO.load(
        args.initial_model,
        env=env,
        device="cpu",
        learning_rate=args.learning_rate,
    )
    progress = Progress()
    checkpoint = CheckpointCallback(
        save_freq=max(25_000 // args.n_envs, 1),
        save_path=str(args.output / "checkpoints"),
        name_prefix="dynamic_forward_v15",
    )
    started = time.time()
    model.learn(
        args.timesteps,
        callback=[progress, checkpoint],
        progress_bar=False,
        reset_num_timesteps=True,
    )
    last_path = args.output / "ppo_dynamic_forward_v15_last.zip"
    model.save(last_path)

    candidate_paths = (
        [initial_path]
        + sorted((args.output / "checkpoints").glob("*.zip"))
        + [last_path]
    )
    comparison = [
        row for path in candidate_paths for row in evaluate_candidate(path, args.seed)
    ]
    grouped = {
        path: [row for row in comparison if row["model"] == path.name]
        for path in candidate_paths
    }
    selected_path = max(
        candidate_paths,
        key=lambda path: (
            sum(bool(row["success"]) for row in grouped[path]),
            sum(bool(row["gait_success"]) for row in grouped[path]),
            sum(int(row["steps"]) for row in grouped[path]),
            -max(float(row["maximum_landing_force_n"]) for row in grouped[path]),
            min(float(row["reward"]) for row in grouped[path]),
        ),
    )
    PPO.load(selected_path, device="cpu").save(
        args.output / "ppo_dynamic_forward_v15_final"
    )
    (args.output / "checkpoint_comparison.json").write_text(
        json.dumps(comparison, indent=2, allow_nan=False), encoding="utf-8"
    )
    step_match = re.search(r"_(\d+)_steps", selected_path.stem)
    recent = progress.episodes[-20:]
    summary = {
        "source_model": str(args.initial_model),
        "speed_curriculum_stage": 2,
        "step_cycle_steps": 85,
        "step_cycle_seconds": 3.40,
        "actual_transfer_timesteps": model.num_timesteps,
        "elapsed_seconds": time.time() - started,
        "episodes": len(progress.episodes),
        "last_20_reward": float(np.mean([row["r"] for row in recent])),
        "last_20_length": float(np.mean([row["l"] for row in recent])),
        "selected_model": selected_path.name,
        "selected_transfer_steps": (
            int(step_match.group(1))
            if step_match
            else (0 if selected_path == initial_path else model.num_timesteps)
        ),
        "selection_rule": (
            "both-order strict success, gait success, survival length, worst impact, "
            "then worst reward"
        ),
    }
    (args.output / "training_summary.json").write_text(
        json.dumps(summary, indent=2, allow_nan=False), encoding="utf-8"
    )
    env.close()
    print(json.dumps(summary, indent=2, allow_nan=False), flush=True)


if __name__ == "__main__":
    main()
