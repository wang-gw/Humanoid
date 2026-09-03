#!/usr/bin/env python3
"""Train the first reference-free periodic walking policy."""

from __future__ import annotations

import argparse
import csv
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

from periodic_gaits import PeriodicGaitConfig, PeriodicGaitEnv  # noqa: E402


DIAGNOSTIC_KEYS = (
    "periodic_cost",
    "command_cost",
    "upright_cost",
    "action_diff_cost",
    "torque_cost",
    "forward_velocity_m_s",
    "left_force_n",
    "right_force_n",
    "left_foot_speed_m_s",
    "right_foot_speed_m_s",
    "torso_height_m",
    "lateral_position_m",
    "lateral_tilt_rad",
    "sagittal_tilt_rad",
    "lateral_tilt_rate_rad_s",
    "sagittal_tilt_rate_rad_s",
)


class Diagnostics(BaseCallback):
    """Write lightweight training diagnostics without changing V1 behavior."""

    def __init__(self, output: Path, interval: int, checkpoint_interval: int) -> None:
        super().__init__()
        self.output = output
        self.interval = interval
        self.next_report = interval
        self.checkpoint_interval = checkpoint_interval
        self.next_checkpoint = checkpoint_interval
        self.episodes: list[dict[str, float]] = []
        self.terminal_infos: list[dict[str, object]] = []
        self.metric_window: dict[str, list[float]] = {key: [] for key in DIAGNOSTIC_KEYS}
        self.action_abs: list[float] = []
        self.action_saturated: list[float] = []
        self.csv_path = output / "training_diagnostics.csv"

    def _on_training_start(self) -> None:
        with self.csv_path.open("w", newline="") as file:
            writer = csv.DictWriter(file, fieldnames=self._fieldnames())
            writer.writeheader()

    @staticmethod
    def _fieldnames() -> list[str]:
        return [
            "timesteps", "episodes", "mean_episode_reward", "mean_episode_length",
            "success_rate", "fall_rate", "mean_abs_action", "action_saturation_fraction",
            *[f"mean_{key}" for key in DIAGNOSTIC_KEYS],
        ]

    def _on_step(self) -> bool:
        infos = self.locals.get("infos", [])
        for info in infos:
            for key in DIAGNOSTIC_KEYS:
                if key in info:
                    self.metric_window[key].append(float(info[key]))
            if "episode" in info:
                self.episodes.append(info["episode"])
                self.terminal_infos.append(info)
        actions = np.asarray(self.locals.get("actions", []), dtype=np.float64)
        if actions.size:
            self.action_abs.extend(np.abs(actions).ravel().tolist())
            self.action_saturated.extend((np.abs(actions).ravel() > 0.98).astype(float).tolist())
        if self.num_timesteps >= self.next_report:
            recent = self.episodes[-20:]
            terminal = self.terminal_infos[-20:]
            row = {
                "timesteps": self.num_timesteps,
                "episodes": len(self.episodes),
                "mean_episode_reward": np.mean([x["r"] for x in recent]) if recent else float("nan"),
                "mean_episode_length": np.mean([x["l"] for x in recent]) if recent else float("nan"),
                "success_rate": np.mean([bool(x.get("is_success", False)) for x in terminal]) if terminal else float("nan"),
                "fall_rate": np.mean([not bool(x.get("TimeLimit.truncated", False)) for x in terminal]) if terminal else float("nan"),
                "mean_abs_action": np.mean(self.action_abs) if self.action_abs else float("nan"),
                "action_saturation_fraction": np.mean(self.action_saturated) if self.action_saturated else float("nan"),
                **{
                    f"mean_{key}": np.mean(values) if values else float("nan")
                    for key, values in self.metric_window.items()
                },
            }
            with self.csv_path.open("a", newline="") as file:
                csv.DictWriter(file, fieldnames=self._fieldnames()).writerow(row)
            print(
                f"steps={self.num_timesteps} reward={row['mean_episode_reward']:.2f} "
                f"length={row['mean_episode_length']:.1f} success={row['success_rate']:.2f} "
                f"sat={row['action_saturation_fraction']:.3f}",
                flush=True,
            )
            self.metric_window = {key: [] for key in DIAGNOSTIC_KEYS}
            self.action_abs.clear()
            self.action_saturated.clear()
            self.next_report += self.interval
        if self.checkpoint_interval > 0 and self.num_timesteps >= self.next_checkpoint:
            checkpoint = self.output / "checkpoints" / f"ppo_v1_{self.num_timesteps:09d}_steps"
            checkpoint.parent.mkdir(parents=True, exist_ok=True)
            self.model.save(checkpoint)
            self.next_checkpoint += self.checkpoint_interval
        return True


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--timesteps", type=int, default=200_000)
    parser.add_argument("--n-envs", type=int, default=4)
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--gait-frequency", type=float, default=0.8)
    parser.add_argument("--log-interval", type=int, default=25_000)
    parser.add_argument("--checkpoint-interval", type=int, default=100_000)
    parser.add_argument("--output", type=Path, default=REPO_ROOT / "outputs/periodic_gaits_research/v1")
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)

    config = PeriodicGaitConfig(gait_frequency_hz=args.gait_frequency)
    env = make_vec_env(
        lambda: Monitor(PeriodicGaitEnv(config=config)), n_envs=args.n_envs, seed=args.seed
    )
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
    callback = Diagnostics(args.output, args.log_interval, args.checkpoint_interval)
    started = time.time()
    model.learn(total_timesteps=args.timesteps, callback=callback)
    model.save(args.output / "ppo_periodic_walk_v1")
    summary = {
        "timesteps": model.num_timesteps,
        "n_envs": args.n_envs,
        "seed": args.seed,
        "config": config.__dict__,
        "elapsed_seconds": time.time() - started,
        "episodes": len(callback.episodes),
        "diagnostics": str(callback.csv_path.resolve()),
        "checkpoint_interval": args.checkpoint_interval,
    }
    (args.output / "training_summary.json").write_text(json.dumps(summary, indent=2))
    env.close()
    print(json.dumps(summary, indent=2), flush=True)


if __name__ == "__main__":
    main()
