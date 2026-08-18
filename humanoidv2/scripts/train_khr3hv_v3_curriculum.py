#!/usr/bin/env python3
"""Train V3 sequentially at 0.05, 0.10, and 0.15 m/s."""

from __future__ import annotations

import argparse
import json
import sys
import time
from dataclasses import replace
from pathlib import Path

import numpy as np
import torch
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import BaseCallback, CheckpointCallback
from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3.common.monitor import Monitor

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from humanoidv2 import KHR3HVV3Env, KHR3HVV31Env, KHR3HVV32Env, KHR3HVV33Env, KHR3HVV34Env  # noqa: E402
from humanoidv2.khr3hv_env import KHRConfig  # noqa: E402


class StageProgressCallback(BaseCallback):
    def __init__(self, stage: int, start_step: int, interval: int = 25_000) -> None:
        super().__init__()
        self.stage = stage
        self.next_report = start_step + interval
        self.interval = interval
        self.episodes: list[dict[str, float]] = []

    def _on_step(self) -> bool:
        for info in self.locals.get("infos", []):
            episode = info.get("episode")
            if episode:
                self.episodes.append({key: float(value) for key, value in episode.items()})
        if self.num_timesteps >= self.next_report:
            recent = self.episodes[-20:]
            mean_reward = float(np.mean([item["r"] for item in recent])) if recent else float("nan")
            mean_length = float(np.mean([item["l"] for item in recent])) if recent else float("nan")
            print(
                f"stage={self.stage} steps={self.num_timesteps} "
                f"recent_reward={mean_reward:.3f} recent_length={mean_length:.1f}",
                flush=True,
            )
            self.next_report += self.interval
        return True


def make_environment(env_class, speed: float, torque_scale: float, sway_width: float, n_envs: int, seed: int):
    config = replace(
        KHRConfig(),
        forward_speed=speed,
        torque_penalty_scale=torque_scale,
        sway_width=sway_width,
        sway_offset=sway_width / 4.0,
    )

    def factory() -> Monitor:
        return Monitor(env_class(config=config))

    return make_vec_env(factory, n_envs=n_envs, seed=seed)


def evaluate(
    model: PPO, env_class, speed: float, torque_scale: float, sway_width: float
) -> dict[str, float | int | bool]:
    config = replace(
        KHRConfig(),
        forward_speed=speed,
        torque_penalty_scale=torque_scale,
        sway_width=sway_width,
        sway_offset=sway_width / 4.0,
    )
    env = env_class(config=config)
    observation, _ = env.reset(seed=7)
    total_reward = 0.0
    info: dict[str, float | bool] = {"distance": 0.0, "mean_episode_forward_velocity": 0.0, "is_success": False}
    terminated = truncated = False
    for step in range(500):
        action = model.predict(observation, deterministic=True)[0]
        observation, reward, terminated, truncated, info = env.step(action)
        total_reward += reward
        if terminated or truncated:
            break
    result = {
        "steps_survived": step + 1,
        "total_reward": total_reward,
        "distance_m": float(info["distance"]),
        "mean_forward_velocity_mps": float(info["mean_episode_forward_velocity"]),
        "terminated": terminated,
        "is_success": bool(info["is_success"]),
    }
    env.close()
    return result


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--steps-per-stage", type=int, default=200_000)
    parser.add_argument("--n-envs", type=int, default=4)
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--version", choices=("v3", "v3_1", "v3_2", "v3_3", "v3_4"), default="v3")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    if args.output is None:
        args.output = ROOT / f"results/khr3hv_{args.version}"
    args.output.mkdir(parents=True, exist_ok=True)
    (args.output / "checkpoints").mkdir(exist_ok=True)

    stages = (
        (0.05, 0.003, 0.030),
        (0.10, 0.005, 0.0325),
        (0.15, 0.010, 0.035),
    )
    env_class = {
        "v3": KHR3HVV3Env,
        "v3_1": KHR3HVV31Env,
        "v3_2": KHR3HVV32Env,
        "v3_3": KHR3HVV33Env,
        "v3_4": KHR3HVV34Env,
    }[args.version]
    model: PPO | None = None
    results: list[dict[str, object]] = []
    started = time.time()
    for stage_index, (speed, torque_scale, sway_width) in enumerate(stages, start=1):
        env = make_environment(
            env_class, speed, torque_scale, sway_width, args.n_envs, args.seed + stage_index - 1
        )
        if model is None:
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
        else:
            model.set_env(env)
        start_step = model.num_timesteps
        progress = StageProgressCallback(stage_index, start_step)
        checkpoint = CheckpointCallback(
            save_freq=max(50_000 // args.n_envs, 1),
            save_path=str(args.output / "checkpoints"),
            name_prefix=f"stage{stage_index}_speed{int(speed * 100):02d}",
        )
        model.learn(
            total_timesteps=args.steps_per_stage,
            callback=[progress, checkpoint],
            reset_num_timesteps=stage_index == 1,
            progress_bar=False,
        )
        model.save(args.output / f"ppo_stage{stage_index}_speed{int(speed * 100):02d}")
        stage_result = evaluate(model, env_class, speed, torque_scale, sway_width)
        stage_result.update(
            {
                "stage": stage_index,
                "commanded_speed_mps": speed,
                "torque_penalty_scale": torque_scale,
                "sway_width_m": sway_width,
                "model_timesteps": model.num_timesteps,
            }
        )
        results.append(stage_result)
        print(json.dumps(stage_result, indent=2), flush=True)
        env.close()

    assert model is not None
    model.save(args.output / "ppo_khr3hv_final")
    summary = {
        "version": args.version,
        "seed": args.seed,
        "n_envs": args.n_envs,
        "steps_per_stage": args.steps_per_stage,
        "actual_total_timesteps": model.num_timesteps,
        "elapsed_seconds": time.time() - started,
        "stages": results,
    }
    (args.output / "training_summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    print(json.dumps(summary, indent=2), flush=True)


if __name__ == "__main__":
    main()
