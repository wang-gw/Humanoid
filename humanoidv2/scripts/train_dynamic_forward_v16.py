#!/usr/bin/env python3
"""Transfer V15 PPO into the 2.56-second V16 speed stage."""

from __future__ import annotations

import argparse
import json
import re
import sys
import time
from pathlib import Path

import numpy as np
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import CheckpointCallback
from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3.common.monitor import Monitor

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from humanoidv2 import DynamicForwardV16Env  # noqa: E402
from scripts.train_dynamic_forward_v15 import Progress  # noqa: E402


def evaluate_candidate(path: Path, seed: int) -> list[dict[str, object]]:
    policy = PPO.load(path, device="cpu")
    rows = []
    for side in ("left", "right"):
        env = DynamicForwardV16Env(fixed_first_side=side)
        observation, _ = env.reset(seed=seed)
        total_reward = 0.0
        torque_peaks: list[float] = []
        saturation: list[float] = []
        for step in range(env.config.max_episode_steps):
            action = policy.predict(observation, deterministic=True)[0]
            observation, reward, terminated, truncated, info = env.step(action)
            total_reward += reward
            torque_peaks.append(float(info["peak_control_interval_torque_n_m"]))
            saturation.append(float(info["actuator_saturation_fraction"]))
            if terminated or truncated:
                break
        rows.append(
            {
                "model": path.name,
                "first_side": side,
                "steps": step + 1,
                "duration_seconds": (step + 1) * env.config.control_dt,
                "reward": total_reward,
                "terminated": terminated,
                "truncated": truncated,
                "gait_success": bool(info["gait_success"]),
                "impact_limit_satisfied": bool(info["impact_limit_satisfied"]),
                "success": bool(info["is_success"]),
                "forward_axis": info["forward_axis"],
                "base_forward_displacement_m": float(info["base_forward_displacement_m"]),
                "maximum_landing_force_n": float(info["maximum_landing_force_n"]),
                "minimum_step_length_m": min(
                    float(info[f"step_{index}_length_m"]) for index in range(1, 9)
                ),
                "minimum_advance_under_5n_fraction": min(
                    float(info[f"step_{index}_advance_under_5n_fraction"])
                    for index in range(1, 9)
                ),
                "maximum_actuator_torque_n_m": max(torque_peaks),
                "maximum_saturation_fraction": max(saturation),
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
        / "results/khr3hv_v15_dynamic_forward_stage2/ppo_dynamic_forward_v15_final.zip",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=ROOT / "results/khr3hv_v16_dynamic_forward_stage3",
    )
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    (args.output / "checkpoints").mkdir(exist_ok=True)

    env = make_vec_env(
        lambda: Monitor(DynamicForwardV16Env()), n_envs=args.n_envs, seed=args.seed
    )
    initial_path = args.output / "ppo_dynamic_forward_v16_initial.zip"
    PPO.load(args.initial_model, device="cpu").save(initial_path)
    model = PPO.load(
        args.initial_model, env=env, device="cpu", learning_rate=args.learning_rate
    )
    progress = Progress()
    checkpoint = CheckpointCallback(
        save_freq=max(25_000 // args.n_envs, 1),
        save_path=str(args.output / "checkpoints"),
        name_prefix="dynamic_forward_v16",
    )
    started = time.time()
    model.learn(
        args.timesteps,
        callback=[progress, checkpoint],
        progress_bar=False,
        reset_num_timesteps=True,
    )
    last_path = args.output / "ppo_dynamic_forward_v16_last.zip"
    model.save(last_path)
    candidates = (
        [initial_path]
        + sorted((args.output / "checkpoints").glob("*.zip"))
        + [last_path]
    )
    comparison = [row for path in candidates for row in evaluate_candidate(path, args.seed)]
    grouped = {
        path: [row for row in comparison if row["model"] == path.name]
        for path in candidates
    }
    selected = max(
        candidates,
        key=lambda path: (
            sum(bool(row["success"]) for row in grouped[path]),
            sum(bool(row["gait_success"]) for row in grouped[path]),
            sum(int(row["steps"]) for row in grouped[path]),
            -max(float(row["maximum_landing_force_n"]) for row in grouped[path]),
            min(float(row["reward"]) for row in grouped[path]),
        ),
    )
    PPO.load(selected, device="cpu").save(args.output / "ppo_dynamic_forward_v16_final")
    (args.output / "checkpoint_comparison.json").write_text(
        json.dumps(comparison, indent=2, allow_nan=False), encoding="utf-8"
    )
    match = re.search(r"_(\d+)_steps", selected.stem)
    recent = progress.episodes[-20:]
    summary = {
        "source_model": str(args.initial_model),
        "speed_curriculum_stage": 3,
        "step_cycle_steps": 64,
        "step_cycle_seconds": 2.56,
        "gain_transition_fraction": 0.12,
        "actual_transfer_timesteps": model.num_timesteps,
        "elapsed_seconds": time.time() - started,
        "episodes": len(progress.episodes),
        "last_20_reward": float(np.mean([row["r"] for row in recent])),
        "last_20_length": float(np.mean([row["l"] for row in recent])),
        "selected_model": selected.name,
        "selected_transfer_steps": (
            int(match.group(1))
            if match
            else (0 if selected == initial_path else model.num_timesteps)
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
