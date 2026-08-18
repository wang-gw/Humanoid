#!/usr/bin/env python3
"""Fine-tune the small residual policy for V17 forward margin."""

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

from humanoidv2 import ForwardMarginV17Env  # noqa: E402
from scripts.train_dynamic_forward_v15 import Progress  # noqa: E402


def evaluate_candidate(path: Path, seed: int) -> list[dict[str, object]]:
    policy = PPO.load(path, device="cpu")
    rows = []
    for side in ("left", "right"):
        env = ForwardMarginV17Env(fixed_first_side=side)
        observation, _ = env.reset(seed=seed)
        total_reward = 0.0
        torques = []
        saturation = []
        for step in range(env.config.max_episode_steps):
            action = policy.predict(observation, deterministic=True)[0]
            observation, reward, terminated, truncated, info = env.step(action)
            total_reward += reward
            torques.append(float(info["peak_control_interval_torque_n_m"]))
            saturation.append(float(info["actuator_saturation_fraction"]))
            if terminated or truncated:
                break
        forward = float(info["base_forward_displacement_m"])
        rows.append(
            {
                "model": path.name,
                "first_side": side,
                "steps": step + 1,
                "reward": total_reward,
                "terminated": terminated,
                "truncated": truncated,
                "gait_success": bool(info["gait_success"]),
                "impact_limit_satisfied": bool(info["impact_limit_satisfied"]),
                "success": bool(info["is_success"]),
                "base_forward_displacement_m": forward,
                "forward_margin_over_200mm_m": forward - 0.200,
                "body_target_m": 8 * env.body_progress_per_step_m,
                "maximum_landing_force_n": float(info["maximum_landing_force_n"]),
                "minimum_step_length_m": min(
                    float(info[f"step_{index}_length_m"]) for index in range(1, 9)
                ),
                "minimum_advance_under_5n_fraction": min(
                    float(info[f"step_{index}_advance_under_5n_fraction"])
                    for index in range(1, 9)
                ),
                "maximum_actuator_torque_n_m": max(torques),
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
        "--initial-model", type=Path,
        default=ROOT / "results/khr3hv_v16_dynamic_forward_stage3/ppo_dynamic_forward_v16_final.zip",
    )
    parser.add_argument(
        "--output", type=Path,
        default=ROOT / "results/khr3hv_v17_forward_margin",
    )
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    (args.output / "checkpoints").mkdir(exist_ok=True)

    env = make_vec_env(
        lambda: Monitor(ForwardMarginV17Env()), n_envs=args.n_envs, seed=args.seed
    )
    initial_path = args.output / "ppo_forward_margin_v17_initial.zip"
    PPO.load(args.initial_model, device="cpu").save(initial_path)
    model = PPO.load(
        args.initial_model, env=env, device="cpu", learning_rate=args.learning_rate
    )
    progress = Progress()
    checkpoint = CheckpointCallback(
        save_freq=max(25_000 // args.n_envs, 1),
        save_path=str(args.output / "checkpoints"),
        name_prefix="forward_margin_v17",
    )
    started = time.time()
    model.learn(
        args.timesteps, callback=[progress, checkpoint],
        progress_bar=False, reset_num_timesteps=True,
    )
    last_path = args.output / "ppo_forward_margin_v17_last.zip"
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
            min(float(row["forward_margin_over_200mm_m"]) for row in grouped[path]),
            -max(float(row["maximum_landing_force_n"]) for row in grouped[path]),
            min(float(row["reward"]) for row in grouped[path]),
        ),
    )
    # Re-save for normal workflow; the final handoff later copies the selected
    # archive byte-for-byte after verification.
    PPO.load(selected, device="cpu").save(args.output / "ppo_forward_margin_v17_final")
    (args.output / "checkpoint_comparison.json").write_text(
        json.dumps(comparison, indent=2, allow_nan=False), encoding="utf-8"
    )
    match = re.search(r"_(\d+)_steps", selected.stem)
    recent = progress.episodes[-20:]
    summary = {
        "source_model": str(args.initial_model),
        "stride_m": 0.042,
        "minimum_forward_m": 0.200,
        "body_target_per_step_m": 0.0275,
        "body_target_episode_m": 0.220,
        "actual_transfer_timesteps": model.num_timesteps,
        "elapsed_seconds": time.time() - started,
        "episodes": len(progress.episodes),
        "last_20_reward": float(np.mean([row["r"] for row in recent])),
        "last_20_length": float(np.mean([row["l"] for row in recent])),
        "selected_model": selected.name,
        "selected_transfer_steps": (
            int(match.group(1)) if match
            else (0 if selected == initial_path else model.num_timesteps)
        ),
        "selection_rule": (
            "both-order strict success, worst forward margin over 200 mm, "
            "worst landing impact, then worst reward"
        ),
    }
    (args.output / "training_summary.json").write_text(
        json.dumps(summary, indent=2, allow_nan=False), encoding="utf-8"
    )
    env.close()
    print(json.dumps(summary, indent=2, allow_nan=False), flush=True)


if __name__ == "__main__":
    main()
