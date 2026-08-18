#!/usr/bin/env python3
"""Evaluate all saved checkpoints without rendering."""

from __future__ import annotations

import json
import re
import sys
import argparse
from pathlib import Path

import numpy as np
from stable_baselines3 import PPO

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from humanoidv2 import KHR3HVEnv, KHR3HVV2Env, KHR3HVV21Env, KHR3HVV22Env, KHR3HVV3Env, KHR3HVV31Env, KHR3HVV32Env, KHR3HVV33Env, KHR3HVV34Env  # noqa: E402


def checkpoint_step(path: Path) -> int:
    match = re.search(r"_(\d+)_steps", path.stem)
    return int(match.group(1)) if match else 0


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--version", choices=("v1", "v2", "v2_1", "v2_2", "v3", "v3_1", "v3_2", "v3_3", "v3_4"), default="v1")
    args = parser.parse_args()
    result_dir = ROOT / f"results/khr3hv_{args.version}"
    paths = sorted((result_dir / "checkpoints").glob("*.zip"), key=checkpoint_step)
    paths.append(result_dir / "ppo_khr3hv_final.zip")
    env_class = {
        "v1": KHR3HVEnv,
        "v2": KHR3HVV2Env,
        "v2_1": KHR3HVV21Env,
        "v2_2": KHR3HVV22Env,
        "v3": KHR3HVV3Env,
        "v3_1": KHR3HVV31Env,
        "v3_2": KHR3HVV32Env,
        "v3_3": KHR3HVV33Env,
        "v3_4": KHR3HVV34Env,
    }[args.version]
    env = env_class()
    results: list[dict[str, float | int | str | bool]] = []
    for path in paths:
        policy = PPO.load(path, device="cpu")
        observation, _ = env.reset(seed=7)
        total_reward = 0.0
        velocities: list[float] = []
        info = {"distance": 0.0}
        terminated = truncated = False
        for step in range(500):
            action = policy.predict(observation, deterministic=True)[0]
            observation, reward, terminated, truncated, info = env.step(action)
            total_reward += reward
            velocities.append(float(info["forward_velocity"]))
            if terminated or truncated:
                break
        results.append(
            {
                "model": path.name,
                "training_steps": checkpoint_step(path) or policy.num_timesteps,
                "steps_survived": step + 1,
                "total_reward": total_reward,
                "distance_m": float(info["distance"]),
                "mean_forward_velocity_mps": float(np.mean(velocities)),
                "terminated": terminated,
                "is_success": bool(info["is_success"]),
            }
        )
    env.close()
    output = result_dir / "checkpoint_comparison.json"
    output.write_text(json.dumps(results, indent=2), encoding="utf-8")
    print(json.dumps(results, indent=2))


if __name__ == "__main__":
    main()
