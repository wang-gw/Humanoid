#!/usr/bin/env python3
"""Evaluate a trained policy (or the reference alone) and save MP4 + JSON."""

from __future__ import annotations

import argparse
import json
import os
import sys
from pathlib import Path

os.environ.setdefault("MUJOCO_GL", "egl")

import imageio.v2 as imageio
import numpy as np
from stable_baselines3 import PPO

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from humanoidv2 import KHR3HVEnv, KHR3HVV2Env, KHR3HVV21Env, KHR3HVV22Env, KHR3HVV3Env, KHR3HVV31Env, KHR3HVV32Env, KHR3HVV33Env, KHR3HVV34Env  # noqa: E402


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--version", choices=("v1", "v2", "v2_1", "v2_2", "v3", "v3_1", "v3_2", "v3_3", "v3_4"), default="v1")
    parser.add_argument("--model", type=Path)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--steps", type=int, default=500)
    parser.add_argument("--reference-only", action="store_true")
    args = parser.parse_args()
    if args.output is None:
        args.output = ROOT / f"results/khr3hv_{args.version}"
    if args.model is None:
        args.model = args.output / "ppo_khr3hv_final.zip"
    args.output.mkdir(parents=True, exist_ok=True)
    label = "reference_only" if args.reference_only else "trained_policy"
    policy = None if args.reference_only else PPO.load(args.model, device="cpu")
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
    env = env_class(render_mode="rgb_array", reference_only=args.reference_only)
    observation, _ = env.reset(seed=7)
    frames: list[np.ndarray] = [env.render()]
    records: list[dict[str, float | int | bool]] = []
    total_reward = 0.0
    terminated = truncated = False
    for step in range(args.steps):
        action = np.zeros(10, dtype=np.float32) if policy is None else policy.predict(observation, deterministic=True)[0]
        observation, reward, terminated, truncated, info = env.step(action)
        total_reward += reward
        row = {key: float(value) for key, value in info.items() if isinstance(value, (int, float, np.number))}
        row.update({"step": step + 1, "reward": float(reward), "terminated": terminated, "truncated": truncated})
        records.append(row)
        frames.append(env.render())
        if terminated or truncated:
            break
    env.close()

    video_path = args.output / f"{label}.mp4"
    imageio.mimsave(video_path, frames, fps=25, macro_block_size=16)
    imageio.imwrite(args.output / f"{label}_first.png", frames[0])
    imageio.imwrite(args.output / f"{label}_midpoint.png", frames[len(frames) // 2])
    imageio.imwrite(args.output / f"{label}_final.png", frames[-1])
    summary = {
        "mode": label,
        "steps_survived": len(records),
        "duration_seconds": len(records) / 25.0,
        "total_reward": total_reward,
        "mean_reward": total_reward / max(len(records), 1),
        "distance_m": records[-1]["distance"] if records else 0.0,
        "mean_forward_velocity_mps": float(np.mean([r["forward_velocity"] for r in records])) if records else 0.0,
        "max_forward_velocity_mps": float(np.max([r["forward_velocity"] for r in records])) if records else 0.0,
        "final_torso_height_m": records[-1]["torso_height"] if records else 0.0,
        "terminated": terminated,
        "truncated": truncated,
        "is_success": bool(records[-1].get("is_success", False)) if records else False,
        "video": str(video_path),
    }
    (args.output / f"{label}_summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    (args.output / f"{label}_trajectory.json").write_text(json.dumps(records, indent=2), encoding="utf-8")
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
