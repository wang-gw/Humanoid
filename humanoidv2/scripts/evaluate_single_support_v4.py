#!/usr/bin/env python3
"""Evaluate V4 static single support and save an MP4 plus metrics."""

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

from humanoidv2 import SingleSupportV4Env  # noqa: E402


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--side", choices=("left", "right"), required=True)
    parser.add_argument("--model", type=Path)
    parser.add_argument("--robot-model", type=Path)
    parser.add_argument("--task-profile", choices=("original", "symmetric"), default="original")
    parser.add_argument("--output", type=Path)
    parser.add_argument("--reference-only", action="store_true")
    parser.add_argument("--seed", type=int, default=7)
    args = parser.parse_args()
    if args.output is None:
        suffix = "_symmetric" if args.task_profile == "symmetric" else ""
        args.output = ROOT / f"results/khr3hv_v4{suffix}"
    if args.model is None:
        args.model = args.output / "ppo_single_support_final.zip"
    args.output.mkdir(parents=True, exist_ok=True)

    mode = "reference" if args.reference_only else "trained"
    label = f"single_support_{args.side}_{mode}"
    policy = None if args.reference_only else PPO.load(args.model, device="cpu")
    env = SingleSupportV4Env(
        render_mode="rgb_array",
        reference_only=args.reference_only,
        fixed_side=args.side,
        model_path=args.robot_model,
        task_profile=args.task_profile,
    )
    observation, _ = env.reset(seed=args.seed)
    frames: list[np.ndarray] = [env.render()]
    records: list[dict[str, float | int | bool | str]] = []
    total_reward = 0.0
    terminated = truncated = False
    for step in range(env.config.max_episode_steps):
        action = np.zeros(10, dtype=np.float32) if policy is None else policy.predict(observation, deterministic=True)[0]
        observation, reward, terminated, truncated, info = env.step(action)
        total_reward += reward
        records.append(
            {
                "step": step + 1,
                "time_seconds": (step + 1) * env.config.control_dt,
                "reward": float(reward),
                **{
                    key: value if isinstance(value, (bool, str)) else float(value)
                    for key, value in info.items()
                    if isinstance(value, (bool, str, int, float, np.number))
                },
            }
        )
        frames.append(env.render())
        if terminated or truncated:
            break
    env.close()

    video_path = args.output / f"{label}.mp4"
    imageio.mimsave(video_path, frames, fps=25, macro_block_size=16)
    imageio.imwrite(args.output / f"{label}_first.png", frames[0])
    imageio.imwrite(args.output / f"{label}_final.png", frames[-1])
    hold = records[-50:]
    summary = {
        "side": args.side,
        "mode": mode,
        "model": None if policy is None else str(args.model),
        "robot_model": str(env.model_path),
        "task_profile": args.task_profile,
        "steps": len(records),
        "duration_seconds": len(records) * env.config.control_dt,
        "total_reward": total_reward,
        "terminated": terminated,
        "truncated": truncated,
        "is_success": bool(records[-1]["is_success"]) if records else False,
        "last_2s_mean_swing_force_n": float(np.mean([row["swing_force_n"] for row in hold])),
        "last_2s_swing_under_5n_fraction": float(np.mean([row["swing_force_n"] < 5.0 for row in hold])),
        "last_2s_mean_stance_force_n": float(np.mean([row["stance_force_n"] for row in hold])),
        "maximum_swing_sole_height_m": float(max(row["swing_height_m"] for row in records)),
        "final_torso_height_m": float(records[-1]["torso_height"]) if records else 0.0,
        "video": str(video_path),
    }
    (args.output / f"{label}_summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    (args.output / f"{label}_trajectory.json").write_text(json.dumps(records, indent=2), encoding="utf-8")
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
