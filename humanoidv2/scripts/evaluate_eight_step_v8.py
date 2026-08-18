#!/usr/bin/env python3
"""Evaluate a V8 eight-step episode and save video plus metrics."""

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

from humanoidv2 import EightStepV8Env  # noqa: E402


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--first-side", choices=("left", "right"), required=True)
    parser.add_argument("--model", type=Path, default=ROOT / "results/khr3hv_v8_symmetric/ppo_eight_step_final.zip")
    parser.add_argument("--output", type=Path, default=ROOT / "results/khr3hv_v8_symmetric")
    parser.add_argument("--reference-only", action="store_true")
    parser.add_argument("--seed", type=int, default=7)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    mode = "reference" if args.reference_only else "trained"
    label = f"eight_step_{args.first_side}_first_{mode}"
    policy = None if args.reference_only else PPO.load(args.model, device="cpu")
    env = EightStepV8Env(render_mode="rgb_array", reference_only=args.reference_only, fixed_first_side=args.first_side)
    observation, _ = env.reset(seed=args.seed)
    frames = [env.render()]
    records = []
    total_reward = 0.0
    terminated = truncated = False
    for step in range(env.config.max_episode_steps):
        action = np.zeros(10, dtype=np.float32) if policy is None else policy.predict(observation, deterministic=True)[0]
        observation, reward, terminated, truncated, info = env.step(action)
        total_reward += reward
        records.append({
            "step": step + 1,
            "time_seconds": (step + 1) * env.config.control_dt,
            "reward": float(reward),
            **{
                key: value if isinstance(value, (bool, str)) else float(value)
                for key, value in info.items()
                if isinstance(value, (bool, str, int, float, np.number))
            },
        })
        frames.append(env.render())
        if terminated or truncated:
            break
    env.close()
    video_path = args.output / f"{label}.mp4"
    imageio.mimsave(video_path, frames, fps=25, macro_block_size=16)
    imageio.imwrite(args.output / f"{label}_first.png", frames[0])
    imageio.imwrite(args.output / f"{label}_step4_landing.png", frames[min(950, len(frames) - 1)])
    imageio.imwrite(args.output / f"{label}_step8_swing.png", frames[min(1900, len(frames) - 1)])
    imageio.imwrite(args.output / f"{label}_final.png", frames[-1])
    final = [row for row in records if row["task_phase"] == "step8_settle"]
    summary: dict[str, object] = {
        "order": records[-1]["step_order"],
        "mode": mode,
        "reanchor_mode": records[-1]["reanchor_mode"],
        "model": None if policy is None else str(args.model),
        "steps": len(records),
        "duration_seconds": len(records) * env.config.control_dt,
        "total_reward": total_reward,
        "terminated": terminated,
        "truncated": truncated,
        "is_success": bool(records[-1]["is_success"]) if records else False,
        "base_forward_displacement_m": float(records[-1]["base_forward_displacement_m"]),
        "final_mean_left_force_n": float(np.mean([row["left_foot_force_n"] for row in final])) if final else 0.0,
        "final_mean_right_force_n": float(np.mean([row["right_foot_force_n"] for row in final])) if final else 0.0,
        "maximum_swing_height_m": float(max(row["swing_height_m"] for row in records)),
        "video": str(video_path),
    }
    for index in range(1, 9):
        summary[f"step_{index}_advance_mean_force_n"] = float(records[-1][f"step_{index}_advance_mean_force_n"])
        summary[f"step_{index}_length_m"] = float(records[-1][f"step_{index}_length_m"])
        summary[f"step_{index}_both_contact_fraction"] = float(records[-1][f"step_{index}_both_contact_fraction"])
        summary[f"step_{index}_peak_landing_force_n"] = float(records[-1][f"step_{index}_peak_landing_force_n"])
        summary[f"step_{index}_anchor_base_forward_m"] = float(records[-1][f"step_{index}_anchor_base_forward_m"])
    (args.output / f"{label}_summary.json").write_text(
        json.dumps(summary, indent=2, allow_nan=False), encoding="utf-8"
    )
    (args.output / f"{label}_trajectory.json").write_text(
        json.dumps(records, indent=2, allow_nan=False), encoding="utf-8"
    )
    print(json.dumps(summary, indent=2, allow_nan=False))


if __name__ == "__main__":
    main()
