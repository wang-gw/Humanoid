#!/usr/bin/env python3
"""Compare zero action and a trained periodic gait policy."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import imageio.v2 as imageio
import numpy as np
from stable_baselines3 import PPO

PROJECT_DIR = Path(__file__).resolve().parent
REPO_ROOT = PROJECT_DIR.parent
sys.path.insert(0, str(Path(__file__).resolve().parent))

from periodic_gaits import PeriodicGaitEnv  # noqa: E402


def rollout(model: PPO | None, output: Path, seed: int) -> dict[str, object]:
    env = PeriodicGaitEnv(render_mode="rgb_array")
    observation, _ = env.reset(seed=seed)
    frames = [env.render()]
    total_reward = 0.0
    info = {}
    terminated = truncated = False
    for step in range(1, env.max_episode_steps + 1):
        action = np.zeros(10, dtype=np.float32) if model is None else model.predict(observation, deterministic=True)[0]
        observation, reward, terminated, truncated, info = env.step(action)
        total_reward += reward
        if step % 2 == 0:  # 100 Hz control to 50 fps video.
            frames.append(env.render())
        if terminated or truncated:
            break
    imageio.mimsave(output, frames, fps=50, macro_block_size=16)
    summary = {
        "steps": step,
        "duration_s": step * env.config.control_dt,
        "terminated": terminated,
        "truncated": truncated,
        "success": bool(info.get("is_success", False)),
        "forward_distance_m": float(info.get("forward_distance_m", 0.0)),
        "final_forward_velocity_m_s": float(info.get("forward_velocity_m_s", 0.0)),
        "final_periodic_cost": float(info.get("periodic_cost", 0.0)),
        "max_abs_torque_nm": float(info.get("max_abs_torque_nm", 0.0)),
        "gait_pattern_satisfied": bool(info.get("gait_pattern_satisfied", False)),
        "mean_swing_force_n": float(info.get("mean_swing_force", 0.0)),
        "mean_stance_force_n": float(info.get("mean_stance_force", 0.0)),
        "mean_swing_speed_m_s": float(info.get("mean_swing_speed", 0.0)),
        "mean_stance_speed_m_s": float(info.get("mean_stance_speed", 0.0)),
        "total_reward": total_reward,
        "video": str(output.resolve()),
    }
    env.close()
    return summary


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", type=Path, default=PROJECT_DIR / "results/v1_baseline/ppo_periodic_walk_v1.zip")
    parser.add_argument("--output", type=Path, default=REPO_ROOT / "outputs/periodic_gaits_research/evaluation_v1")
    parser.add_argument("--seed", type=int, default=17)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    model = PPO.load(args.model, device="cpu")
    summary = {
        "zero_action": rollout(None, args.output / "zero_action.mp4", args.seed),
        "trained_policy": rollout(model, args.output / "trained_policy.mp4", args.seed),
    }
    (args.output / "evaluation_summary.json").write_text(json.dumps(summary, indent=2))
    print(json.dumps(summary, indent=2), flush=True)


if __name__ == "__main__":
    main()
