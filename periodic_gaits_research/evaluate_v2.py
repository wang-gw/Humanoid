#!/usr/bin/env python3
"""Evaluate V2 and compare it with the original V1 policy."""

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

from periodic_gaits import JOINT_NAMES, NaturalGaitConfig, PeriodicGaitConfig, PeriodicGaitEnv  # noqa: E402


def rollout(policy: PPO, config, seed: int, video: Path | None = None) -> dict[str, object]:
    env = PeriodicGaitEnv(config=config, render_mode="rgb_array" if video else None)
    obs, _ = env.reset(seed=seed)
    frames = [env.render()] if video else []
    q_values = []
    actions = []
    swing_knee_flexion = []
    stance_knee_flexion = []
    swing_clearance = []
    swing_forward_velocity_relative_base = []
    swing_knee_forward_advantage = []
    info = {}
    terminated = truncated = False
    for step in range(1, env.max_episode_steps + 1):
        action = policy.predict(obs, deterministic=True)[0]
        obs, _, terminated, truncated, info = env.step(action)
        q_values.append(env.data.qpos[env.qpos_ids].copy())
        actions.append(action.copy())
        for side in ("left", "right"):
            swing_weight = float(info[f"{side}_swing_weight"])
            knee_flexion = float(info[f"{side}_knee_flexion_rad"])
            if swing_weight > 0.8:
                swing_knee_flexion.append(knee_flexion)
                swing_clearance.append(float(info[f"{side}_foot_clearance_m"]))
                swing_forward_velocity_relative_base.append(
                    float(info[f"{side}_foot_forward_velocity_relative_base_m_s"])
                )
                other = "right" if side == "left" else "left"
                swing_knee_forward_advantage.append(
                    float(info[f"{side}_knee_forward_m"] - info[f"{other}_knee_forward_m"])
                )
            elif swing_weight < 0.2:
                stance_knee_flexion.append(knee_flexion)
        if video and step % 2 == 0:
            frames.append(env.render())
        if terminated or truncated:
            break
    if video:
        imageio.mimsave(video, frames, fps=50, macro_block_size=16)
    q = np.asarray(q_values)
    actions_array = np.asarray(actions)
    result = {
        "seed": seed,
        "steps": step,
        "duration_s": step * env.config.control_dt,
        "terminated": terminated,
        "success": bool(info.get("is_success", False)),
        "gait_pattern_satisfied": bool(info.get("gait_pattern_satisfied", False)),
        "forward_distance_m": float(info.get("forward_distance_m", 0.0)),
        "mean_swing_force_n": float(info.get("mean_swing_force", 0.0)),
        "mean_stance_force_n": float(info.get("mean_stance_force", 0.0)),
        "mean_swing_speed_m_s": float(info.get("mean_swing_speed", 0.0)),
        "mean_stance_speed_m_s": float(info.get("mean_stance_speed", 0.0)),
        "peak_torque_nm": float(info.get("max_abs_torque_nm", 0.0)),
        "action_saturation_fraction": float(np.mean(np.abs(actions_array) > 0.98)),
        "mean_swing_knee_flexion_rad": float(np.mean(swing_knee_flexion)),
        "mean_stance_knee_flexion_rad": float(np.mean(stance_knee_flexion)),
        "mean_swing_foot_clearance_m": float(np.mean(swing_clearance)),
        "mean_swing_foot_forward_velocity_relative_base_m_s": float(
            np.mean(swing_forward_velocity_relative_base)
        ),
        "mean_swing_knee_forward_advantage_m": float(np.mean(swing_knee_forward_advantage)),
        "joint_ranges_rad": {
            name: float(np.ptp(q[:, index])) for index, name in enumerate(JOINT_NAMES)
        },
    }
    if video:
        result["video"] = str(video.resolve())
    env.close()
    return result


def aggregate(rows: list[dict[str, object]]) -> dict[str, object]:
    return {
        "runs": len(rows),
        "successes": sum(bool(row["success"]) for row in rows),
        "falls": sum(bool(row["terminated"]) for row in rows),
        "mean_forward_distance_m": float(np.mean([row["forward_distance_m"] for row in rows])),
        "mean_swing_force_n": float(np.mean([row["mean_swing_force_n"] for row in rows])),
        "mean_stance_force_n": float(np.mean([row["mean_stance_force_n"] for row in rows])),
        "mean_action_saturation_fraction": float(np.mean([row["action_saturation_fraction"] for row in rows])),
        "mean_swing_knee_flexion_rad": float(np.mean([row["mean_swing_knee_flexion_rad"] for row in rows])),
        "mean_stance_knee_flexion_rad": float(np.mean([row["mean_stance_knee_flexion_rad"] for row in rows])),
        "mean_swing_foot_clearance_m": float(np.mean([row["mean_swing_foot_clearance_m"] for row in rows])),
        "mean_swing_foot_forward_velocity_relative_base_m_s": float(
            np.mean([row["mean_swing_foot_forward_velocity_relative_base_m_s"] for row in rows])
        ),
        "mean_swing_knee_forward_advantage_m": float(
            np.mean([row["mean_swing_knee_forward_advantage_m"] for row in rows])
        ),
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--v1-model", type=Path, default=PROJECT_DIR / "results/v1_baseline/ppo_periodic_walk_v1.zip")
    parser.add_argument("--v2-model", type=Path, default=PROJECT_DIR / "results/v2_experimental/ppo_periodic_walk_v2.zip")
    parser.add_argument("--output", type=Path, default=REPO_ROOT / "outputs/periodic_gaits_research/comparison")
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    v1 = PPO.load(args.v1_model, device="cpu")
    v2 = PPO.load(args.v2_model, device="cpu")
    v1_rows = [rollout(v1, PeriodicGaitConfig(), seed) for seed in range(30, 40)]
    v2_rows = [rollout(v2, NaturalGaitConfig(), seed) for seed in range(30, 40)]
    representative_v1 = rollout(v1, PeriodicGaitConfig(), 17, args.output / "v1_comparison.mp4")
    representative_v2 = rollout(v2, NaturalGaitConfig(), 17, args.output / "v2_natural_swing.mp4")
    summary = {
        "v1": aggregate(v1_rows),
        "v2": aggregate(v2_rows),
        "v1_representative": representative_v1,
        "v2_representative": representative_v2,
        "v1_runs": v1_rows,
        "v2_runs": v2_rows,
    }
    (args.output / "comparison_summary.json").write_text(json.dumps(summary, indent=2))
    print(json.dumps(summary, indent=2), flush=True)


if __name__ == "__main__":
    main()
