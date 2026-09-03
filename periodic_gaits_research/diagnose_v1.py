#!/usr/bin/env python3
"""Extract detailed per-joint and reward diagnostics from a trained V1 policy."""

from __future__ import annotations

import argparse
import csv
import json
import sys
from collections import defaultdict
from pathlib import Path

import imageio.v2 as imageio
import numpy as np
from stable_baselines3 import PPO

PROJECT_DIR = Path(__file__).resolve().parent
REPO_ROOT = PROJECT_DIR.parent
sys.path.insert(0, str(PROJECT_DIR))

from periodic_gaits import JOINT_NAMES, PeriodicGaitEnv  # noqa: E402


REWARD_KEYS = (
    "periodic_cost", "command_cost", "upright_cost", "action_diff_cost",
    "torque_cost", "angular_cost", "forward_velocity_m_s", "left_force_n",
    "right_force_n", "left_foot_speed_m_s", "right_foot_speed_m_s",
    "torso_height_m",
)


def rollout(model: PPO, seed: int, trace_path: Path | None, video_path: Path | None) -> tuple[dict, dict]:
    env = PeriodicGaitEnv(render_mode="rgb_array" if video_path else None)
    observation, _ = env.reset(seed=seed)
    series: dict[str, list[float]] = defaultdict(list)
    joint_series = {
        name: defaultdict(list) for name in JOINT_NAMES
    }
    trace_rows: list[dict[str, float | int | bool]] = []
    frames = [env.render()] if video_path else []
    total_reward = 0.0
    info: dict = {}
    terminated = truncated = False

    for step in range(1, env.max_episode_steps + 1):
        action = np.asarray(model.predict(observation, deterministic=True)[0], dtype=np.float64)
        observation, reward, terminated, truncated, info = env.step(action)
        total_reward += reward
        q = env.data.qpos[env.qpos_ids].copy()
        qd = env.data.qvel[env.dof_ids].copy()
        target = env.filtered_target.copy()
        tracking_error = target - q
        applied_torque = env.data.ctrl.copy()
        unclipped_torque = env.config.kp * tracking_error - env.config.kd * qd

        for key in REWARD_KEYS:
            series[key].append(float(info[key]))
        for index, name in enumerate(JOINT_NAMES):
            values = joint_series[name]
            values["action"].append(float(action[index]))
            values["position_rad"].append(float(q[index]))
            values["velocity_rad_s"].append(float(qd[index]))
            values["target_rad"].append(float(target[index]))
            values["tracking_error_rad"].append(float(tracking_error[index]))
            values["unclipped_torque_nm"].append(float(unclipped_torque[index]))
            values["applied_torque_nm"].append(float(applied_torque[index]))

        if trace_path:
            row: dict[str, float | int | bool] = {
                "step": step,
                "time_s": step * env.config.control_dt,
                "phase": float(info["phase"]),
                "reward": float(reward),
                "terminated": terminated,
                **{key: float(info[key]) for key in REWARD_KEYS},
            }
            for index, name in enumerate(JOINT_NAMES):
                row.update({
                    f"{name}_action": float(action[index]),
                    f"{name}_position_rad": float(q[index]),
                    f"{name}_target_rad": float(target[index]),
                    f"{name}_tracking_error_rad": float(tracking_error[index]),
                    f"{name}_velocity_rad_s": float(qd[index]),
                    f"{name}_unclipped_torque_nm": float(unclipped_torque[index]),
                    f"{name}_applied_torque_nm": float(applied_torque[index]),
                })
            trace_rows.append(row)
        if video_path and step % 2 == 0:
            frames.append(env.render())
        if terminated or truncated:
            break

    if trace_path and trace_rows:
        with trace_path.open("w", newline="") as file:
            writer = csv.DictWriter(file, fieldnames=list(trace_rows[0]))
            writer.writeheader()
            writer.writerows(trace_rows)
    if video_path:
        imageio.mimsave(video_path, frames, fps=50, macro_block_size=16)

    joint_summary = {}
    for index, name in enumerate(JOINT_NAMES):
        values = {key: np.asarray(value) for key, value in joint_series[name].items()}
        joint_summary[name] = {
            "action_saturation_fraction": float(np.mean(np.abs(values["action"]) > 0.98)),
            "mean_abs_action": float(np.mean(np.abs(values["action"]))),
            "position_range_rad": float(np.ptp(values["position_rad"])),
            "mean_abs_tracking_error_rad": float(np.mean(np.abs(values["tracking_error_rad"]))),
            "max_abs_tracking_error_rad": float(np.max(np.abs(values["tracking_error_rad"]))),
            "mean_abs_velocity_rad_s": float(np.mean(np.abs(values["velocity_rad_s"]))),
            "max_abs_unclipped_torque_nm": float(np.max(np.abs(values["unclipped_torque_nm"]))),
            "torque_clip_fraction": float(
                np.mean(np.abs(values["applied_torque_nm"]) >= env.torque_limit[index] - 1e-6)
            ),
        }
    episode = {
        "seed": seed,
        "steps": step,
        "duration_s": step * env.config.control_dt,
        "terminated": terminated,
        "truncated": truncated,
        "success": bool(info.get("is_success", False)),
        "gait_pattern_satisfied": bool(info.get("gait_pattern_satisfied", False)),
        "forward_distance_m": float(info.get("forward_distance_m", 0.0)),
        "total_reward": float(total_reward),
        "mean_swing_force_n": float(info.get("mean_swing_force", 0.0)),
        "mean_stance_force_n": float(info.get("mean_stance_force", 0.0)),
        "mean_swing_speed_m_s": float(info.get("mean_swing_speed", 0.0)),
        "mean_stance_speed_m_s": float(info.get("mean_stance_speed", 0.0)),
        "reward_metrics": {
            key: {
                "mean": float(np.mean(values)),
                "std": float(np.std(values)),
                "min": float(np.min(values)),
                "max": float(np.max(values)),
            }
            for key, values in series.items()
        },
        "joints": joint_summary,
    }
    env.close()
    return episode, joint_summary


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", type=Path, default=PROJECT_DIR / "results/v1_baseline/ppo_periodic_walk_v1.zip")
    parser.add_argument("--output", type=Path, default=REPO_ROOT / "outputs/periodic_gaits_research/diagnostics_v1")
    parser.add_argument("--seeds", type=int, nargs="+", default=list(range(30, 40)))
    parser.add_argument("--representative-seed", type=int, default=17)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    model = PPO.load(args.model, device="cpu")

    episodes = []
    all_joint_rows: dict[str, list[dict]] = {name: [] for name in JOINT_NAMES}
    for seed in args.seeds:
        episode, joints = rollout(model, seed, None, None)
        episodes.append(episode)
        for name in JOINT_NAMES:
            all_joint_rows[name].append(joints[name])
    representative, _ = rollout(
        model,
        args.representative_seed,
        args.output / f"rollout_seed_{args.representative_seed}.csv",
        args.output / f"rollout_seed_{args.representative_seed}.mp4",
    )

    aggregate_joints = {
        name: {
            key: float(np.mean([row[key] for row in rows]))
            for key in rows[0]
        }
        for name, rows in all_joint_rows.items()
    }
    summary = {
        "model": str(args.model.resolve()),
        "seeds": args.seeds,
        "aggregate": {
            "runs": len(episodes),
            "successes": sum(bool(row["success"]) for row in episodes),
            "falls": sum(bool(row["terminated"]) for row in episodes),
            "mean_forward_distance_m": float(np.mean([row["forward_distance_m"] for row in episodes])),
            "mean_total_reward": float(np.mean([row["total_reward"] for row in episodes])),
            "joints": aggregate_joints,
        },
        "representative": representative,
        "episodes": episodes,
    }
    (args.output / "diagnostics_summary.json").write_text(json.dumps(summary, indent=2))
    print(json.dumps(summary["aggregate"], indent=2), flush=True)


if __name__ == "__main__":
    main()
