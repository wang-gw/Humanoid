#!/usr/bin/env python3
"""Evaluate V10 policy under V11 physics and reset perturbations."""

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

from humanoidv2 import RobustSoftLandingV11Env  # noqa: E402


MILD_NOISE = {"initial_joint_noise_rad": 0.0035, "initial_joint_velocity_noise_rad_s": 0.01}
MODERATE_NOISE = {"initial_joint_noise_rad": 0.0087, "initial_joint_velocity_noise_rad_s": 0.02}
SCENARIOS: dict[str, dict[str, float]] = {
    "nominal": {},
    "initial_mild": MILD_NOISE,
    "initial_moderate": MODERATE_NOISE,
    "friction_low": {"friction_scale": 0.7, **MILD_NOISE},
    "friction_high": {"friction_scale": 1.3, **MILD_NOISE},
    "mass_light": {"mass_scale": 0.95, **MILD_NOISE},
    "mass_heavy": {"mass_scale": 1.05, **MILD_NOISE},
    "left_heavy": {"leg_mass_asymmetry": 0.03, **MILD_NOISE},
    "right_heavy": {"leg_mass_asymmetry": -0.03, **MILD_NOISE},
    "combined_left_heavy": {
        "friction_scale": 0.7,
        "mass_scale": 1.05,
        "leg_mass_asymmetry": 0.03,
        **MODERATE_NOISE,
    },
    "combined_right_heavy": {
        "friction_scale": 0.7,
        "mass_scale": 1.05,
        "leg_mass_asymmetry": -0.03,
        **MODERATE_NOISE,
    },
}
DEFAULT_SEEDS = (7, 17, 29, 43, 61)


def failure_reason(info: dict[str, object], terminated: bool) -> str | None:
    if terminated:
        return "fall"
    if not bool(info["impact_limit_satisfied"]):
        return "impact_limit"
    if float(info["base_forward_displacement_m"]) < 0.180:
        return "forward_distance"
    under = min(float(info[f"step_{index}_advance_under_5n_fraction"]) for index in range(1, 9))
    if under < 0.9:
        return "swing_unload"
    contact = min(float(info[f"step_{index}_both_contact_fraction"]) for index in range(1, 9))
    if contact < 0.9:
        return "double_support"
    length = min(float(info[f"step_{index}_length_m"]) for index in range(1, 9))
    if length < 0.020:
        return "step_length"
    return None if bool(info["is_success"]) else "other"


def run_episode(
    policy: PPO,
    scenario: str,
    first_side: str,
    seed: int,
    render: bool = False,
    params_override: dict[str, float] | None = None,
) -> tuple[dict[str, object], list[np.ndarray]]:
    params = SCENARIOS[scenario] if params_override is None else params_override
    env = RobustSoftLandingV11Env(
        render_mode="rgb_array" if render else None,
        fixed_first_side=first_side,
        **params,
    )
    observation, _ = env.reset(seed=seed)
    frames = [env.render()] if render else []
    total_reward = 0.0
    terminated = truncated = False
    for step in range(env.config.max_episode_steps):
        action = policy.predict(observation, deterministic=True)[0]
        observation, reward, terminated, truncated, info = env.step(action)
        total_reward += reward
        if render:
            frames.append(env.render())
        if terminated or truncated:
            break
    row: dict[str, object] = {
        "scenario": scenario,
        "first_side": first_side,
        "seed": seed,
        **params,
        "steps": step + 1,
        "duration_seconds": (step + 1) * env.config.control_dt,
        "terminated": terminated,
        "truncated": truncated,
        "gait_success": bool(info["gait_success"]),
        "impact_limit_satisfied": bool(info["impact_limit_satisfied"]),
        "success": bool(info["is_success"]),
        "failure_reason": failure_reason(info, terminated),
        "maximum_landing_force_n": float(info["maximum_landing_force_n"]),
        "base_forward_displacement_m": float(info["base_forward_displacement_m"]),
        "minimum_step_length_m": min(
            float(info[f"step_{index}_length_m"]) for index in range(1, 9)
        ),
        "minimum_advance_under_5n_fraction": min(
            float(info[f"step_{index}_advance_under_5n_fraction"]) for index in range(1, 9)
        ),
        "minimum_both_contact_fraction": min(
            float(info[f"step_{index}_both_contact_fraction"]) for index in range(1, 9)
        ),
        "total_reward": total_reward,
    }
    env.close()
    return row, frames


def scenario_seeds(name: str) -> tuple[int, ...]:
    if name == "nominal":
        return (7,)
    if name in {"initial_mild", "initial_moderate", "combined_left_heavy", "combined_right_heavy"}:
        return DEFAULT_SEEDS
    return DEFAULT_SEEDS[:3]


def batch(policy: PPO, output: Path) -> None:
    rows = []
    total = sum(len(scenario_seeds(name)) * 2 for name in SCENARIOS)
    completed = 0
    for scenario in SCENARIOS:
        for seed in scenario_seeds(scenario):
            for first_side in ("left", "right"):
                row, _ = run_episode(policy, scenario, first_side, seed)
                rows.append(row)
                completed += 1
                if completed % 10 == 0 or completed == total:
                    print(f"completed={completed}/{total}", flush=True)

    aggregates = []
    for scenario in SCENARIOS:
        selected = [row for row in rows if row["scenario"] == scenario]
        successes = [row for row in selected if row["success"]]
        aggregates.append(
            {
                "scenario": scenario,
                "parameters": SCENARIOS[scenario],
                "runs": len(selected),
                "successes": len(successes),
                "success_rate": len(successes) / len(selected),
                "falls": sum(row["terminated"] for row in selected),
                "impact_failures": sum(row["failure_reason"] == "impact_limit" for row in selected),
                "worst_landing_force_n": max(row["maximum_landing_force_n"] for row in selected),
                "minimum_forward_displacement_m": min(
                    row["base_forward_displacement_m"] for row in selected
                ),
            }
        )
    successes = [row for row in rows if row["success"]]
    failures = [row for row in rows if not row["success"]]
    summary = {
        "policy": str(ROOT / "results/khr3hv_v10_symmetric/ppo_soft_landing_final.zip"),
        "impact_limit_n": 50.0,
        "total_runs": len(rows),
        "successes": len(successes),
        "success_rate": len(successes) / len(rows),
        "scenario_aggregates": aggregates,
        "worst_success": max(successes, key=lambda row: row["maximum_landing_force_n"])
        if successes
        else None,
        "first_failure": failures[0] if failures else None,
        "failure_counts": {
            reason: sum(row["failure_reason"] == reason for row in failures)
            for reason in sorted({str(row["failure_reason"]) for row in failures})
        },
    }
    output.mkdir(parents=True, exist_ok=True)
    (output / "robustness_results.json").write_text(
        json.dumps(rows, indent=2, allow_nan=False), encoding="utf-8"
    )
    (output / "robustness_summary.json").write_text(
        json.dumps(summary, indent=2, allow_nan=False), encoding="utf-8"
    )
    print(json.dumps(summary, indent=2, allow_nan=False))


def video(policy: PPO, output: Path, scenario: str, first_side: str, seed: int) -> None:
    row, frames = run_episode(policy, scenario, first_side, seed, render=True)
    label = f"robust_{scenario}_{first_side}_seed{seed}"
    output.mkdir(parents=True, exist_ok=True)
    video_path = output / f"{label}.mp4"
    imageio.mimsave(video_path, frames, fps=25, macro_block_size=16)
    imageio.imwrite(output / f"{label}_final.png", frames[-1])
    row["video"] = str(video_path)
    (output / f"{label}_summary.json").write_text(
        json.dumps(row, indent=2, allow_nan=False), encoding="utf-8"
    )
    print(json.dumps(row, indent=2, allow_nan=False))


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--model",
        type=Path,
        default=ROOT / "results/khr3hv_v10_symmetric/ppo_soft_landing_final.zip",
    )
    parser.add_argument("--output", type=Path, default=ROOT / "results/khr3hv_v11_robustness")
    parser.add_argument("--video-scenario", choices=tuple(SCENARIOS))
    parser.add_argument("--first-side", choices=("left", "right"), default="left")
    parser.add_argument("--seed", type=int, default=7)
    args = parser.parse_args()
    policy = PPO.load(args.model, device="cpu")
    if args.video_scenario:
        video(policy, args.output, args.video_scenario, args.first_side, args.seed)
    else:
        batch(policy, args.output)


if __name__ == "__main__":
    main()
