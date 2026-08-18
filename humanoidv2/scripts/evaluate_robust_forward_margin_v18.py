#!/usr/bin/env python3
"""Map V17 policy robustness in the V18 perturbed environment."""

from __future__ import annotations

import argparse
import json
import os
import sys
from collections import Counter
from pathlib import Path

os.environ.setdefault("MUJOCO_GL", "egl")

import imageio.v2 as imageio
import numpy as np
from stable_baselines3 import PPO

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from humanoidv2 import RobustForwardMarginV18Env  # noqa: E402
from scripts.evaluate_dynamic_forward_v15 import annotate  # noqa: E402


POLICY_PATH = ROOT / "results/khr3hv_v17_forward_margin/ppo_forward_margin_v17_final.zip"
OUTPUT_DIR = ROOT / "results/khr3hv_v18_robustness"
SEEDS = (7, 17, 29, 43, 61)
AXES: dict[str, tuple[float | int, ...]] = {
    "friction_scale": (0.70, 0.80, 0.90, 1.00, 1.10, 1.20, 1.30),
    "mass_scale": (0.95, 0.98, 1.00, 1.02, 1.05),
    "leg_mass_asymmetry": (-0.03, -0.02, -0.01, 0.00, 0.01, 0.02, 0.03),
    "motor_gain_scale": (0.90, 0.95, 1.00, 1.05, 1.10),
    "control_delay_steps": (0, 1, 2),
    "initial_joint_noise_rad": (0.00175, 0.00350, 0.00525),
    "initial_joint_velocity_noise_rad_s": (0.005, 0.010, 0.015),
}
BOUNDARY_AXES: dict[str, tuple[float, ...]] = {
    "friction_scale": (0.75, 0.80, 0.85, 0.90, 0.95, 1.00, 1.05, 1.10, 1.15),
    "mass_scale": (0.96, 0.97, 0.98, 0.99, 1.00, 1.01, 1.02),
    "leg_mass_asymmetry": (
        -0.015, -0.0125, -0.010, -0.0075, -0.005, 0.0,
        0.005, 0.0075, 0.010, 0.0125, 0.015,
    ),
    "motor_gain_scale": (0.96, 0.97, 0.98, 0.99, 1.00, 1.01, 1.02, 1.03, 1.04),
}


def criteria(info: dict[str, object], terminated: bool) -> dict[str, bool]:
    return {
        "not_fallen": not terminated,
        "impact": bool(info["impact_limit_satisfied"]),
        "forward": float(info["base_forward_displacement_m"]) >= 0.200,
        "swing_unload": min(
            float(info[f"step_{index}_advance_under_5n_fraction"])
            for index in range(1, 9)
        ) >= 0.9,
        "double_support": min(
            float(info[f"step_{index}_both_contact_fraction"])
            for index in range(1, 9)
        ) >= 0.9,
        "step_length": min(
            float(info[f"step_{index}_length_m"]) for index in range(1, 9)
        ) >= 0.020,
    }


def run_episode(
    policy: PPO,
    first_side: str,
    seed: int,
    parameters: dict[str, float | int],
    *,
    render: bool = False,
    camera_azimuth: float = 160.0,
) -> tuple[dict[str, object], list[np.ndarray]]:
    env = RobustForwardMarginV18Env(
        render_mode="rgb_array" if render else None,
        fixed_first_side=first_side,
        **parameters,
    )
    if render:
        env.configure_render_camera(
            mode="world_fixed",
            lookat=(0.0, -0.12, 0.18),
            distance=1.8,
            azimuth=camera_azimuth,
            elevation=-10.0,
        )
    observation, _ = env.reset(seed=seed)
    frames: list[np.ndarray] = []
    if render:
        frames.append(
            annotate(
                env.render(),
                ("V18 robustness | V17 policy fixed", "t=0.0 s | forward=0.000 m"),
            )
        )
    total_reward = 0.0
    terminated = truncated = False
    maximum_torque = 0.0
    maximum_saturation = 0.0
    for step in range(env.config.max_episode_steps):
        action = policy.predict(observation, deterministic=True)[0]
        observation, reward, terminated, truncated, info = env.step(action)
        total_reward += reward
        maximum_torque = max(maximum_torque, float(info["peak_control_interval_torque_n_m"]))
        maximum_saturation = max(
            maximum_saturation, float(info["actuator_saturation_fraction"])
        )
        if render:
            frames.append(
                annotate(
                    env.render(),
                    (
                        "V18 robustness | V17 policy fixed",
                        f"t={(step + 1) * env.config.control_dt:.1f} s | "
                        f"forward={float(info['base_forward_displacement_m']):.3f} m | "
                        f"impact={float(info['maximum_landing_force_n']):.1f} N",
                    ),
                )
            )
        if terminated or truncated:
            break

    checks = criteria(info, terminated)
    failed = [name for name, passed in checks.items() if not passed]
    row: dict[str, object] = {
        "first_side": first_side,
        "seed": seed,
        **parameters,
        "steps": step + 1,
        "duration_seconds": (step + 1) * env.config.control_dt,
        "terminated": terminated,
        "truncated": truncated,
        "success": bool(info["is_success"]),
        "failed_criteria": failed,
        "primary_failure": failed[0] if failed else None,
        "maximum_landing_force_n": float(info["maximum_landing_force_n"]),
        "base_forward_displacement_m": float(info["base_forward_displacement_m"]),
        "forward_margin_m": float(info["base_forward_displacement_m"]) - 0.200,
        "impact_margin_n": 50.0 - float(info["maximum_landing_force_n"]),
        "minimum_step_length_m": min(
            float(info[f"step_{index}_length_m"]) for index in range(1, 9)
        ),
        "minimum_advance_under_5n_fraction": min(
            float(info[f"step_{index}_advance_under_5n_fraction"])
            for index in range(1, 9)
        ),
        "minimum_both_contact_fraction": min(
            float(info[f"step_{index}_both_contact_fraction"])
            for index in range(1, 9)
        ),
        "maximum_actuator_torque_n_m": maximum_torque,
        "maximum_saturation_fraction": maximum_saturation,
        "total_reward": total_reward,
    }
    env.close()
    return row, frames


def aggregate(rows: list[dict[str, object]]) -> dict[str, object]:
    successes = [row for row in rows if row["success"]]
    failure_counts = Counter(
        reason for row in rows for reason in row["failed_criteria"]  # type: ignore[union-attr]
    )
    return {
        "runs": len(rows),
        "successes": len(successes),
        "success_rate": len(successes) / len(rows),
        "failure_counts": dict(sorted(failure_counts.items())),
        "worst_landing_force_n": max(float(row["maximum_landing_force_n"]) for row in rows),
        "minimum_forward_displacement_m": min(
            float(row["base_forward_displacement_m"]) for row in rows
        ),
        "minimum_impact_margin_n": min(float(row["impact_margin_n"]) for row in rows),
        "minimum_forward_margin_m": min(float(row["forward_margin_m"]) for row in rows),
    }


def sweep(policy: PPO, output: Path) -> None:
    rows: list[dict[str, object]] = []
    total = sum(
        len(values) * 2 * (len(SEEDS) if "noise" in axis else 1)
        for axis, values in AXES.items()
    )
    completed = 0
    for axis, values in AXES.items():
        seeds = SEEDS if "noise" in axis else (17,)
        for value in values:
            for seed in seeds:
                for first_side in ("left", "right"):
                    row, _ = run_episode(policy, first_side, seed, {axis: value})
                    row.update({"experiment": "single_factor", "axis": axis, "value": value})
                    rows.append(row)
                    completed += 1
                    if completed % 10 == 0 or completed == total:
                        print(f"single-factor completed={completed}/{total}", flush=True)

    aggregates: list[dict[str, object]] = []
    for axis, values in AXES.items():
        for value in values:
            selected = [row for row in rows if row["axis"] == axis and row["value"] == value]
            aggregates.append({"axis": axis, "value": value, **aggregate(selected)})

    output.mkdir(parents=True, exist_ok=True)
    summary = {
        "policy": str(POLICY_PATH),
        "policy_changed": False,
        "preserved_v17_targets": {
            "stride_m": 0.042,
            "minimum_forward_m": 0.200,
            "impact_limit_n": 50.0,
            "step_cycle_seconds": 2.56,
        },
        "seeded_noise_runs_per_side": len(SEEDS),
        "total_runs": len(rows),
        "overall": aggregate(rows),
        "aggregates": aggregates,
    }
    (output / "single_factor_results.json").write_text(
        json.dumps(rows, indent=2, allow_nan=False), encoding="utf-8"
    )
    (output / "single_factor_summary.json").write_text(
        json.dumps(summary, indent=2, allow_nan=False), encoding="utf-8"
    )
    print(json.dumps(summary, indent=2, allow_nan=False))


def boundary_sweep(policy: PPO, output: Path) -> None:
    rows: list[dict[str, object]] = []
    total = sum(len(values) * 2 for values in BOUNDARY_AXES.values())
    completed = 0
    for axis, values in BOUNDARY_AXES.items():
        for value in values:
            for first_side in ("left", "right"):
                row, _ = run_episode(policy, first_side, 17, {axis: value})
                row.update({"experiment": "boundary", "axis": axis, "value": value})
                rows.append(row)
                completed += 1
                if completed % 10 == 0 or completed == total:
                    print(f"boundary completed={completed}/{total}", flush=True)

    aggregates = []
    for axis, values in BOUNDARY_AXES.items():
        for value in values:
            selected = [row for row in rows if row["axis"] == axis and row["value"] == value]
            aggregates.append({"axis": axis, "value": value, **aggregate(selected)})
    summary = {
        "policy": str(POLICY_PATH),
        "policy_changed": False,
        "total_runs": len(rows),
        "overall": aggregate(rows),
        "aggregates": aggregates,
    }
    output.mkdir(parents=True, exist_ok=True)
    (output / "boundary_results.json").write_text(
        json.dumps(rows, indent=2, allow_nan=False), encoding="utf-8"
    )
    (output / "boundary_summary.json").write_text(
        json.dumps(summary, indent=2, allow_nan=False), encoding="utf-8"
    )
    print(json.dumps(summary, indent=2, allow_nan=False))


def combined_sweep(policy: PPO, output: Path) -> None:
    """Evaluate the planned V18 design box with all errors active together."""
    rows: list[dict[str, object]] = []
    combined_seeds = tuple(range(101, 121))
    for index, seed in enumerate(combined_seeds, start=1):
        generator = np.random.default_rng(seed)
        parameters: dict[str, float | int] = {
            "friction_scale": float(generator.uniform(0.80, 1.20)),
            "mass_scale": float(generator.uniform(0.98, 1.02)),
            "leg_mass_asymmetry": float(generator.uniform(-0.01, 0.01)),
            "initial_joint_noise_rad": 0.0035,
            "initial_joint_velocity_noise_rad_s": 0.010,
            "motor_gain_scale": float(generator.uniform(0.95, 1.05)),
            "control_delay_steps": int(generator.integers(0, 3)),
        }
        for first_side in ("left", "right"):
            row, _ = run_episode(policy, first_side, seed, parameters)
            row.update({"experiment": "combined_design_box", "sample": index})
            rows.append(row)
        if index % 5 == 0 or index == len(combined_seeds):
            print(f"combined samples completed={index}/{len(combined_seeds)}", flush=True)

    paired = []
    for index in range(1, len(combined_seeds) + 1):
        selected = [row for row in rows if row["sample"] == index]
        paired.append(
            {
                "sample": index,
                "seed": selected[0]["seed"],
                "both_sides_success": all(row["success"] for row in selected),
                "parameters": {
                    key: selected[0][key]
                    for key in (
                        "friction_scale", "mass_scale", "leg_mass_asymmetry",
                        "initial_joint_noise_rad", "initial_joint_velocity_noise_rad_s",
                        "motor_gain_scale", "control_delay_steps",
                    )
                },
                **aggregate(selected),
            }
        )
    summary = {
        "policy": str(POLICY_PATH),
        "policy_changed": False,
        "design_box": {
            "friction_scale": [0.80, 1.20],
            "mass_scale": [0.98, 1.02],
            "leg_mass_asymmetry": [-0.01, 0.01],
            "initial_joint_noise_rad": 0.0035,
            "initial_joint_velocity_noise_rad_s": 0.010,
            "motor_gain_scale": [0.95, 1.05],
            "control_delay_steps": [0, 2],
        },
        "total_runs": len(rows),
        "overall": aggregate(rows),
        "paired_samples": len(paired),
        "both_sides_successful_samples": sum(item["both_sides_success"] for item in paired),
        "paired": paired,
    }
    output.mkdir(parents=True, exist_ok=True)
    (output / "combined_results.json").write_text(
        json.dumps(rows, indent=2, allow_nan=False), encoding="utf-8"
    )
    (output / "combined_summary.json").write_text(
        json.dumps(summary, indent=2, allow_nan=False), encoding="utf-8"
    )
    print(json.dumps(summary, indent=2, allow_nan=False))


def video(policy: PPO, output: Path, args: argparse.Namespace) -> None:
    parameters: dict[str, float | int] = {
        "friction_scale": args.friction_scale,
        "mass_scale": args.mass_scale,
        "leg_mass_asymmetry": args.leg_mass_asymmetry,
        "initial_joint_noise_rad": args.initial_joint_noise_rad,
        "initial_joint_velocity_noise_rad_s": args.initial_joint_velocity_noise_rad_s,
        "motor_gain_scale": args.motor_gain_scale,
        "control_delay_steps": args.control_delay_steps,
    }
    row, frames = run_episode(
        policy,
        args.first_side,
        args.seed,
        parameters,
        render=True,
        camera_azimuth=args.camera_azimuth,
    )
    label = args.label or f"v18_{args.first_side}_seed{args.seed}"
    output.mkdir(parents=True, exist_ok=True)
    video_path = output / f"{label}.mp4"
    imageio.mimsave(video_path, frames, fps=25, macro_block_size=16)
    imageio.imwrite(output / f"{label}_final.png", frames[-1])
    row["video"] = str(video_path)
    row["camera_azimuth"] = args.camera_azimuth
    (output / f"{label}_summary.json").write_text(
        json.dumps(row, indent=2, allow_nan=False), encoding="utf-8"
    )
    print(json.dumps(row, indent=2, allow_nan=False))


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", type=Path, default=POLICY_PATH)
    parser.add_argument("--output", type=Path, default=OUTPUT_DIR)
    parser.add_argument("--video", action="store_true")
    parser.add_argument("--boundary", action="store_true")
    parser.add_argument("--combined", action="store_true")
    parser.add_argument("--label")
    parser.add_argument("--first-side", choices=("left", "right"), default="left")
    parser.add_argument("--seed", type=int, default=17)
    parser.add_argument("--friction-scale", type=float, default=1.0)
    parser.add_argument("--mass-scale", type=float, default=1.0)
    parser.add_argument("--leg-mass-asymmetry", type=float, default=0.0)
    parser.add_argument("--initial-joint-noise-rad", type=float, default=0.0)
    parser.add_argument("--initial-joint-velocity-noise-rad-s", type=float, default=0.0)
    parser.add_argument("--motor-gain-scale", type=float, default=1.0)
    parser.add_argument("--control-delay-steps", type=int, default=0)
    parser.add_argument("--camera-azimuth", type=float, default=160.0)
    args = parser.parse_args()
    policy = PPO.load(args.model, device="cpu")
    selected_modes = sum((args.video, args.boundary, args.combined))
    if selected_modes > 1:
        parser.error("choose at most one of --video, --boundary, and --combined")
    if args.video:
        video(policy, args.output, args)
    elif args.boundary:
        boundary_sweep(policy, args.output)
    elif args.combined:
        combined_sweep(policy, args.output)
    else:
        sweep(policy, args.output)


if __name__ == "__main__":
    main()
