#!/usr/bin/env python3
"""Compare the frozen V17 controller with the V19 landing corrector."""

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

from humanoidv2 import LandingResidualV19Env  # noqa: E402
from scripts.evaluate_dynamic_forward_v15 import annotate  # noqa: E402
from scripts.train_landing_residual_v19 import combined_domain  # noqa: E402


BASE_POLICY = ROOT / "results/khr3hv_v17_forward_margin/ppo_forward_margin_v17_final.zip"
INITIAL_CORRECTOR = (
    ROOT / "results/khr3hv_v19_landing_residual_joint/ppo_landing_residual_v19_initial.zip"
)
FINAL_CORRECTOR = (
    ROOT / "results/khr3hv_v19_landing_residual_joint/ppo_landing_residual_v19_final.zip"
)
OUTPUT = ROOT / "results/khr3hv_v19_landing_residual_joint"


def failed_criteria(info: dict[str, object], terminated: bool) -> list[str]:
    checks = {
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
    return [name for name, passed in checks.items() if not passed]


def run_episode(
    base_policy: PPO,
    corrector: PPO,
    first_side: str,
    seed: int,
    domain: dict[str, float | int],
    *,
    render: bool = False,
) -> tuple[dict[str, object], list[np.ndarray]]:
    env = LandingResidualV19Env(
        render_mode="rgb_array" if render else None,
        fixed_first_side=first_side,
        base_policy=base_policy,
        fixed_domain=domain,
    )
    if render:
        env.configure_render_camera(
            mode="world_fixed", lookat=(0.0, -0.12, 0.18),
            distance=1.8, azimuth=160.0, elevation=-10.0,
        )
    observation, _ = env.reset(seed=seed)
    frames: list[np.ndarray] = []
    if render:
        frames.append(annotate(env.render(), ("V19 landing residual", "t=0.0 s")))
    maximum_correction = maximum_torque = maximum_saturation = 0.0
    total_reward = 0.0
    terminated = truncated = False
    for step in range(env.config.max_episode_steps):
        action = corrector.predict(observation, deterministic=True)[0]
        observation, reward, terminated, truncated, info = env.step(action)
        total_reward += reward
        maximum_correction = max(maximum_correction, float(info["applied_correction_max_abs"]))
        maximum_torque = max(maximum_torque, float(info["peak_control_interval_torque_n_m"]))
        maximum_saturation = max(
            maximum_saturation, float(info["actuator_saturation_fraction"])
        )
        if render:
            frames.append(
                annotate(
                    env.render(),
                    (
                        "V19 landing residual | V17 base frozen",
                        f"t={(step + 1) * env.config.control_dt:.1f} s | "
                        f"forward={float(info['base_forward_displacement_m']):.3f} m | "
                        f"impact={float(info['maximum_landing_force_n']):.1f} N",
                    ),
                )
            )
        if terminated or truncated:
            break
    failures = failed_criteria(info, terminated)
    row: dict[str, object] = {
        "first_side": first_side,
        "seed": seed,
        **domain,
        "steps": step + 1,
        "duration_seconds": (step + 1) * env.config.control_dt,
        "terminated": terminated,
        "truncated": truncated,
        "success": bool(info["is_success"]),
        "failed_criteria": failures,
        "maximum_landing_force_n": float(info["maximum_landing_force_n"]),
        "impact_margin_n": 50.0 - float(info["maximum_landing_force_n"]),
        "base_forward_displacement_m": float(info["base_forward_displacement_m"]),
        "forward_margin_m": float(info["base_forward_displacement_m"]) - 0.200,
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
        "maximum_applied_correction_rad": maximum_correction,
        "maximum_actuator_torque_n_m": maximum_torque,
        "maximum_saturation_fraction": maximum_saturation,
        "total_reward": total_reward,
    }
    env.close()
    return row, frames


def aggregate(rows: list[dict[str, object]]) -> dict[str, object]:
    failures = Counter(
        reason for row in rows for reason in row["failed_criteria"]  # type: ignore[union-attr]
    )
    return {
        "runs": len(rows),
        "successes": sum(bool(row["success"]) for row in rows),
        "success_rate": sum(bool(row["success"]) for row in rows) / len(rows),
        "falls": sum(bool(row["terminated"]) for row in rows),
        "failure_counts": dict(sorted(failures.items())),
        "worst_landing_force_n": max(float(row["maximum_landing_force_n"]) for row in rows),
        "minimum_forward_displacement_m": min(
            float(row["base_forward_displacement_m"]) for row in rows
        ),
        "maximum_applied_correction_rad": max(
            float(row["maximum_applied_correction_rad"]) for row in rows
        ),
    }


def batch(base_policy: PPO, initial: PPO, final: PPO, output: Path) -> None:
    rows: list[dict[str, object]] = []
    for controller, corrector in (("v17_zero", initial), ("v19", final)):
        for sample, seed in enumerate(range(101, 121), start=1):
            domain = combined_domain(seed)
            for first_side in ("left", "right"):
                row, _ = run_episode(base_policy, corrector, first_side, seed, domain)
                row.update({"controller": controller, "sample": sample})
                rows.append(row)
            if sample % 5 == 0:
                print(f"{controller} samples completed={sample}/20", flush=True)
    summaries = {
        controller: aggregate([row for row in rows if row["controller"] == controller])
        for controller in ("v17_zero", "v19")
    }
    paired = []
    for sample in range(1, 21):
        item: dict[str, object] = {"sample": sample, "seed": 100 + sample}
        for controller in ("v17_zero", "v19"):
            selected = [
                row for row in rows
                if row["sample"] == sample and row["controller"] == controller
            ]
            item[f"{controller}_both_sides_success"] = all(row["success"] for row in selected)
        paired.append(item)
    summary = {
        "base_policy": str(BASE_POLICY),
        "initial_corrector": str(INITIAL_CORRECTOR),
        "final_corrector": str(FINAL_CORRECTOR),
        "stride_m": 0.042,
        "step_cycle_seconds": 2.56,
        "minimum_forward_m": 0.200,
        "impact_limit_n": 50.0,
        "landing_correction_limit_rad": 0.025,
        "controllers": summaries,
        "v17_both_sides_successful_samples": sum(
            item["v17_zero_both_sides_success"] for item in paired
        ),
        "v19_both_sides_successful_samples": sum(
            item["v19_both_sides_success"] for item in paired
        ),
        "paired": paired,
    }
    output.mkdir(parents=True, exist_ok=True)
    (output / "robustness_comparison_results.json").write_text(
        json.dumps(rows, indent=2, allow_nan=False), encoding="utf-8"
    )
    (output / "robustness_comparison_summary.json").write_text(
        json.dumps(summary, indent=2, allow_nan=False), encoding="utf-8"
    )
    print(json.dumps(summary, indent=2, allow_nan=False))


def video(base_policy: PPO, corrector: PPO, output: Path, args: argparse.Namespace) -> None:
    domain = combined_domain(args.seed) if args.combined else {}
    row, frames = run_episode(
        base_policy, corrector, args.first_side, args.seed, domain, render=True
    )
    label = args.label or f"v19_{args.first_side}_seed{args.seed}"
    output.mkdir(parents=True, exist_ok=True)
    path = output / f"{label}.mp4"
    imageio.mimsave(path, frames, fps=25, macro_block_size=16)
    imageio.imwrite(output / f"{label}_final.png", frames[-1])
    row["video"] = str(path)
    (output / f"{label}_summary.json").write_text(
        json.dumps(row, indent=2, allow_nan=False), encoding="utf-8"
    )
    print(json.dumps(row, indent=2, allow_nan=False))


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--base-policy", type=Path, default=BASE_POLICY)
    parser.add_argument("--initial-corrector", type=Path, default=INITIAL_CORRECTOR)
    parser.add_argument("--corrector", type=Path, default=FINAL_CORRECTOR)
    parser.add_argument("--output", type=Path, default=OUTPUT)
    parser.add_argument("--video", action="store_true")
    parser.add_argument("--combined", action="store_true")
    parser.add_argument("--label")
    parser.add_argument("--first-side", choices=("left", "right"), default="left")
    parser.add_argument("--seed", type=int, default=17)
    args = parser.parse_args()
    base_policy = PPO.load(args.base_policy, device="cpu")
    final = PPO.load(args.corrector, device="cpu")
    if args.video:
        video(base_policy, final, args.output, args)
    else:
        initial = PPO.load(args.initial_corrector, device="cpu")
        batch(base_policy, initial, final, args.output)


if __name__ == "__main__":
    main()
