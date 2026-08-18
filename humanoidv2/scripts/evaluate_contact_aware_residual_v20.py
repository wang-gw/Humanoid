#!/usr/bin/env python3
"""Evaluate calibrated V20 and compare it with saved V17/V19 results."""

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

from humanoidv2 import ContactAwareResidualV20Env  # noqa: E402
from scripts.evaluate_dynamic_forward_v15 import annotate  # noqa: E402
from scripts.train_landing_residual_v19 import combined_domain  # noqa: E402


BASE = ROOT / "results/khr3hv_v17_forward_margin/ppo_forward_margin_v17_final.zip"
CORRECTOR = ROOT / "results/khr3hv_v20_contact_aware/ppo_contact_aware_v20_final.zip"
OUTPUT = ROOT / "results/khr3hv_v20_contact_aware"
V19_RESULTS = (
    ROOT / "results/khr3hv_v19_landing_residual_joint/robustness_comparison_summary.json"
)
EARLY_GATE_BLEND = 0.75


def criteria(info: dict[str, object], terminated: bool) -> list[str]:
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


def run_episode(base, corrector, side, seed, domain, render=False):
    env = ContactAwareResidualV20Env(
        render_mode="rgb_array" if render else None,
        base_policy=base,
        fixed_first_side=side,
        fixed_domain=domain,
        curriculum_stage=2,
        early_gate_blend=EARLY_GATE_BLEND,
    )
    if render:
        env.configure_render_camera(
            mode="world_fixed", lookat=(0.0, -0.12, 0.18),
            distance=1.8, azimuth=160.0, elevation=-10.0,
        )
    observation, _ = env.reset(seed=seed)
    frames = []
    if render:
        frames.append(annotate(env.render(), ("V20 contact-aware residual", "t=0.0 s")))
    maximum_correction = maximum_torque = maximum_saturation = 0.0
    terminated = truncated = False
    for step in range(env.config.max_episode_steps):
        action = corrector.predict(observation, deterministic=True)[0]
        observation, _, terminated, truncated, info = env.step(action)
        maximum_correction = max(
            maximum_correction, float(info["applied_correction_max_abs"])
        )
        maximum_torque = max(maximum_torque, float(info["peak_control_interval_torque_n_m"]))
        maximum_saturation = max(
            maximum_saturation, float(info["actuator_saturation_fraction"])
        )
        if render:
            frames.append(
                annotate(
                    env.render(),
                    (
                        "V20 contact-aware | V17 base frozen",
                        f"t={(step + 1) * env.config.control_dt:.1f} s | "
                        f"forward={float(info['base_forward_displacement_m']):.3f} m | "
                        f"impact={float(info['maximum_landing_force_n']):.1f} N",
                    ),
                )
            )
        if terminated or truncated:
            break
    failures = criteria(info, terminated)
    row = {
        "first_side": side,
        "seed": seed,
        **domain,
        "steps": step + 1,
        "duration_seconds": (step + 1) * env.config.control_dt,
        "terminated": terminated,
        "truncated": truncated,
        "success": bool(info["is_success"]),
        "failed_criteria": failures,
        "maximum_landing_force_n": float(info["maximum_landing_force_n"]),
        "base_forward_displacement_m": float(info["base_forward_displacement_m"]),
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
    }
    env.close()
    return row, frames


def aggregate(rows):
    counts = Counter(reason for row in rows for reason in row["failed_criteria"])
    return {
        "runs": len(rows),
        "successes": sum(row["success"] for row in rows),
        "success_rate": sum(row["success"] for row in rows) / len(rows),
        "falls": sum(row["terminated"] for row in rows),
        "failure_counts": dict(sorted(counts.items())),
        "worst_landing_force_n": max(row["maximum_landing_force_n"] for row in rows),
        "minimum_forward_displacement_m": min(
            row["base_forward_displacement_m"] for row in rows
        ),
        "maximum_applied_correction_rad": max(
            row["maximum_applied_correction_rad"] for row in rows
        ),
    }


def batch(base, corrector, output):
    rows = []
    nominal = []
    for side in ("left", "right"):
        row, _ = run_episode(base, corrector, side, 17, {})
        row["case"] = "nominal"
        nominal.append(row)
    for sample, seed in enumerate(range(101, 121), start=1):
        for side in ("left", "right"):
            row, _ = run_episode(base, corrector, side, seed, combined_domain(seed))
            row.update({"case": "combined", "sample": sample})
            rows.append(row)
        if sample % 5 == 0:
            print(f"V20 samples completed={sample}/20", flush=True)
    v19 = json.loads(V19_RESULTS.read_text(encoding="utf-8"))
    v20 = aggregate(rows)
    paired = sum(
        all(row["success"] for row in rows if row["sample"] == sample)
        for sample in range(1, 21)
    )
    summary = {
        "base_policy": str(BASE),
        "v20_corrector": str(CORRECTOR),
        "early_gate_blend": EARLY_GATE_BLEND,
        "stride_m": 0.042,
        "step_cycle_seconds": 2.56,
        "minimum_forward_m": 0.200,
        "impact_limit_n": 50.0,
        "controllers": {
            "v17": v19["controllers"]["v17_zero"],
            "v19": v19["controllers"]["v19"],
            "v20": v20,
        },
        "both_sides_successful_samples": {
            "v17": v19["v17_both_sides_successful_samples"],
            "v19": v19["v19_both_sides_successful_samples"],
            "v20": paired,
        },
        "nominal": nominal,
    }
    output.mkdir(parents=True, exist_ok=True)
    (output / "robustness_results.json").write_text(
        json.dumps(rows, indent=2, allow_nan=False), encoding="utf-8"
    )
    (output / "robustness_summary.json").write_text(
        json.dumps(summary, indent=2, allow_nan=False), encoding="utf-8"
    )
    print(json.dumps(summary, indent=2, allow_nan=False))


def video(base, corrector, output, args):
    domain = combined_domain(args.seed) if args.combined else {}
    row, frames = run_episode(
        base, corrector, args.first_side, args.seed, domain, render=True
    )
    label = args.label or f"v20_{args.first_side}_seed{args.seed}"
    output.mkdir(parents=True, exist_ok=True)
    path = output / f"{label}.mp4"
    imageio.mimsave(path, frames, fps=25, macro_block_size=16)
    imageio.imwrite(output / f"{label}_final.png", frames[-1])
    row["video"] = str(path)
    (output / f"{label}_summary.json").write_text(
        json.dumps(row, indent=2, allow_nan=False), encoding="utf-8"
    )
    print(json.dumps(row, indent=2, allow_nan=False))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--base-policy", type=Path, default=BASE)
    parser.add_argument("--corrector", type=Path, default=CORRECTOR)
    parser.add_argument("--output", type=Path, default=OUTPUT)
    parser.add_argument("--video", action="store_true")
    parser.add_argument("--combined", action="store_true")
    parser.add_argument("--seed", type=int, default=17)
    parser.add_argument("--first-side", choices=("left", "right"), default="left")
    parser.add_argument("--label")
    args = parser.parse_args()
    base = PPO.load(args.base_policy, device="cpu")
    corrector = PPO.load(args.corrector, device="cpu")
    if args.video:
        video(base, corrector, args.output, args)
    else:
        batch(base, corrector, args.output)


if __name__ == "__main__":
    main()
