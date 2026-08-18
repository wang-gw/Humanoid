#!/usr/bin/env python3
"""Evaluate V10 soft-landing walking and save video plus metrics."""

from __future__ import annotations

import argparse
import json
import os
import sys
from pathlib import Path

os.environ.setdefault("MUJOCO_GL", "egl")

import imageio.v2 as imageio
import numpy as np
from PIL import Image, ImageDraw, ImageFont
from stable_baselines3 import PPO

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from humanoidv2 import SoftLandingEightStepV10Env  # noqa: E402


def annotate_frame(frame: np.ndarray, text: str) -> np.ndarray:
    image = Image.fromarray(frame)
    draw = ImageDraw.Draw(image)
    font = ImageFont.load_default(size=18)
    bounds = draw.textbbox((0, 0), text, font=font)
    draw.rectangle((8, 8, bounds[2] + 24, 38), fill=(0, 0, 0))
    draw.text((16, 12), text, fill=(255, 255, 255), font=font)
    return np.asarray(image)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--first-side", choices=("left", "right"), required=True)
    parser.add_argument(
        "--model",
        type=Path,
        default=ROOT / "results/khr3hv_v10_symmetric/ppo_soft_landing_final.zip",
    )
    parser.add_argument("--output", type=Path, default=ROOT / "results/khr3hv_v10_symmetric")
    parser.add_argument("--reference-only", action="store_true")
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--camera-mode", choices=("tracking", "world_fixed"), default="tracking")
    parser.add_argument("--camera-azimuth", type=float, default=135.0)
    parser.add_argument("--camera-elevation", type=float, default=-12.0)
    parser.add_argument("--camera-distance", type=float, default=1.25)
    parser.add_argument("--camera-lookat", type=float, nargs=3, metavar=("X", "Y", "Z"), default=(0.0, 0.10, 0.18))
    parser.add_argument("--show-progress", action="store_true")
    parser.add_argument("--label-suffix", default="")
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    mode = "reference" if args.reference_only else "trained"
    suffix = f"_{args.label_suffix}" if args.label_suffix else ""
    label = f"soft_landing_{args.first_side}_first_{mode}{suffix}"
    policy = None if args.reference_only else PPO.load(args.model, device="cpu")
    env = SoftLandingEightStepV10Env(
        render_mode="rgb_array",
        reference_only=args.reference_only,
        fixed_first_side=args.first_side,
    )
    env.configure_render_camera(
        mode=args.camera_mode,
        lookat=tuple(args.camera_lookat),
        distance=args.camera_distance,
        azimuth=args.camera_azimuth,
        elevation=args.camera_elevation,
    )
    observation, _ = env.reset(seed=args.seed)
    initial_frame = env.render()
    if args.show_progress:
        initial_frame = annotate_frame(initial_frame, "world fixed | t=0.0 s | base forward=0.000 m")
    frames = [initial_frame]
    records = []
    total_reward = 0.0
    terminated = truncated = False
    for step in range(env.config.max_episode_steps):
        action = (
            np.zeros(10, dtype=np.float32)
            if policy is None
            else policy.predict(observation, deterministic=True)[0]
        )
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
        frame = env.render()
        if args.show_progress:
            frame = annotate_frame(
                frame,
                f"world fixed | t={(step + 1) * env.config.control_dt:.1f} s | "
                f"base forward={float(info['base_forward_displacement_m']):.3f} m",
            )
        frames.append(frame)
        if terminated or truncated:
            break
    env.close()

    video_path = args.output / f"{label}.mp4"
    imageio.mimsave(video_path, frames, fps=25, macro_block_size=16)
    imageio.imwrite(args.output / f"{label}_first.png", frames[0])
    imageio.imwrite(args.output / f"{label}_step4_landing.png", frames[min(585, len(frames) - 1)])
    imageio.imwrite(args.output / f"{label}_step8_landing.png", frames[min(1185, len(frames) - 1)])
    imageio.imwrite(args.output / f"{label}_final.png", frames[-1])
    final = [row for row in records if row["task_phase"] == "step8_settle"]
    summary: dict[str, object] = {
        "order": records[-1]["step_order"],
        "mode": mode,
        "seconds_per_step": 6.0,
        "phase_seconds": {"lift": 2.4, "advance": 1.2, "land": 2.0, "settle": 0.4},
        "model": None if policy is None else str(args.model),
        "steps": len(records),
        "duration_seconds": len(records) * env.config.control_dt,
        "total_reward": total_reward,
        "terminated": terminated,
        "truncated": truncated,
        "gait_success": bool(records[-1]["gait_success"]),
        "impact_limit_satisfied": bool(records[-1]["impact_limit_satisfied"]),
        "is_success": bool(records[-1]["is_success"]),
        "impact_limit_n": float(records[-1]["impact_limit_n"]),
        "maximum_landing_force_n": float(records[-1]["maximum_landing_force_n"]),
        "base_forward_displacement_m": float(records[-1]["base_forward_displacement_m"]),
        "mean_forward_velocity_m_s": float(records[-1]["base_forward_displacement_m"])
        / (len(records) * env.config.control_dt),
        "final_mean_left_force_n": float(np.mean([row["left_foot_force_n"] for row in final]))
        if final
        else 0.0,
        "final_mean_right_force_n": float(np.mean([row["right_foot_force_n"] for row in final]))
        if final
        else 0.0,
        "maximum_swing_height_m": float(max(row["swing_height_m"] for row in records)),
        "video": str(video_path),
        "camera": {
            "mode": args.camera_mode,
            "azimuth": args.camera_azimuth,
            "elevation": args.camera_elevation,
            "distance": args.camera_distance,
            "lookat": list(args.camera_lookat),
        },
    }
    for index in range(1, 9):
        for metric in (
            "advance_mean_force_n",
            "advance_under_5n_fraction",
            "length_m",
            "both_contact_fraction",
            "peak_landing_force_n",
        ):
            summary[f"step_{index}_{metric}"] = float(records[-1][f"step_{index}_{metric}"])
    (args.output / f"{label}_summary.json").write_text(
        json.dumps(summary, indent=2, allow_nan=False), encoding="utf-8"
    )
    (args.output / f"{label}_trajectory.json").write_text(
        json.dumps(records, indent=2, allow_nan=False), encoding="utf-8"
    )
    print(json.dumps(summary, indent=2, allow_nan=False))


if __name__ == "__main__":
    main()
