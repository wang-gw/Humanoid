#!/usr/bin/env python3
"""Evaluate V14 stage-1 stepping with a fixed, axis-labelled camera."""

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

from humanoidv2 import DynamicForwardV14Env  # noqa: E402


def annotate_frame(frame: np.ndarray, lines: tuple[str, ...]) -> np.ndarray:
    image = Image.fromarray(frame)
    draw = ImageDraw.Draw(image)
    font = ImageFont.load_default(size=17)
    height = 12 + 22 * len(lines)
    draw.rectangle((8, 8, 630, height), fill=(0, 0, 0))
    for index, line in enumerate(lines):
        draw.text((16, 12 + 22 * index), line, fill=(255, 255, 255), font=font)
    return np.asarray(image)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--first-side", choices=("left", "right"), required=True)
    parser.add_argument(
        "--model",
        type=Path,
        default=ROOT
        / "results/khr3hv_v14_dynamic_forward_stage1/ppo_dynamic_forward_v14_final.zip",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=ROOT / "results/khr3hv_v14_dynamic_forward_stage1",
    )
    parser.add_argument("--reference-only", action="store_true")
    parser.add_argument("--label-suffix", default="")
    parser.add_argument("--seed", type=int, default=17)
    parser.add_argument("--camera-azimuth", type=float, default=160.0)
    parser.add_argument("--camera-elevation", type=float, default=-10.0)
    parser.add_argument("--camera-distance", type=float, default=1.8)
    parser.add_argument(
        "--camera-lookat",
        type=float,
        nargs=3,
        metavar=("X", "Y", "Z"),
        default=(0.0, -0.12, 0.18),
    )
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    mode = "reference" if args.reference_only else "trained"
    suffix = f"_{args.label_suffix}" if args.label_suffix else ""
    label = f"dynamic_forward_v14_{args.first_side}_first_{mode}_world_fixed{suffix}"
    policy = None if args.reference_only else PPO.load(args.model, device="cpu")
    env = DynamicForwardV14Env(
        render_mode="rgb_array",
        reference_only=args.reference_only,
        fixed_first_side=args.first_side,
    )
    env.configure_render_camera(
        mode="world_fixed",
        lookat=tuple(args.camera_lookat),
        distance=args.camera_distance,
        azimuth=args.camera_azimuth,
        elevation=args.camera_elevation,
    )
    observation, _ = env.reset(seed=args.seed)
    start_world_y = float(env.data.qpos[1])
    frames = [
        annotate_frame(
            env.render(),
            (
                "V14 stage 1 | anatomical forward = world -Y",
                "t=0.0 s | world dY=0.000 m | forward=0.000 m",
            ),
        )
    ]
    records: list[dict[str, object]] = []
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
                "world_y_m": float(env.data.qpos[1]),
                "world_y_displacement_m": float(env.data.qpos[1] - start_world_y),
                "reward": float(reward),
                **{
                    key: value if isinstance(value, (bool, str)) else float(value)
                    for key, value in info.items()
                    if isinstance(value, (bool, str, int, float, np.number))
                },
            }
        )
        frames.append(
            annotate_frame(
                env.render(),
                (
                    "V14 stage 1 | anatomical forward = world -Y",
                    f"t={(step + 1) * env.config.control_dt:.1f} s | "
                    f"world dY={env.data.qpos[1] - start_world_y:+.3f} m | "
                    f"forward={float(info['base_forward_displacement_m']):.3f} m",
                ),
            )
        )
        if terminated or truncated:
            break
    env.close()

    video_path = args.output / f"{label}.mp4"
    imageio.mimsave(video_path, frames, fps=25, macro_block_size=16)
    for name, index in (
        ("first", 0),
        ("step4", min(4 * env.step_cycle_steps, len(frames) - 1)),
        ("final", len(frames) - 1),
    ):
        imageio.imwrite(args.output / f"{label}_{name}.png", frames[index])
    final = records[-1]
    summary: dict[str, object] = {
        "forward_axis": final["forward_axis"],
        "order": final["step_order"],
        "mode": mode,
        "model": None if policy is None else str(args.model),
        "steps": len(records),
        "duration_seconds": len(records) * env.config.control_dt,
        "step_cycle_seconds": env.step_cycle_steps * env.config.control_dt,
        "total_reward": total_reward,
        "terminated": terminated,
        "truncated": truncated,
        "gait_success": bool(final["gait_success"]),
        "impact_limit_satisfied": bool(final["impact_limit_satisfied"]),
        "is_success": bool(final["is_success"]),
        "world_y_displacement_m": float(final["world_y_displacement_m"]),
        "base_forward_displacement_m": float(final["base_forward_displacement_m"]),
        "average_forward_velocity_m_s": float(final["base_forward_displacement_m"])
        / (len(records) * env.config.control_dt),
        "maximum_landing_force_n": float(final["maximum_landing_force_n"]),
        "minimum_step_length_m": min(
            float(final[f"step_{index}_length_m"]) for index in range(1, 9)
        ),
        "minimum_advance_under_5n_fraction": min(
            float(final[f"step_{index}_advance_under_5n_fraction"])
            for index in range(1, 9)
        ),
        "video": str(video_path),
        "camera": {
            "mode": "world_fixed",
            "azimuth": args.camera_azimuth,
            "elevation": args.camera_elevation,
            "distance": args.camera_distance,
            "lookat": list(args.camera_lookat),
        },
    }
    (args.output / f"{label}_summary.json").write_text(
        json.dumps(summary, indent=2, allow_nan=False), encoding="utf-8"
    )
    (args.output / f"{label}_trajectory.json").write_text(
        json.dumps(records, indent=2, allow_nan=False), encoding="utf-8"
    )
    print(json.dumps(summary, indent=2, allow_nan=False))


if __name__ == "__main__":
    main()
