#!/usr/bin/env python3
"""Render nominal, recovered, and boundary-regression V22 comparisons."""

from __future__ import annotations

import json
import os
import sys
from dataclasses import dataclass
from pathlib import Path

os.environ.setdefault("MUJOCO_GL", "egl")

import imageio.v2 as imageio
import numpy as np
from stable_baselines3 import PPO

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from humanoidv2 import FirstStepStanceHipRollV22Env  # noqa: E402
from scripts.diagnose_counterfactual_v22 import BASE, CORRECTOR, OUTPUT  # noqa: E402
from scripts.evaluate_dynamic_forward_v15 import annotate  # noqa: E402
from scripts.train_landing_residual_v19 import combined_domain  # noqa: E402


@dataclass(frozen=True)
class RenderCase:
    title: str
    side: str
    seed: int
    combined: bool
    amplitude: float


def build(case: RenderCase):
    base = PPO.load(BASE, device="cpu")
    corrector = PPO.load(CORRECTOR, device="cpu")
    env = FirstStepStanceHipRollV22Env(
        render_mode="rgb_array",
        base_policy=base,
        fixed_first_side=case.side,
        fixed_domain=combined_domain(case.seed) if case.combined else {},
        curriculum_stage=2,
        early_gate_blend=0.75,
        stance_hip_roll_lift_offset_rad=case.amplitude,
    )
    env.configure_render_camera(
        mode="world_fixed",
        lookat=(0.0, -0.12, 0.18),
        distance=1.8,
        azimuth=160.0,
        elevation=-10.0,
    )
    observation, _ = env.reset(seed=case.seed)
    return corrector, env, observation


def metrics(info, terminated, steps):
    return {
        "steps": steps,
        "terminated": terminated,
        "success": bool(info["is_success"]),
        "forward_displacement_m": float(info["base_forward_displacement_m"]),
        "maximum_landing_force_n": float(info["maximum_landing_force_n"]),
        "minimum_unload_fraction": min(
            float(info[f"step_{index}_advance_under_5n_fraction"])
            for index in range(1, 9)
        ),
        "minimum_both_contact_fraction": min(
            float(info[f"step_{index}_both_contact_fraction"])
            for index in range(1, 9)
        ),
        "minimum_step_length_m": min(
            float(info[f"step_{index}_length_m"]) for index in range(1, 9)
        ),
    }


def render_pair(label: str, first: RenderCase, second: RenderCase) -> None:
    policy_a, env_a, obs_a = build(first)
    policy_b, env_b, obs_b = build(second)
    frames = []
    info_a = info_b = None
    terminated_a = terminated_b = False
    truncated_a = truncated_b = False
    for step in range(env_a.config.max_episode_steps + 1):
        if step:
            obs_a, _, terminated_a, truncated_a, info_a = env_a.step(
                policy_a.predict(obs_a, deterministic=True)[0]
            )
            obs_b, _, terminated_b, truncated_b, info_b = env_b.step(
                policy_b.predict(obs_b, deterministic=True)[0]
            )
        time_s = step * env_a.config.control_dt
        if info_a is None:
            status_a = status_b = "forward=0.000 m | peak=0.0 N"
        else:
            status_a = (
                f"forward={float(info_a['base_forward_displacement_m']):.3f} m | "
                f"peak={float(info_a['maximum_landing_force_n']):.1f} N"
            )
            status_b = (
                f"forward={float(info_b['base_forward_displacement_m']):.3f} m | "
                f"peak={float(info_b['maximum_landing_force_n']):.1f} N"
            )
        frame_a = annotate(
            env_a.render(), (first.title, f"t={time_s:.1f} s | {status_a}")
        )
        frame_b = annotate(
            env_b.render(), (second.title, f"t={time_s:.1f} s | {status_b}")
        )
        frames.append(np.hstack((frame_a, frame_b)))
        if step and (terminated_a or truncated_a or terminated_b or truncated_b):
            break
    path = OUTPUT / f"{label}.mp4"
    imageio.mimsave(path, frames, fps=25, macro_block_size=16)
    imageio.imwrite(OUTPUT / f"{label}_final.png", frames[-1])
    summary = {
        "video": str(path),
        "first": {
            **first.__dict__,
            **metrics(info_a, terminated_a, step),
        },
        "second": {
            **second.__dict__,
            **metrics(info_b, terminated_b, step),
        },
    }
    (OUTPUT / f"{label}_summary.json").write_text(
        json.dumps(summary, indent=2, allow_nan=False), encoding="utf-8"
    )
    env_a.close()
    env_b.close()
    print(json.dumps(summary, indent=2, allow_nan=False), flush=True)


def main() -> None:
    OUTPUT.mkdir(parents=True, exist_ok=True)
    render_pair(
        "v22_nominal_left_right_front",
        RenderCase("V22 0.008 rad | left first", "left", 17, False, 0.008),
        RenderCase("V22 0.008 rad | right first", "right", 17, False, 0.008),
    )
    render_pair(
        "v22_seed106_right_v20_vs_v22_front",
        RenderCase("V20 baseline | fail", "right", 106, True, 0.0),
        RenderCase("V22 stance correction | pass", "right", 106, True, 0.008),
    )
    render_pair(
        "v22_seed134_right_holdout_recovery_front",
        RenderCase("V20 holdout | fail", "right", 134, True, 0.0),
        RenderCase("V22 holdout | pass", "right", 134, True, 0.008),
    )
    render_pair(
        "v22_seed188_left_boundary_regression_front",
        RenderCase("V20 audit | pass", "left", 188, True, 0.0),
        RenderCase("V22 audit | 50.05 N fail", "left", 188, True, 0.008),
    )


if __name__ == "__main__":
    main()
