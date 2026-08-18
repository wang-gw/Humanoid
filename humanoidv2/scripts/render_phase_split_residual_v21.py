#!/usr/bin/env python3
"""Render the selected V21 fallback and representative learned trade-offs."""

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

from humanoidv2 import PhaseSplitResidualV21Env  # noqa: E402
from scripts.evaluate_dynamic_forward_v15 import annotate  # noqa: E402
from scripts.train_landing_residual_v19 import combined_domain  # noqa: E402


BASE = ROOT / "results/khr3hv_v17_forward_margin/ppo_forward_margin_v17_final.zip"
OUTPUT = ROOT / "results/khr3hv_v21_phase_split"


@dataclass(frozen=True)
class RenderCase:
    title: str
    model: Path
    side: str
    seed: int
    combined: bool
    gate_scale: float


def build(case: RenderCase):
    base = PPO.load(BASE, device="cpu")
    corrector = PPO.load(case.model, device="cpu")
    domain = combined_domain(case.seed) if case.combined else {}
    env = PhaseSplitResidualV21Env(
        render_mode="rgb_array",
        base_policy=base,
        fixed_first_side=case.side,
        fixed_domain=domain,
        curriculum_stage=2,
        early_gate_blend=0.75,
        conditional_lift_gate_scale=case.gate_scale,
    )
    env.configure_render_camera(
        mode="world_fixed",
        lookat=(0.0, -0.12, 0.18),
        distance=1.8,
        azimuth=160.0,
        elevation=-10.0,
    )
    observation, _ = env.reset(seed=case.seed)
    return base, corrector, env, observation


def summarize(info, terminated, steps):
    return {
        "steps": steps,
        "terminated": terminated,
        "success": bool(info["is_success"]),
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
    }


def render_pair(label: str, first: RenderCase, second: RenderCase) -> None:
    _, policy_a, env_a, obs_a = build(first)
    _, policy_b, env_b, obs_b = build(second)
    frames = []
    info_a = info_b = None
    terminated_a = terminated_b = False
    truncated_a = truncated_b = False
    for step in range(env_a.config.max_episode_steps + 1):
        if step:
            action_a = policy_a.predict(obs_a, deterministic=True)[0]
            action_b = policy_b.predict(obs_b, deterministic=True)[0]
            obs_a, _, terminated_a, truncated_a, info_a = env_a.step(action_a)
            obs_b, _, terminated_b, truncated_b, info_b = env_b.step(action_b)
        time_s = step * env_a.config.control_dt
        if info_a is None:
            status_a = "forward=0.000 m | peak=0.0 N"
            status_b = "forward=0.000 m | peak=0.0 N"
        else:
            status_a = (
                f"forward={float(info_a['base_forward_displacement_m']):.3f} m | "
                f"peak={float(info_a['maximum_landing_force_n']):.1f} N"
            )
            status_b = (
                f"forward={float(info_b['base_forward_displacement_m']):.3f} m | "
                f"peak={float(info_b['maximum_landing_force_n']):.1f} N"
            )
        frame_a = annotate(env_a.render(), (first.title, f"t={time_s:.1f} s | {status_a}"))
        frame_b = annotate(env_b.render(), (second.title, f"t={time_s:.1f} s | {status_b}"))
        frames.append(np.hstack((frame_a, frame_b)))
        if step and (
            terminated_a or truncated_a or terminated_b or truncated_b
        ):
            break
    path = OUTPUT / f"{label}.mp4"
    imageio.mimsave(path, frames, fps=25, macro_block_size=16)
    imageio.imwrite(OUTPUT / f"{label}_final.png", frames[-1])
    summary = {
        "video": str(path),
        "first": {
            "title": first.title,
            "model": str(first.model),
            "side": first.side,
            "seed": first.seed,
            "combined": first.combined,
            "conditional_lift_gate_scale": first.gate_scale,
            **summarize(info_a, terminated_a, step),
        },
        "second": {
            "title": second.title,
            "model": str(second.model),
            "side": second.side,
            "seed": second.seed,
            "combined": second.combined,
            "conditional_lift_gate_scale": second.gate_scale,
            **summarize(info_b, terminated_b, step),
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
    selected = OUTPUT / "ppo_phase_split_v21_final.zip"
    learned = OUTPUT / "ppo_phase_split_v21_stage0.zip"
    render_pair(
        "v21_selected_nominal_left_right_front",
        RenderCase("V21 selected | left first", selected, "left", 17, False, 0.0),
        RenderCase("V21 selected | right first", selected, "right", 17, False, 0.0),
    )
    render_pair(
        "v21_seed106_right_selected_vs_learned_front",
        RenderCase("Selected fallback | V20-equivalent", selected, "right", 106, True, 0.0),
        RenderCase("Learned split | lower peak, still fail", learned, "right", 106, True, 0.0),
    )
    render_pair(
        "v21_seed120_left_selected_vs_learned_front",
        RenderCase("Selected fallback | pass", selected, "left", 120, True, 0.0),
        RenderCase("Learned split | 50 N regression", learned, "left", 120, True, 0.0),
    )


if __name__ == "__main__":
    main()
