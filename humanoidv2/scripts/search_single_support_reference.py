#!/usr/bin/env python3
"""Search slow IK/PD transitions that can hold one-foot support."""

from __future__ import annotations

import argparse
import itertools
import json
import sys
from dataclasses import replace
from pathlib import Path

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from humanoidv2 import KHR3HVEnv  # noqa: E402
from humanoidv2.khr3hv_env import KHRConfig  # noqa: E402


def simulate(
    sway_width: float,
    foot_height: float,
    ramp_seconds: float,
    kp: float,
    side: str,
    model_path: str | Path | None = None,
) -> dict[str, float | str | bool | int]:
    config = replace(
        KHRConfig(),
        enhanced_collisions=True,
        sway_width=sway_width,
        sway_offset=min(0.01, sway_width / 4.0),
        foot_height=foot_height,
        foot_height_offset=0.005,
        kp=kp,
        kd=max(0.05, kp * 0.004),
    )
    env = KHR3HVEnv(model_path=model_path, config=config)
    env.reset(seed=7)
    phase = 12 if side == "left" else 37
    start = env.data.qpos[env.qpos_ids].copy()
    target = env.reference[phase].copy()
    ramp_steps = int(round(ramp_seconds / config.control_dt))
    hold_steps = int(round(4.0 / config.control_dt))
    swing_forces: list[float] = []
    stance_forces: list[float] = []
    swing_heights: list[float] = []
    survived = True
    for step in range(ramp_steps + hold_steps):
        if step < ramp_steps:
            fraction = (step + 1) / ramp_steps
            blend = fraction * fraction * (3.0 - 2.0 * fraction)
        else:
            blend = 1.0
        joint_target = start + blend * (target - start)
        for _ in range(env.frame_skip):
            error = joint_target - env.data.qpos[env.qpos_ids]
            torque = config.kp * error - config.kd * env.data.qvel[env.dof_ids]
            env.data.ctrl[:] = np.clip(torque, -env.torque_limit, env.torque_limit)
            mujoco.mj_step(env.model, env.data)
        left_contact, right_contact, left_force, right_force = env._foot_contacts()
        if side == "left":
            swing_forces.append(left_force)
            stance_forces.append(right_force)
            swing_heights.append(float(env._sole_point(env.left_foot_id, True)[2]))
        else:
            swing_forces.append(right_force)
            stance_forces.append(left_force)
            swing_heights.append(float(env._sole_point(env.right_foot_id, True)[2]))
        if env.data.xipos[env.base_id, 2] < env.fall_height or not np.isfinite(env.data.qpos).all():
            survived = False
            break

    hold_window = min(50, max(1, len(swing_forces) - ramp_steps))
    recent_swing = np.asarray(swing_forces[-hold_window:])
    recent_stance = np.asarray(stance_forces[-hold_window:])
    result = {
        "side": side,
        "sway_width_m": sway_width,
        "foot_height_m": foot_height,
        "ramp_seconds": ramp_seconds,
        "kp": kp,
        "kd": config.kd,
        "steps_completed": len(swing_forces),
        "survived": survived and len(swing_forces) == ramp_steps + hold_steps,
        "mean_swing_force_n": float(recent_swing.mean()),
        "min_swing_force_n": float(recent_swing.min()),
        "swing_under_5n_fraction": float(np.mean(recent_swing < 5.0)),
        "mean_stance_force_n": float(recent_stance.mean()),
        "max_swing_height_m": float(max(swing_heights)),
        "final_torso_height_m": float(env.data.xipos[env.base_id, 2]),
    }
    result["success"] = bool(
        result["survived"]
        and result["mean_swing_force_n"] < 5.0
        and result["swing_under_5n_fraction"] >= 0.9
        and result["mean_stance_force_n"] > 30.0
    )
    env.close()
    return result


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, default=ROOT / "results/khr3hv_v4")
    parser.add_argument("--model", type=Path)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    widths = (0.025, 0.030, 0.035, 0.040, 0.050, 0.060)
    heights = (0.025, 0.035, 0.045)
    ramps = (2.0, 3.0, 4.0)
    gains = (18.5, 30.0, 50.0)
    results = [
        simulate(width, height, ramp, kp, side, args.model)
        for width, height, ramp, kp, side in itertools.product(widths, heights, ramps, gains, ("left", "right"))
    ]
    # The symmetric range finds a right-side solution, but the current CAD
    # model needs a larger center-of-mass shift and more damping on the left.
    results.extend(
        simulate(width, height, ramp, kp, "left", args.model)
        for width, height, ramp, kp in itertools.product(
            (0.080, 0.100, 0.120),
            (0.045, 0.055),
            (3.0, 4.0),
            (50.0, 65.0, 80.0),
        )
    )
    output = args.output / "single_support_search.json"
    output.write_text(json.dumps(results, indent=2), encoding="utf-8")
    ranked = sorted(
        results,
        key=lambda row: (
            not row["success"],
            not row["survived"],
            row["mean_swing_force_n"],
            -row["mean_stance_force_n"],
        ),
    )
    summary = {
        "evaluations": len(results),
        "successes": sum(bool(row["success"]) for row in results),
        "top_20": ranked[:20],
    }
    (args.output / "single_support_search_summary.json").write_text(
        json.dumps(summary, indent=2), encoding="utf-8"
    )
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
