#!/usr/bin/env python3
"""Search stride length at the fixed V16 timing and gain schedule."""

from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np
from stable_baselines3 import PPO

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from humanoidv2 import DynamicForwardV16Env  # noqa: E402


def main() -> None:
    model_path = (
        ROOT / "results/khr3hv_v16_dynamic_forward_stage3/ppo_dynamic_forward_v16_final.zip"
    )
    policy = PPO.load(model_path, device="cpu")
    rows = []
    for stride_m in (0.035, 0.038, 0.040, 0.042):
        for mode in ("reference", "policy"):
            for side in ("left", "right"):
                env = DynamicForwardV16Env(
                    stride_m=stride_m,
                    reference_only=mode == "reference",
                    fixed_first_side=side,
                )
                observation, _ = env.reset(seed=17)
                start_world_y = float(env.data.qpos[1])
                torque_peaks = []
                saturation = []
                for step in range(env.config.max_episode_steps):
                    action = (
                        np.zeros(10, dtype=np.float32)
                        if mode == "reference"
                        else policy.predict(observation, deterministic=True)[0]
                    )
                    observation, reward, terminated, truncated, info = env.step(action)
                    torque_peaks.append(float(info["peak_control_interval_torque_n_m"]))
                    saturation.append(float(info["actuator_saturation_fraction"]))
                    if terminated or truncated:
                        break
                rows.append(
                    {
                        "stride_m": stride_m,
                        "mode": mode,
                        "first_side": side,
                        "steps": step + 1,
                        "terminated": terminated,
                        "truncated": truncated,
                        "strict_success": bool(info["is_success"]),
                        "forward_displacement_m": float(info["base_forward_displacement_m"]),
                        "world_y_displacement_m": float(env.data.qpos[1] - start_world_y),
                        "reaches_200mm": bool(info["base_forward_displacement_m"] >= 0.200),
                        "maximum_landing_force_n": float(info["maximum_landing_force_n"]),
                        "minimum_step_length_m": min(
                            float(info[f"step_{index}_length_m"])
                            for index in range(1, 9)
                        ),
                        "minimum_advance_under_5n_fraction": min(
                            float(info[f"step_{index}_advance_under_5n_fraction"])
                            for index in range(1, 9)
                        ),
                        "maximum_actuator_torque_n_m": max(torque_peaks),
                        "maximum_saturation_fraction": max(saturation),
                    }
                )
                env.close()
    groups = {
        stride: [row for row in rows if row["stride_m"] == stride]
        for stride in (0.035, 0.038, 0.040, 0.042)
    }
    selected = max(
        groups,
        key=lambda stride: (
            sum(bool(row["strict_success"]) for row in groups[stride]),
            sum(bool(row["reaches_200mm"]) for row in groups[stride]),
            min(float(row["forward_displacement_m"]) for row in groups[stride]),
            -max(float(row["maximum_landing_force_n"]) for row in groups[stride]),
        ),
    )
    result = {
        "source_policy": str(model_path),
        "selected_stride_m": selected,
        "rows": rows,
    }
    output = ROOT / "results/khr3hv_v17_forward_margin"
    output.mkdir(parents=True, exist_ok=True)
    (output / "stride_search.json").write_text(
        json.dumps(result, indent=2, allow_nan=False), encoding="utf-8"
    )
    print(json.dumps(result, indent=2, allow_nan=False))


if __name__ == "__main__":
    main()
