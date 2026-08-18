#!/usr/bin/env python3
"""Compare hard and smooth V16 gain transitions on reference and policy."""

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
        ROOT / "results/khr3hv_v15_dynamic_forward_stage2/ppo_dynamic_forward_v15_final.zip"
    )
    policy = PPO.load(model_path, device="cpu")
    rows = []
    for transition in (0.0, 0.10, 0.11, 0.12, 0.15, 0.18, 0.20, 0.25, 0.40):
        for mode in ("reference", "policy"):
            for side in ("left", "right"):
                env = DynamicForwardV16Env(
                    reference_only=mode == "reference",
                    fixed_first_side=side,
                    gain_transition_fraction=transition,
                )
                observation, _ = env.reset(seed=17)
                for step in range(env.config.max_episode_steps):
                    action = (
                        np.zeros(10, dtype=np.float32)
                        if mode == "reference"
                        else policy.predict(observation, deterministic=True)[0]
                    )
                    observation, reward, terminated, truncated, info = env.step(action)
                    if terminated or truncated:
                        break
                rows.append(
                    {
                        "transition_fraction": transition,
                        "transition_control_samples": int(
                            np.ceil(transition * env.advance_steps)
                        ),
                        "mode": mode,
                        "first_side": side,
                        "steps": step + 1,
                        "success": bool(info["is_success"]),
                        "gait_success": bool(info["gait_success"]),
                        "impact_limit_satisfied": bool(info["impact_limit_satisfied"]),
                        "forward_displacement_m": float(info["base_forward_displacement_m"]),
                        "maximum_landing_force_n": float(info["maximum_landing_force_n"]),
                        "minimum_step_length_m": min(
                            float(info[f"step_{index}_length_m"])
                            for index in range(1, 9)
                        ),
                        "minimum_advance_under_5n_fraction": min(
                            float(info[f"step_{index}_advance_under_5n_fraction"])
                            for index in range(1, 9)
                        ),
                    }
                )
                env.close()
    result = {
        "policy": str(model_path),
        "selected_transition_fraction": 0.12,
        "selection_reason": (
            "smallest transition with an intermediate-gain sample, both-order strict "
            "success, and more than 4 mm forward margin"
        ),
        "rows": rows,
    }
    output = ROOT / "results/khr3hv_v16_dynamic_forward_stage3/transition_comparison.json"
    output.write_text(json.dumps(result, indent=2, allow_nan=False), encoding="utf-8")
    print(json.dumps(result, indent=2, allow_nan=False))


if __name__ == "__main__":
    main()
