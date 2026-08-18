#!/usr/bin/env python3
"""Reproduce V20 failures and report the exact failing footsteps for V22."""

from __future__ import annotations

import json
import sys
from pathlib import Path

from stable_baselines3 import PPO

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from humanoidv2 import CounterfactualProbeV22Env  # noqa: E402
from scripts.train_landing_residual_v19 import combined_domain  # noqa: E402


BASE = ROOT / "results/khr3hv_v17_forward_margin/ppo_forward_margin_v17_final.zip"
CORRECTOR = ROOT / "results/khr3hv_v20_contact_aware/ppo_contact_aware_v20_final.zip"
V20_RESULTS = ROOT / "results/khr3hv_v20_contact_aware/robustness_results.json"
OUTPUT = ROOT / "results/khr3hv_v22_counterfactual"


def step_diagnostics(info: dict[str, object]) -> list[dict[str, object]]:
    rows = []
    for number in range(1, 9):
        impact = float(info[f"step_{number}_peak_landing_force_n"])
        unload = float(info[f"step_{number}_advance_under_5n_fraction"])
        double_support = float(info[f"step_{number}_both_contact_fraction"])
        length = float(info[f"step_{number}_length_m"])
        failures = []
        if impact > 50.0:
            failures.append("impact")
        if unload < 0.9:
            failures.append("swing_unload")
        if double_support < 0.9:
            failures.append("double_support")
        if length < 0.020:
            failures.append("step_length")
        rows.append(
            {
                "step": number,
                "side": str(info["step_order"]).split("->")[number - 1],
                "peak_landing_force_n": impact,
                "advance_under_5n_fraction": unload,
                "both_contact_fraction": double_support,
                "step_length_m": length,
                "failures": failures,
            }
        )
    return rows


def run(base: PPO, corrector: PPO, side: str, seed: int) -> dict[str, object]:
    env = CounterfactualProbeV22Env(
        base_policy=base,
        fixed_first_side=side,
        fixed_domain=combined_domain(seed),
        curriculum_stage=2,
        early_gate_blend=0.75,
    )
    observation, _ = env.reset(seed=seed)
    terminated = truncated = False
    for count in range(1, env.config.max_episode_steps + 1):
        action = corrector.predict(observation, deterministic=True)[0]
        observation, _, terminated, truncated, info = env.step(action)
        if terminated or truncated:
            break
    steps = step_diagnostics(info)
    row = {
        "seed": seed,
        "first_side": side,
        "terminated": terminated,
        "truncated": truncated,
        "success": bool(info["is_success"]),
        "base_forward_displacement_m": float(info["base_forward_displacement_m"]),
        "maximum_landing_force_n": float(info["maximum_landing_force_n"]),
        "failing_steps": [step["step"] for step in steps if step["failures"]],
        "steps": steps,
    }
    env.close()
    return row


def main() -> None:
    OUTPUT.mkdir(parents=True, exist_ok=True)
    previous = json.loads(V20_RESULTS.read_text(encoding="utf-8"))
    failures = [row for row in previous if not row["success"]]
    base = PPO.load(BASE, device="cpu")
    corrector = PPO.load(CORRECTOR, device="cpu")
    rows = []
    for previous_row in failures:
        row = run(
            base,
            corrector,
            str(previous_row["first_side"]),
            int(previous_row["seed"]),
        )
        rows.append(row)
        print(
            json.dumps(
                {
                    "seed": row["seed"],
                    "first_side": row["first_side"],
                    "failing_steps": row["failing_steps"],
                    "step_failures": {
                        str(step["step"]): step["failures"]
                        for step in row["steps"]
                        if step["failures"]
                    },
                },
                allow_nan=False,
            ),
            flush=True,
        )
    summary = {
        "controllers": {
            "base": str(BASE),
            "corrector": str(CORRECTOR),
        },
        "runs": len(rows),
        "failing_footsteps": sum(len(row["failing_steps"]) for row in rows),
        "rows": rows,
    }
    (OUTPUT / "baseline_failure_diagnostics.json").write_text(
        json.dumps(summary, indent=2, allow_nan=False), encoding="utf-8"
    )
    print(json.dumps({"runs": len(rows), "failing_footsteps": summary["failing_footsteps"]}, indent=2))


if __name__ == "__main__":
    main()
