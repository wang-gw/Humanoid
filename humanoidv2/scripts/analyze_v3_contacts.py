#!/usr/bin/env python3
"""Measure whether V3's nominal swing foot actually leaves the floor."""

from __future__ import annotations

import json
import sys
import argparse
from pathlib import Path

import numpy as np
from stable_baselines3 import PPO

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from humanoidv2 import KHR3HVV3Env, KHR3HVV31Env, KHR3HVV32Env, KHR3HVV33Env, KHR3HVV34Env  # noqa: E402


def analyze(name: str, model_path: Path | None, env_class) -> dict[str, float | int | str]:
    env = env_class(reference_only=model_path is None)
    model = None if model_path is None else PPO.load(model_path, device="cpu")
    observation, _ = env.reset(seed=7)
    rows: list[dict[str, float | bool]] = []
    for _ in range(500):
        action = np.zeros(10, dtype=np.float32) if model is None else model.predict(observation, deterministic=True)[0]
        observation, _, terminated, truncated, info = env.step(action)
        rows.append(info)
        if terminated or truncated:
            break
    left_sign = -1.0 if env.config.swap_swing_legs else 1.0
    left_swing = [
        row for row in rows if left_sign * np.sin(2.0 * np.pi * row["reference_phase"] / 50.0) > 0.2
    ]
    right_swing = [
        row for row in rows if left_sign * np.sin(2.0 * np.pi * row["reference_phase"] / 50.0) < -0.2
    ]
    result = {
        "name": name,
        "steps": len(rows),
        "distance_m": float(rows[-1]["distance"]),
        "double_support_fraction": float(
            np.mean([row["left_foot_contact"] and row["right_foot_contact"] for row in rows])
        ),
        "left_swing_clear_fraction": float(np.mean([not row["left_foot_contact"] for row in left_swing])),
        "right_swing_clear_fraction": float(np.mean([not row["right_foot_contact"] for row in right_swing])),
        "mean_left_normal_force_n": float(np.mean([row["left_foot_normal_force"] for row in rows])),
        "mean_right_normal_force_n": float(np.mean([row["right_foot_normal_force"] for row in rows])),
    }
    env.close()
    return result


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--version", choices=("v3", "v3_1", "v3_2", "v3_3", "v3_4"), default="v3")
    args = parser.parse_args()
    result_dir = ROOT / f"results/khr3hv_{args.version}"
    env_class = {
        "v3": KHR3HVV3Env,
        "v3_1": KHR3HVV31Env,
        "v3_2": KHR3HVV32Env,
        "v3_3": KHR3HVV33Env,
        "v3_4": KHR3HVV34Env,
    }[args.version]
    comparison_path = result_dir / "checkpoint_comparison.json"
    best_path: Path | None = None
    if comparison_path.exists():
        comparison = json.loads(comparison_path.read_text(encoding="utf-8"))
        checkpoint_rows = [row for row in comparison if row["model"] != "ppo_khr3hv_final.zip"]
        if checkpoint_rows:
            best_name = max(checkpoint_rows, key=lambda row: row["distance_m"])["model"]
            best_path = result_dir / "checkpoints" / best_name
    cases = (
        ("reference_only", None),
        ("best_checkpoint", best_path),
        ("final_policy", result_dir / "ppo_khr3hv_final.zip"),
    )
    available_cases = [
        (name, path)
        for name, path in cases
        if name == "reference_only" or (path is not None and path.exists())
    ]
    results = [analyze(name, path, env_class) for name, path in available_cases]
    output = result_dir / "contact_analysis.json"
    output.write_text(json.dumps(results, indent=2), encoding="utf-8")
    print(json.dumps(results, indent=2))


if __name__ == "__main__":
    main()
