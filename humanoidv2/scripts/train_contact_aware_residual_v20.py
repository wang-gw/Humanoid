#!/usr/bin/env python3
"""Transfer V19 into a contact-aware 90-input V20 corrector and train it."""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

import numpy as np
import torch
from stable_baselines3 import PPO
from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3.common.monitor import Monitor

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from humanoidv2 import ContactAwareResidualV20Env  # noqa: E402
from scripts.train_landing_residual_v19 import Progress, combined_domain  # noqa: E402


BASE = ROOT / "results/khr3hv_v17_forward_margin/ppo_forward_margin_v17_final.zip"
SOURCE = ROOT / "results/khr3hv_v19_landing_residual_joint/ppo_landing_residual_v19_final.zip"
OUTPUT = ROOT / "results/khr3hv_v20_contact_aware"


def transfer_v19_policy(source: PPO, target: PPO) -> None:
    """Copy every V19 parameter and zero only the eight new input columns."""
    source_state = source.policy.state_dict()
    target_state = target.policy.state_dict()
    expanded = {
        "mlp_extractor.policy_net.0.weight",
        "mlp_extractor.value_net.0.weight",
    }
    with torch.no_grad():
        for name, target_value in target_state.items():
            source_value = source_state[name]
            if name in expanded:
                target_value.zero_()
                target_value[:, : source_value.shape[1]].copy_(source_value)
            else:
                target_value.copy_(source_value)
    target.policy.load_state_dict(target_state)


def run_episode(
    corrector: PPO,
    base: PPO,
    side: str,
    seed: int,
    domain: dict[str, float | int],
) -> dict[str, object]:
    env = ContactAwareResidualV20Env(
        base_policy=base,
        fixed_first_side=side,
        fixed_domain=domain,
        curriculum_stage=3,
        early_gate_blend=1.0,
    )
    observation, _ = env.reset(seed=seed)
    maximum_correction = 0.0
    for step in range(env.config.max_episode_steps):
        action = corrector.predict(observation, deterministic=True)[0]
        observation, _, terminated, truncated, info = env.step(action)
        maximum_correction = max(
            maximum_correction, float(info["applied_correction_max_abs"])
        )
        if terminated or truncated:
            break
    row: dict[str, object] = {
        "first_side": side,
        "seed": seed,
        **domain,
        "steps": step + 1,
        "terminated": terminated,
        "truncated": truncated,
        "success": bool(info["is_success"]),
        "impact_limit_satisfied": bool(info["impact_limit_satisfied"]),
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
    }
    env.close()
    return row


def evaluate(path: Path, base_path: Path) -> list[dict[str, object]]:
    corrector = PPO.load(path, device="cpu")
    base = PPO.load(base_path, device="cpu")
    rows: list[dict[str, object]] = []
    for side in ("left", "right"):
        row = run_episode(corrector, base, side, 17, {})
        row.update({"model": path.name, "case": "nominal"})
        rows.append(row)
    for sample, seed in enumerate(range(101, 121), start=1):
        for side in ("left", "right"):
            row = run_episode(corrector, base, side, seed, combined_domain(seed))
            row.update({"model": path.name, "case": "combined", "sample": sample})
            rows.append(row)
    return rows


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--base-policy", type=Path, default=BASE)
    parser.add_argument("--source", type=Path, default=SOURCE)
    parser.add_argument("--output", type=Path, default=OUTPUT)
    parser.add_argument("--n-envs", type=int, default=4)
    parser.add_argument("--seed", type=int, default=37)
    parser.add_argument("--learning-rate", type=float, default=1.0e-4)
    parser.add_argument("--stage-timesteps", type=int, nargs=3, default=(32_000, 48_000, 80_000))
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)

    def build_env():
        base = PPO.load(args.base_policy, device="cpu")
        return Monitor(
            ContactAwareResidualV20Env(
                base_policy=base,
                curriculum_stage=1,
                early_gate_blend=0.0,
                landing_force_reward_scale=1.25,
                downward_velocity_reward_scale=0.20,
                correction_penalty_scale=0.002,
                correction_rate_penalty_scale=0.002,
                terminal_success_bonus=50.0,
            )
        )

    env = make_vec_env(build_env, n_envs=args.n_envs, seed=args.seed)
    model = PPO(
        "MlpPolicy",
        env,
        device="cpu",
        learning_rate=args.learning_rate,
        n_steps=1024,
        batch_size=256,
        n_epochs=7,
        gamma=0.995,
        gae_lambda=0.95,
        clip_range=0.10,
        ent_coef=0.003,
        target_kl=0.02,
        policy_kwargs={"net_arch": dict(pi=[128, 128], vf=[128, 128])},
        seed=args.seed,
        verbose=0,
    )
    source = PPO.load(args.source, device="cpu")
    transfer_v19_policy(source, model)
    initial_path = args.output / "ppo_contact_aware_v20_initial.zip"
    model.save(initial_path)
    candidates = [initial_path]
    progress = Progress()
    stage_records = []
    started = time.time()
    for stage, timesteps in enumerate(args.stage_timesteps):
        env.env_method("set_v20_stage", stage)
        model.learn(
            timesteps,
            callback=progress,
            progress_bar=False,
            reset_num_timesteps=stage == 0,
        )
        path = args.output / f"ppo_contact_aware_v20_stage{stage}.zip"
        model.save(path)
        candidates.append(path)
        recent = progress.episodes[-20:]
        record = {
            "stage": stage,
            "requested_timesteps": timesteps,
            "cumulative_timesteps": model.num_timesteps,
            "episodes_seen": len(progress.episodes),
            "recent_reward": float(np.mean([row["r"] for row in recent])) if recent else None,
            "recent_length": float(np.mean([row["l"] for row in recent])) if recent else None,
            "model": path.name,
        }
        stage_records.append(record)
        print(json.dumps(record, allow_nan=False), flush=True)

    comparison: list[dict[str, object]] = []
    for path in candidates:
        print(f"evaluating {path.name}", flush=True)
        comparison.extend(evaluate(path, args.base_policy))
    grouped = {
        path: [row for row in comparison if row["model"] == path.name]
        for path in candidates
    }

    def key(path: Path) -> tuple[float, ...]:
        rows = grouped[path]
        nominal = [row for row in rows if row["case"] == "nominal"]
        combined = [row for row in rows if row["case"] == "combined"]
        paired = sum(
            all(row["success"] for row in combined if row["sample"] == sample)
            for sample in range(1, 21)
        )
        return (
            float(sum(row["success"] for row in nominal)),
            float(sum(row["success"] for row in combined)),
            float(paired),
            -max(float(row["maximum_landing_force_n"]) for row in combined),
            min(float(row["base_forward_displacement_m"]) for row in rows),
        )

    selected = max(candidates, key=key)
    PPO.load(selected, device="cpu").save(args.output / "ppo_contact_aware_v20_final")
    scores = []
    for path in candidates:
        rows = grouped[path]
        nominal = [row for row in rows if row["case"] == "nominal"]
        combined = [row for row in rows if row["case"] == "combined"]
        scores.append(
            {
                "model": path.name,
                "nominal_successes": sum(row["success"] for row in nominal),
                "combined_successes": sum(row["success"] for row in combined),
                "combined_runs": len(combined),
                "both_sides_successful_samples": sum(
                    all(row["success"] for row in combined if row["sample"] == sample)
                    for sample in range(1, 21)
                ),
                "worst_landing_force_n": max(
                    float(row["maximum_landing_force_n"]) for row in combined
                ),
                "minimum_forward_displacement_m": min(
                    float(row["base_forward_displacement_m"]) for row in rows
                ),
                "selection_key": list(key(path)),
            }
        )
    summary = {
        "base_policy": str(args.base_policy),
        "source_v19_corrector": str(args.source),
        "observation_size": 90,
        "new_input_weights_initialized_to_zero": True,
        "elapsed_seconds": time.time() - started,
        "actual_timesteps": model.num_timesteps,
        "stage_records": stage_records,
        "candidate_scores": scores,
        "selected_model": selected.name,
    }
    (args.output / "checkpoint_comparison.json").write_text(
        json.dumps(comparison, indent=2, allow_nan=False), encoding="utf-8"
    )
    (args.output / "training_summary.json").write_text(
        json.dumps(summary, indent=2, allow_nan=False), encoding="utf-8"
    )
    env.close()
    print(json.dumps(summary, indent=2, allow_nan=False), flush=True)


if __name__ == "__main__":
    main()
