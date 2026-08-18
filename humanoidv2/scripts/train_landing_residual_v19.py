#!/usr/bin/env python3
"""Train a bounded landing-only corrector over the frozen V17 policy."""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

import numpy as np
import torch
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import BaseCallback
from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3.common.monitor import Monitor

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from humanoidv2 import LandingResidualV19Env  # noqa: E402


BASE_POLICY = ROOT / "results/khr3hv_v17_forward_margin/ppo_forward_margin_v17_final.zip"
OUTPUT = ROOT / "results/khr3hv_v19_landing_residual_joint"
VALIDATION_SEEDS = (101, 102, 106, 108, 113, 115, 117, 120)


class Progress(BaseCallback):
    def __init__(self) -> None:
        super().__init__()
        self.episodes: list[dict[str, float]] = []
        self.last_report = 0

    def _on_step(self) -> bool:
        for info in self.locals.get("infos", []):
            if "episode" in info:
                self.episodes.append(
                    {key: float(value) for key, value in info["episode"].items()}
                )
        if self.num_timesteps - self.last_report >= 10_000:
            self.last_report = self.num_timesteps
            recent = self.episodes[-20:]
            print(
                json.dumps(
                    {
                        "timesteps": self.num_timesteps,
                        "episodes": len(self.episodes),
                        "recent_reward": float(np.mean([row["r"] for row in recent]))
                        if recent else None,
                        "recent_length": float(np.mean([row["l"] for row in recent]))
                        if recent else None,
                    },
                    allow_nan=False,
                ),
                flush=True,
            )
        return True


def combined_domain(seed: int) -> dict[str, float | int]:
    generator = np.random.default_rng(seed)
    return {
        "friction_scale": float(generator.uniform(0.80, 1.20)),
        "mass_scale": float(generator.uniform(0.98, 1.02)),
        "leg_mass_asymmetry": float(generator.uniform(-0.01, 0.01)),
        "initial_joint_noise_rad": 0.0035,
        "initial_joint_velocity_noise_rad_s": 0.010,
        "motor_gain_scale": float(generator.uniform(0.95, 1.05)),
        "control_delay_steps": int(generator.integers(0, 3)),
    }


def run_episode(
    corrector: PPO,
    base_policy: PPO,
    first_side: str,
    seed: int,
    domain: dict[str, float | int],
) -> dict[str, object]:
    env = LandingResidualV19Env(
        fixed_first_side=first_side,
        base_policy=base_policy,
        fixed_domain=domain,
    )
    observation, _ = env.reset(seed=seed)
    maximum_applied_correction = 0.0
    total_reward = 0.0
    for step in range(env.config.max_episode_steps):
        action = corrector.predict(observation, deterministic=True)[0]
        observation, reward, terminated, truncated, info = env.step(action)
        total_reward += reward
        maximum_applied_correction = max(
            maximum_applied_correction, float(info["applied_correction_max_abs"])
        )
        if terminated or truncated:
            break
    row: dict[str, object] = {
        "first_side": first_side,
        "seed": seed,
        **domain,
        "steps": step + 1,
        "terminated": terminated,
        "truncated": truncated,
        "success": bool(info["is_success"]),
        "gait_success": bool(info["gait_success"]),
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
        "maximum_applied_correction": maximum_applied_correction,
        "total_reward": total_reward,
    }
    env.close()
    return row


def evaluate_candidate(path: Path, base_policy_path: Path) -> list[dict[str, object]]:
    corrector = PPO.load(path, device="cpu")
    base_policy = PPO.load(base_policy_path, device="cpu")
    rows: list[dict[str, object]] = []
    for first_side in ("left", "right"):
        row = run_episode(corrector, base_policy, first_side, 17, {})
        row.update({"model": path.name, "case": "nominal"})
        rows.append(row)
    for seed in VALIDATION_SEEDS:
        domain = combined_domain(seed)
        for first_side in ("left", "right"):
            row = run_episode(corrector, base_policy, first_side, seed, domain)
            row.update({"model": path.name, "case": "combined"})
            rows.append(row)
    return rows


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--base-policy", type=Path, default=BASE_POLICY)
    parser.add_argument("--output", type=Path, default=OUTPUT)
    parser.add_argument("--n-envs", type=int, default=4)
    parser.add_argument("--seed", type=int, default=19)
    parser.add_argument("--learning-rate", type=float, default=2.0e-4)
    parser.add_argument("--stage-timesteps", type=int, nargs=3, default=(24_000, 40_000, 56_000))
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)

    def build_env():
        base_policy = PPO.load(args.base_policy, device="cpu")
        return Monitor(
            LandingResidualV19Env(
                base_policy=base_policy,
                curriculum_stage=0,
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
        ent_coef=0.005,
        target_kl=0.02,
        policy_kwargs={"net_arch": dict(pi=[128, 128], vf=[128, 128])},
        seed=args.seed,
        verbose=0,
    )
    # Exact zero deterministic correction reproduces V17 before learning.
    with torch.no_grad():
        model.policy.action_net.weight.zero_()
        model.policy.action_net.bias.zero_()
        model.policy.log_std.fill_(-0.5)

    initial_path = args.output / "ppo_landing_residual_v19_initial.zip"
    model.save(initial_path)
    candidates = [initial_path]
    progress = Progress()
    stage_records = []
    started = time.time()
    for stage, timesteps in enumerate(args.stage_timesteps):
        env.env_method("set_curriculum_stage", stage)
        before = model.num_timesteps
        model.learn(
            timesteps,
            callback=progress,
            progress_bar=False,
            reset_num_timesteps=stage == 0,
        )
        path = args.output / f"ppo_landing_residual_v19_stage{stage}.zip"
        model.save(path)
        candidates.append(path)
        recent = progress.episodes[-20:]
        record = {
            "stage": stage,
            "requested_timesteps": timesteps,
            "cumulative_timesteps": model.num_timesteps,
            "stage_actual_timesteps": model.num_timesteps - (0 if stage == 0 else before),
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
        comparison.extend(evaluate_candidate(path, args.base_policy))
    grouped = {
        path: [row for row in comparison if row["model"] == path.name] for path in candidates
    }

    def selection_key(path: Path) -> tuple[float, ...]:
        rows = grouped[path]
        nominal = [row for row in rows if row["case"] == "nominal"]
        combined = [row for row in rows if row["case"] == "combined"]
        return (
            float(sum(row["success"] for row in nominal)),
            float(sum(row["success"] for row in combined)),
            -max(float(row["maximum_landing_force_n"]) for row in combined),
            min(float(row["base_forward_displacement_m"]) for row in rows),
            -max(float(row["maximum_applied_correction"]) for row in rows),
        )

    selected = max(candidates, key=selection_key)
    PPO.load(selected, device="cpu").save(args.output / "ppo_landing_residual_v19_final")
    (args.output / "checkpoint_comparison.json").write_text(
        json.dumps(comparison, indent=2, allow_nan=False), encoding="utf-8"
    )
    candidate_scores = []
    for path in candidates:
        rows = grouped[path]
        nominal = [row for row in rows if row["case"] == "nominal"]
        combined = [row for row in rows if row["case"] == "combined"]
        candidate_scores.append(
            {
                "model": path.name,
                "nominal_successes": sum(row["success"] for row in nominal),
                "combined_successes": sum(row["success"] for row in combined),
                "combined_runs": len(combined),
                "worst_combined_landing_force_n": max(
                    float(row["maximum_landing_force_n"]) for row in combined
                ),
                "minimum_forward_displacement_m": min(
                    float(row["base_forward_displacement_m"]) for row in rows
                ),
                "maximum_applied_correction": max(
                    float(row["maximum_applied_correction"]) for row in rows
                ),
                "selection_key": list(selection_key(path)),
            }
        )
    summary = {
        "base_policy": str(args.base_policy),
        "elapsed_seconds": time.time() - started,
        "landing_correction_limit_rad": 0.025,
        "learning_rate": args.learning_rate,
        "n_steps": 1024,
        "batch_size": 256,
        "n_epochs": 7,
        "actual_timesteps": model.num_timesteps,
        "stage_records": stage_records,
        "validation_seeds": list(VALIDATION_SEEDS),
        "candidate_scores": candidate_scores,
        "selected_model": selected.name,
        "selection_rule": (
            "both nominal successes, combined successes, worst combined impact, "
            "minimum forward distance, then smaller correction"
        ),
    }
    (args.output / "training_summary.json").write_text(
        json.dumps(summary, indent=2, allow_nan=False), encoding="utf-8"
    )
    env.close()
    print(json.dumps(summary, indent=2, allow_nan=False), flush=True)


if __name__ == "__main__":
    main()
