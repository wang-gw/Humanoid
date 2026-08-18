#!/usr/bin/env python3
"""Conservatively fine-tune V10 with staged domain randomization."""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

import numpy as np
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import BaseCallback
from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3.common.monitor import Monitor

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from humanoidv2 import CurriculumSoftLandingV12Env, RobustSoftLandingV11Env  # noqa: E402


MILD_NOISE = {"initial_joint_noise_rad": 0.0035, "initial_joint_velocity_noise_rad_s": 0.01}
MODERATE_NOISE = {"initial_joint_noise_rad": 0.0087, "initial_joint_velocity_noise_rad_s": 0.02}
VALIDATION_CASES = (
    ("nominal", {}, (7,)),
    ("initial_mild", MILD_NOISE, (7, 17, 29, 43, 61)),
    ("initial_moderate", MODERATE_NOISE, (29, 43)),
    ("friction_high", {"friction_scale": 1.3, **MILD_NOISE}, (7, 17, 29)),
    ("mass_heavy", {"mass_scale": 1.05, **MILD_NOISE}, (7, 17, 29)),
    ("right_heavy_1pct", {"leg_mass_asymmetry": -0.01, **MILD_NOISE}, (7, 17, 29)),
    ("left_heavy_05pct", {"leg_mass_asymmetry": 0.005, **MILD_NOISE}, (7, 17, 29)),
)


class EpisodeProgress(BaseCallback):
    def __init__(self) -> None:
        super().__init__()
        self.episodes: list[dict[str, float]] = []

    def _on_step(self) -> bool:
        for info in self.locals.get("infos", []):
            if "episode" in info:
                self.episodes.append({key: float(value) for key, value in info["episode"].items()})
        return True


def evaluate_model(path: Path) -> list[dict[str, object]]:
    policy = PPO.load(path, device="cpu")
    rows = []
    for case_name, params, seeds in VALIDATION_CASES:
        for seed in seeds:
            for first_side in ("left", "right"):
                env = RobustSoftLandingV11Env(fixed_first_side=first_side, **params)
                observation, _ = env.reset(seed=seed)
                total_reward = 0.0
                for step in range(env.config.max_episode_steps):
                    action = policy.predict(observation, deterministic=True)[0]
                    observation, reward, terminated, truncated, info = env.step(action)
                    total_reward += reward
                    if terminated or truncated:
                        break
                rows.append(
                    {
                        "model": path.name,
                        "case": case_name,
                        "first_side": first_side,
                        "seed": seed,
                        "steps": step + 1,
                        "terminated": terminated,
                        "gait_success": bool(info["gait_success"]),
                        "impact_limit_satisfied": bool(info["impact_limit_satisfied"]),
                        "success": bool(info["is_success"]),
                        "maximum_landing_force_n": float(info["maximum_landing_force_n"]),
                        "base_forward_displacement_m": float(info["base_forward_displacement_m"]),
                        "total_reward": total_reward,
                    }
                )
                env.close()
    return rows


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--n-envs", type=int, default=4)
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--learning-rate", type=float, default=1.0e-5)
    parser.add_argument(
        "--initial-model",
        type=Path,
        default=ROOT / "results/khr3hv_v10_symmetric/ppo_soft_landing_final.zip",
    )
    parser.add_argument("--output", type=Path, default=ROOT / "results/khr3hv_v12_curriculum")
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    env = make_vec_env(
        lambda: Monitor(CurriculumSoftLandingV12Env(curriculum_stage=0)),
        n_envs=args.n_envs,
        seed=args.seed,
    )
    initial_path = args.output / "ppo_curriculum_initial.zip"
    PPO.load(args.initial_model, device="cpu").save(initial_path)
    model = PPO.load(
        args.initial_model,
        env=env,
        device="cpu",
        learning_rate=args.learning_rate,
        n_steps=1200,
        batch_size=128,
        n_epochs=2,
        clip_range=0.05,
        target_kl=0.002,
    )
    progress = EpisodeProgress()
    stage_steps = (9_600, 19_200, 24_000)
    candidate_paths = [initial_path]
    stage_records = []
    started = time.time()
    for stage, timesteps in enumerate(stage_steps):
        env.env_method("set_curriculum_stage", stage)
        before = model.num_timesteps
        model.learn(
            timesteps,
            callback=progress,
            progress_bar=False,
            reset_num_timesteps=stage == 0,
        )
        path = args.output / f"ppo_curriculum_stage{stage}.zip"
        model.save(path)
        candidate_paths.append(path)
        recent = progress.episodes[-20:]
        stage_record = {
            "stage": stage,
            "requested_timesteps": timesteps,
            "cumulative_timesteps": model.num_timesteps,
            "stage_actual_timesteps": model.num_timesteps - (0 if stage == 0 else before),
            "episodes_seen": len(progress.episodes),
            "recent_reward": float(np.mean([row["r"] for row in recent])) if recent else None,
            "recent_length": float(np.mean([row["l"] for row in recent])) if recent else None,
            "model": path.name,
        }
        stage_records.append(stage_record)
        print(json.dumps(stage_record, allow_nan=False), flush=True)

    comparison = [row for path in candidate_paths for row in evaluate_model(path)]
    grouped = {
        path: [row for row in comparison if row["model"] == path.name]
        for path in candidate_paths
    }

    def selection_key(path: Path) -> tuple[float, ...]:
        rows = grouped[path]
        nominal = [row for row in rows if row["case"] == "nominal"]
        completed = [row for row in rows if not row["terminated"]]
        return (
            float(sum(row["success"] for row in nominal)),
            float(sum(row["success"] for row in rows)),
            float(-sum(row["terminated"] for row in rows)),
            float(sum(row["impact_limit_satisfied"] for row in completed)),
            float(min(row["total_reward"] for row in nominal)),
        )

    selected_path = max(candidate_paths, key=selection_key)
    PPO.load(selected_path, device="cpu").save(args.output / "ppo_curriculum_final")
    (args.output / "validation_comparison.json").write_text(
        json.dumps(comparison, indent=2, allow_nan=False), encoding="utf-8"
    )
    scores = []
    for path in candidate_paths:
        rows = grouped[path]
        scores.append(
            {
                "model": path.name,
                "nominal_successes": sum(
                    row["success"] for row in rows if row["case"] == "nominal"
                ),
                "total_successes": sum(row["success"] for row in rows),
                "runs": len(rows),
                "falls": sum(row["terminated"] for row in rows),
                "impact_failures": sum(
                    not row["impact_limit_satisfied"] and not row["terminated"] for row in rows
                ),
                "selection_key": list(selection_key(path)),
            }
        )
    summary = {
        "source_model": str(args.initial_model),
        "elapsed_seconds": time.time() - started,
        "learning_rate": args.learning_rate,
        "n_steps": 1200,
        "n_epochs": 2,
        "clip_range": 0.05,
        "target_kl": 0.002,
        "stage_records": stage_records,
        "validation_runs_per_model": len(grouped[candidate_paths[0]]),
        "candidate_scores": scores,
        "selected_model": selected_path.name,
        "selection_rule": (
            "nominal successes, total robustness successes, fewer falls, impact successes, "
            "then lower-side nominal reward"
        ),
    }
    (args.output / "training_summary.json").write_text(
        json.dumps(summary, indent=2, allow_nan=False), encoding="utf-8"
    )
    env.close()
    print(json.dumps(summary, indent=2, allow_nan=False), flush=True)


if __name__ == "__main__":
    main()

