#!/usr/bin/env python3
"""Transfer V20 into the phase-split V21 corrector and train it."""

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

from humanoidv2 import (  # noqa: E402
    ContactAwareResidualV20Env,
    PhaseSplitResidualV21Env,
)
from scripts.train_landing_residual_v19 import Progress, combined_domain  # noqa: E402


BASE = ROOT / "results/khr3hv_v17_forward_margin/ppo_forward_margin_v17_final.zip"
SOURCE = ROOT / "results/khr3hv_v20_contact_aware/ppo_contact_aware_v20_final.zip"
OUTPUT = ROOT / "results/khr3hv_v21_phase_split"
EARLY_GATE_BLEND = 0.75


def transfer_v20_policy(source: PPO, target: PPO) -> None:
    """Expand 90/10 V20 into 94/20 V21 without changing its initial mean action."""
    source_state = source.policy.state_dict()
    target_state = target.policy.state_dict()
    expanded_inputs = {
        "mlp_extractor.policy_net.0.weight",
        "mlp_extractor.value_net.0.weight",
    }
    duplicated_outputs = {"action_net.weight", "action_net.bias", "log_std"}
    with torch.no_grad():
        for name, target_value in target_state.items():
            source_value = source_state[name]
            if name in expanded_inputs:
                target_value.zero_()
                target_value[:, : source_value.shape[1]].copy_(source_value)
            elif name in duplicated_outputs:
                target_value[:10].copy_(source_value)
                target_value[10:].copy_(source_value)
            else:
                target_value.copy_(source_value)
    target.policy.load_state_dict(target_state)


def verify_exact_transfer(
    source: PPO, target: PPO, base_path: Path, seed: int = 17
) -> dict[str, float | int | bool]:
    """Compare a complete nominal V20 and V21 rollout before V21 training."""
    base_v20 = PPO.load(base_path, device="cpu")
    base_v21 = PPO.load(base_path, device="cpu")
    v20 = ContactAwareResidualV20Env(
        base_policy=base_v20,
        fixed_first_side="left",
        fixed_domain={},
        curriculum_stage=2,
        early_gate_blend=EARLY_GATE_BLEND,
    )
    v21 = PhaseSplitResidualV21Env(
        base_policy=base_v21,
        fixed_first_side="left",
        fixed_domain={},
        curriculum_stage=2,
        early_gate_blend=EARLY_GATE_BLEND,
        conditional_lift_gate_scale=0.0,
    )
    obs20, _ = v20.reset(seed=seed)
    obs21, _ = v21.reset(seed=seed)
    maximum_source_head_difference = 0.0
    maximum_head_pair_difference = 0.0
    maximum_qpos_difference = 0.0
    maximum_forward_difference = 0.0
    maximum_impact_difference = 0.0
    steps = 0
    for steps in range(1, v20.config.max_episode_steps + 1):
        action20 = np.asarray(source.predict(obs20, deterministic=True)[0])
        action21 = np.asarray(target.predict(obs21, deterministic=True)[0])
        maximum_source_head_difference = max(
            maximum_source_head_difference,
            float(np.max(np.abs(action20 - action21[:10]))),
            float(np.max(np.abs(action20 - action21[10:]))),
        )
        maximum_head_pair_difference = max(
            maximum_head_pair_difference,
            float(np.max(np.abs(action21[:10] - action21[10:]))),
        )
        obs20, _, terminated20, truncated20, info20 = v20.step(action20)
        obs21, _, terminated21, truncated21, info21 = v21.step(action21)
        maximum_qpos_difference = max(
            maximum_qpos_difference,
            float(np.max(np.abs(v20.data.qpos - v21.data.qpos))),
        )
        maximum_forward_difference = max(
            maximum_forward_difference,
            abs(
                float(info20["base_forward_displacement_m"])
                - float(info21["base_forward_displacement_m"])
            ),
        )
        maximum_impact_difference = max(
            maximum_impact_difference,
            abs(
                float(info20["maximum_landing_force_n"])
                - float(info21["maximum_landing_force_n"])
            ),
        )
        if terminated20 or truncated20 or terminated21 or truncated21:
            if (terminated20, truncated20) != (terminated21, truncated21):
                raise RuntimeError("V20 and transferred V21 ended at different times")
            break
    v20.close()
    v21.close()
    tolerance = 1.0e-7
    result: dict[str, float | int | bool] = {
        "steps": steps,
        "maximum_source_head_difference": maximum_source_head_difference,
        "maximum_head_pair_difference": maximum_head_pair_difference,
        "maximum_qpos_difference": maximum_qpos_difference,
        "maximum_forward_difference_m": maximum_forward_difference,
        "maximum_impact_difference_n": maximum_impact_difference,
        "tolerance": tolerance,
        "passed": max(
            maximum_source_head_difference,
            maximum_head_pair_difference,
            maximum_qpos_difference,
            maximum_forward_difference,
            maximum_impact_difference,
        )
        <= tolerance,
    }
    if not result["passed"]:
        raise RuntimeError(f"V20-to-V21 transfer changed the rollout: {result}")
    return result


def run_episode(
    corrector: PPO,
    base: PPO,
    side: str,
    seed: int,
    domain: dict[str, float | int],
    conditional_lift_gate_scale: float = 0.35,
) -> dict[str, object]:
    env = PhaseSplitResidualV21Env(
        base_policy=base,
        fixed_first_side=side,
        fixed_domain=domain,
        curriculum_stage=2,
        early_gate_blend=EARLY_GATE_BLEND,
        conditional_lift_gate_scale=conditional_lift_gate_scale,
    )
    observation, _ = env.reset(seed=seed)
    maximum_correction = 0.0
    maximum_lift_gate = 0.0
    terminated = truncated = False
    for step in range(env.config.max_episode_steps):
        action = corrector.predict(observation, deterministic=True)[0]
        observation, _, terminated, truncated, info = env.step(action)
        maximum_correction = max(
            maximum_correction, float(info["applied_correction_max_abs"])
        )
        if str(info["task_phase"]).endswith("_lift"):
            maximum_lift_gate = max(
                maximum_lift_gate, float(info["landing_correction_gate"])
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
        "maximum_conditional_lift_gate": maximum_lift_gate,
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


def selection_key(rows: list[dict[str, object]]) -> tuple[float, ...]:
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


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--base-policy", type=Path, default=BASE)
    parser.add_argument("--source", type=Path, default=SOURCE)
    parser.add_argument("--output", type=Path, default=OUTPUT)
    parser.add_argument("--n-envs", type=int, default=4)
    parser.add_argument("--seed", type=int, default=59)
    parser.add_argument("--learning-rate", type=float, default=5.0e-5)
    parser.add_argument(
        "--stage-timesteps", type=int, nargs=3, default=(40_000, 60_000, 80_000)
    )
    parser.add_argument("--verify-only", action="store_true")
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)

    def build_env():
        base = PPO.load(args.base_policy, device="cpu")
        return Monitor(
            PhaseSplitResidualV21Env(
                base_policy=base,
                curriculum_stage=2,
                early_gate_blend=EARLY_GATE_BLEND,
                conditional_lift_gate_scale=0.0,
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
    transfer_v20_policy(source, model)
    verification = verify_exact_transfer(source, model, args.base_policy)
    (args.output / "transfer_verification.json").write_text(
        json.dumps(verification, indent=2, allow_nan=False), encoding="utf-8"
    )
    print(json.dumps({"transfer_verification": verification}, indent=2), flush=True)
    initial_path = args.output / "ppo_phase_split_v21_initial.zip"
    model.save(initial_path)
    if args.verify_only:
        env.close()
        return

    candidates = [initial_path]
    progress = Progress()
    stage_records: list[dict[str, object]] = []
    started = time.time()
    for stage, timesteps in enumerate(args.stage_timesteps):
        env.env_method("set_v21_stage", stage)
        model.learn(
            timesteps,
            callback=progress,
            progress_bar=False,
            reset_num_timesteps=stage == 0,
        )
        path = args.output / f"ppo_phase_split_v21_stage{stage}.zip"
        model.save(path)
        candidates.append(path)
        recent = progress.episodes[-20:]
        record: dict[str, object] = {
            "stage": stage,
            "domain_stage": (2, 3, 2)[stage],
            "conditional_lift_gate_scale": (0.0, 0.20, 0.35)[stage],
            "requested_timesteps": timesteps,
            "cumulative_timesteps": model.num_timesteps,
            "episodes_seen": len(progress.episodes),
            "recent_reward": float(np.mean([row["r"] for row in recent]))
            if recent
            else None,
            "recent_length": float(np.mean([row["l"] for row in recent]))
            if recent
            else None,
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
    selected = max(candidates, key=lambda path: selection_key(grouped[path]))
    PPO.load(selected, device="cpu").save(args.output / "ppo_phase_split_v21_final")
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
                    all(
                        row["success"]
                        for row in combined
                        if row["sample"] == sample
                    )
                    for sample in range(1, 21)
                ),
                "worst_landing_force_n": max(
                    float(row["maximum_landing_force_n"]) for row in combined
                ),
                "minimum_forward_displacement_m": min(
                    float(row["base_forward_displacement_m"]) for row in rows
                ),
                "selection_key": list(selection_key(rows)),
            }
        )
    summary = {
        "base_policy": str(args.base_policy),
        "source_v20_corrector": str(args.source),
        "observation_size": 94,
        "action_size": 20,
        "new_input_weights_initialized_to_zero": True,
        "action_heads_initialized_identically": True,
        "transfer_verification": verification,
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
