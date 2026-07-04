from __future__ import annotations

import argparse
import json
import sys
from datetime import datetime
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from envs.urdf_f_env import DEFAULT_MODEL, DEFAULT_POSE, UrdfFEnv


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Train a PPO policy on the URDF_F humanoid RL environment.")
    parser.add_argument("--model", type=Path, default=DEFAULT_MODEL)
    parser.add_argument("--pose-json", type=Path, default=DEFAULT_POSE)
    parser.add_argument("--nominal-pose-json", type=Path, default=None)
    parser.add_argument(
        "--task",
        choices=[
            "standing",
            "weight_shift_left",
            "weight_shift_right",
            "right_unload",
            "left_unload",
            "right_clearance",
            "left_clearance",
            "right_return",
            "left_return",
        ],
        default="standing",
    )
    parser.add_argument("--total-timesteps", type=int, default=100_000)
    parser.add_argument("--n-envs", type=int, default=8)
    parser.add_argument("--max-episode-steps", type=int, default=500)
    parser.add_argument("--frame-skip", type=int, default=10)
    parser.add_argument("--action-scale", type=float, default=0.35)
    parser.add_argument("--upright-penalty-weight", type=float, default=8.0)
    parser.add_argument("--action-penalty-weight", type=float, default=0.01)
    parser.add_argument("--action-delta-penalty-weight", type=float, default=0.0)
    parser.add_argument("--right-contact-penalty-weight", type=float, default=0.0)
    parser.add_argument("--left-contact-penalty-weight", type=float, default=0.0)
    parser.add_argument("--clearance-reward-weight", type=float, default=2.0)
    parser.add_argument("--clearance-target", type=float, default=0.002)
    parser.add_argument("--gated-clearance-reward", action="store_true")
    parser.add_argument("--clearance-gate-roll", type=float, default=0.12)
    parser.add_argument("--clearance-gate-pitch", type=float, default=0.12)
    parser.add_argument("--fall-penalty", type=float, default=0.0)
    parser.add_argument("--stability-excess-penalty-weight", type=float, default=0.0)
    parser.add_argument("--termination-roll-limit", type=float, default=0.55)
    parser.add_argument("--termination-pitch-limit", type=float, default=0.55)
    parser.add_argument("--termination-base-drop", type=float, default=0.12)
    parser.add_argument("--seed", type=int, default=1)
    parser.add_argument("--device", type=str, default="auto")
    parser.add_argument("--save-dir", type=Path, default=Path("outputs/train/urdf_f"))
    parser.add_argument("--run-name", type=str, default="")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    try:
        from stable_baselines3 import PPO
        from stable_baselines3.common.monitor import Monitor
        from stable_baselines3.common.vec_env import DummyVecEnv, VecNormalize
    except ImportError as exc:
        raise SystemExit(
            "stable-baselines3 is required. Install project requirements before training."
        ) from exc

    run_name = args.run_name or f"{datetime.now().strftime('%Y%m%d_%H%M%S')}_{args.task}"
    run_dir = args.save_dir / run_name
    run_dir.mkdir(parents=True, exist_ok=True)

    def make_env(rank: int):
        def _factory():
            env = UrdfFEnv(
                model_path=args.model,
                pose_path=args.pose_json,
                nominal_pose_path=args.nominal_pose_json,
                task=args.task,
                max_episode_steps=args.max_episode_steps,
                frame_skip=args.frame_skip,
                action_scale=args.action_scale,
                upright_penalty_weight=args.upright_penalty_weight,
                action_penalty_weight=args.action_penalty_weight,
                action_delta_penalty_weight=args.action_delta_penalty_weight,
                right_contact_penalty_weight=args.right_contact_penalty_weight,
                left_contact_penalty_weight=args.left_contact_penalty_weight,
                clearance_reward_weight=args.clearance_reward_weight,
                clearance_target=args.clearance_target,
                gated_clearance_reward=args.gated_clearance_reward,
                clearance_gate_roll=args.clearance_gate_roll,
                clearance_gate_pitch=args.clearance_gate_pitch,
                fall_penalty=args.fall_penalty,
                stability_excess_penalty_weight=args.stability_excess_penalty_weight,
                termination_roll_limit=args.termination_roll_limit,
                termination_pitch_limit=args.termination_pitch_limit,
                termination_base_drop=args.termination_base_drop,
            )
            return Monitor(env)

        return _factory

    env = DummyVecEnv([make_env(i) for i in range(args.n_envs)])
    env = VecNormalize(env, norm_obs=True, norm_reward=True, clip_obs=10.0)

    config = {
        "model": str(args.model.resolve()),
        "pose_json": str(args.pose_json.resolve()),
        "nominal_pose_json": str(args.nominal_pose_json.resolve()) if args.nominal_pose_json else None,
        "task": args.task,
        "total_timesteps": args.total_timesteps,
        "n_envs": args.n_envs,
        "max_episode_steps": args.max_episode_steps,
        "frame_skip": args.frame_skip,
        "action_scale": args.action_scale,
        "upright_penalty_weight": args.upright_penalty_weight,
        "action_penalty_weight": args.action_penalty_weight,
        "action_delta_penalty_weight": args.action_delta_penalty_weight,
        "right_contact_penalty_weight": args.right_contact_penalty_weight,
        "left_contact_penalty_weight": args.left_contact_penalty_weight,
        "clearance_reward_weight": args.clearance_reward_weight,
        "clearance_target": args.clearance_target,
        "gated_clearance_reward": args.gated_clearance_reward,
        "clearance_gate_roll": args.clearance_gate_roll,
        "clearance_gate_pitch": args.clearance_gate_pitch,
        "fall_penalty": args.fall_penalty,
        "stability_excess_penalty_weight": args.stability_excess_penalty_weight,
        "termination_roll_limit": args.termination_roll_limit,
        "termination_pitch_limit": args.termination_pitch_limit,
        "termination_base_drop": args.termination_base_drop,
        "seed": args.seed,
        "device": args.device,
    }
    (run_dir / "config.json").write_text(json.dumps(config, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")

    model = PPO(
        "MlpPolicy",
        env,
        verbose=1,
        seed=args.seed,
        device=args.device,
        n_steps=512,
        batch_size=256,
        learning_rate=3e-4,
        gamma=0.99,
        gae_lambda=0.95,
        clip_range=0.2,
        ent_coef=0.0,
    )
    model.learn(total_timesteps=args.total_timesteps, progress_bar=False)
    model.save(run_dir / "ppo_policy")
    env.save(str(run_dir / "vecnormalize.pkl"))
    env.close()
    print(json.dumps({"run_dir": str(run_dir.resolve()), "policy": str((run_dir / "ppo_policy.zip").resolve())}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
