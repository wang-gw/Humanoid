#!/usr/bin/env python3
"""End-to-end walking policy training with checkpoint support.

체크포인트마다 자동 저장 → 컴퓨터 꺼도 이어서 학습 가능.

Usage:
  # 처음 시작
  python scripts/train_e2e_walking.py --run-name e2e_walk_v1 --total-timesteps 2000000

  # 이어서 (같은 명령 그대로)
  python scripts/train_e2e_walking.py --run-name e2e_walk_v1 --total-timesteps 2000000
"""

from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path

os.environ.setdefault("MUJOCO_GL", "egl")
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import numpy as np
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import BaseCallback, CheckpointCallback
from stable_baselines3.common.vec_env import DummyVecEnv, VecNormalize

from envs.gait_walking_env import GaitWalkingEnv
from envs.urdf_f_v2_env import GaitWalkingV2Env
from envs.urdf_f_env import UrdfFEnv

# Env class used for training. `gait` = old urdf_f model (defaults below);
# `gait_v2` = new urdf_f_v2 model, which pins its own model/pose/foot/reward.
ENV_CLASSES = {"gait": GaitWalkingEnv, "gait_v2": GaitWalkingV2Env}
ENV_CLASS = GaitWalkingEnv

TRAIN_DIR = Path("outputs/train/urdf_f")
MODEL_PATH = "envs/robots/urdf_f_link/URDF_F_link_virtual_stl_footprint_contact.xml"
POSE_PATH = "configs/symmetric_standing_pose.json"
# Foot body names the env uses for contact/clearance sensing. Defaults match the
# old urdf_f model; the urdf_f_v2 model names its right foot "foot_R_1".
LEFT_FOOT_BODY = "foot_L_1"
RIGHT_FOOT_BODY = "foot_R_v1_1"
# Nominal attitude-stabilizer signs. Defaults match the old urdf_f model; the
# urdf_f_v2 model has flipped pitch-joint axes, so it needs roll_sign=-1.
NOMINAL_ROLL_SIGN = 1.0
NOMINAL_PITCH_SIGN = -1.0
NOMINAL_KCOM = 1.0


class SaveVecNormCallback(BaseCallback):
    """Saves VecNormalize stats alongside each model checkpoint."""

    def __init__(self, save_freq: int, ckpt_dir: Path, vec_env: VecNormalize):
        super().__init__()
        self.save_freq = save_freq
        self.ckpt_dir = ckpt_dir
        self.vec_env = vec_env

    def _on_step(self) -> bool:
        if self.num_timesteps % self.save_freq == 0:
            path = self.ckpt_dir / f"vecnorm_{self.num_timesteps}.pkl"
            self.vec_env.save(str(path))
        return True


def find_latest_checkpoint(ckpt_dir: Path):
    """Return (model_path, vecnorm_path, timestep) of the latest checkpoint, or None.

    Sort NUMERICALLY by timestep — lexicographic sort is wrong (e.g. 'ppo_950000'
    would rank above 'ppo_2500000' because '9' > '2').
    """
    models = sorted(ckpt_dir.glob("ppo_*_steps.zip"), key=lambda p: int(p.stem.split("_")[1]))
    if not models:
        return None, None, 0
    latest_model = models[-1]
    # filename: ppo_<N>_steps.zip
    timestep = int(latest_model.stem.split("_")[1])
    vecnorms = sorted(ckpt_dir.glob("vecnorm_*.pkl"), key=lambda p: int(p.stem.split("_")[1]))
    latest_vecnorm = vecnorms[-1] if vecnorms else None
    return latest_model, latest_vecnorm, timestep


def make_env_fn(flat_foot_weight: float = 0.0, swing_step_weight: float = 0.0,
                weight_shift_weight: float = 0.0, swing_contact_weight: float = 0.0,
                foot_split_weight: float = 0.0, jitter_qpos: float = 0.0,
                jitter_qvel: float = 0.0, **kwargs):
    def _init():
        env_kwargs = dict(
            task="walking",
            max_episode_steps=1000,
            frame_skip=10,
            action_scale=0.15,
            upright_penalty_weight=20.0,
            action_penalty_weight=0.05,
            action_delta_penalty_weight=0.02,
            flat_foot_penalty_weight=flat_foot_weight,
            swing_step_reward_weight=swing_step_weight,
            weight_shift_reward_weight=weight_shift_weight,
            swing_contact_penalty_weight=swing_contact_weight,
            foot_split_penalty_weight=foot_split_weight,
            jitter_qpos_std=jitter_qpos,
            jitter_qvel_std=jitter_qvel,
            fall_penalty=20.0,
            termination_roll_limit=0.55,
            termination_pitch_limit=0.55,
            gait_period_steps=400,
        )
        # GaitWalkingV2Env pins its own model/pose/foot defaults; only the base
        # `gait` class takes them from this script's globals (old urdf_f model).
        if ENV_CLASS is GaitWalkingEnv:
            env_kwargs.update(
                model_path=MODEL_PATH,
                pose_path=POSE_PATH,
                left_foot_body=LEFT_FOOT_BODY,
                right_foot_body=RIGHT_FOOT_BODY,
                nominal_roll_sign=NOMINAL_ROLL_SIGN,
                nominal_pitch_sign=NOMINAL_PITCH_SIGN,
                nominal_kcom=NOMINAL_KCOM,
            )
        return ENV_CLASS(**env_kwargs)
    return _init


def train(run_name: str, total_timesteps: int, n_envs: int, checkpoint_freq: int,
          flat_foot_weight: float = 0.0, swing_step_weight: float = 0.0,
          weight_shift_weight: float = 0.0, swing_contact_weight: float = 0.0,
          foot_split_weight: float = 0.0, jitter_qpos: float = 0.0, jitter_qvel: float = 0.0,
          ent_coef: float = 0.01, init_from: str | None = None):
    run_dir = TRAIN_DIR / run_name
    run_dir.mkdir(parents=True, exist_ok=True)
    ckpt_dir = run_dir / "checkpoints"
    ckpt_dir.mkdir(exist_ok=True)

    latest_model, latest_vecnorm, start_ts = find_latest_checkpoint(ckpt_dir)
    remaining = total_timesteps - start_ts

    if remaining <= 0:
        print(f"Already trained {start_ts}/{total_timesteps} steps. Done.")
        return

    env_fns = [make_env_fn(flat_foot_weight=flat_foot_weight, swing_step_weight=swing_step_weight,
                           weight_shift_weight=weight_shift_weight, swing_contact_weight=swing_contact_weight,
                           foot_split_weight=foot_split_weight, jitter_qpos=jitter_qpos, jitter_qvel=jitter_qvel)
               for _ in range(n_envs)]
    vec_env = DummyVecEnv(env_fns)

    # Warm-start a fresh run from another run's policy+vecnorm (e.g. v2 from v1).
    if latest_model is None and init_from:
        init_dir = TRAIN_DIR / init_from
        init_policy = init_dir / "ppo_policy.zip"
        init_vecnorm = init_dir / "vecnormalize.pkl"
        print(f"Warm-starting {run_name} from {init_from} ({init_policy.name})")
        vec_env = VecNormalize.load(str(init_vecnorm), vec_env)
        vec_env.training = True
        model = PPO.load(str(init_policy), env=vec_env, device="cpu")
        model.num_timesteps = 0
        _run_learn(model, vec_env, ckpt_dir, run_dir, remaining, n_envs, checkpoint_freq)
        return

    if latest_model is not None:
        print(f"Resuming from {latest_model.name}  ({start_ts}/{total_timesteps} steps, {remaining} remaining)")
        if latest_vecnorm:
            vec_env = VecNormalize.load(str(latest_vecnorm), vec_env)
            print(f"  Loaded VecNorm: {latest_vecnorm.name}")
        else:
            vec_env = VecNormalize(vec_env, norm_obs=True, norm_reward=True, clip_obs=10.0)
        vec_env.training = True
        model = PPO.load(str(latest_model), env=vec_env, device="cpu")
        model.num_timesteps = start_ts
    else:
        print(f"Starting fresh training: {run_name} ({total_timesteps} steps)")
        vec_env = VecNormalize(vec_env, norm_obs=True, norm_reward=True, clip_obs=10.0)
        model = PPO(
            "MlpPolicy",
            vec_env,
            verbose=1,
            device="cpu",
            n_steps=2048,
            batch_size=256,
            n_epochs=10,
            learning_rate=3e-4,
            gamma=0.99,
            gae_lambda=0.95,
            clip_range=0.2,
            ent_coef=ent_coef,
            policy_kwargs=dict(net_arch=[256, 256]),
        )

    _run_learn(model, vec_env, ckpt_dir, run_dir, remaining, n_envs, checkpoint_freq)


def _run_learn(model, vec_env, ckpt_dir, run_dir, remaining, n_envs, checkpoint_freq):
    save_freq_per_env = max(checkpoint_freq // n_envs, 1)
    checkpoint_cb = CheckpointCallback(
        save_freq=save_freq_per_env,
        save_path=str(ckpt_dir),
        name_prefix="ppo",
    )
    vecnorm_cb = SaveVecNormCallback(checkpoint_freq, ckpt_dir, vec_env)

    model.learn(
        total_timesteps=remaining,
        callback=[checkpoint_cb, vecnorm_cb],
        reset_num_timesteps=False,
    )

    model.save(str(run_dir / "ppo_policy"))
    vec_env.save(str(run_dir / "vecnormalize.pkl"))
    print(f"\nSaved to {run_dir}")


def evaluate(run_name: str, n_cycles: int = 3):
    """Quick eval of the trained end-to-end policy."""
    run_dir = TRAIN_DIR / run_name
    dummy = DummyVecEnv([make_env_fn()])
    vec_env = VecNormalize.load(str(run_dir / "vecnormalize.pkl"), dummy)
    vec_env.training = False
    vec_env.norm_reward = False
    model = PPO.load(str(run_dir / "ppo_policy"), env=vec_env, device="cpu")

    eval_kwargs = dict(task="walking", max_episode_steps=n_cycles * 800,
                       frame_skip=10, gait_period_steps=400)
    if ENV_CLASS is GaitWalkingEnv:
        eval_kwargs.update(model_path=MODEL_PATH, pose_path=POSE_PATH,
                           left_foot_body=LEFT_FOOT_BODY, right_foot_body=RIGHT_FOOT_BODY)
    env = ENV_CLASS(**eval_kwargs)
    obs, _ = env.reset()
    init_ly = env._left_foot_y()
    init_ry = env._right_foot_y()

    total_steps = n_cycles * 800
    for s in range(total_steps):
        obs_n = vec_env.normalize_obs(obs[None])[0]
        action, _ = model.predict(obs_n, deterministic=True)
        obs, _, term, trunc, info = env.step(action)
        if term or trunc:
            print(f"Ended at step {s}: {'fell' if term else 'truncated'}")
            break

    net_l = env._left_foot_y() - init_ly
    net_r = env._right_foot_y() - init_ry
    print(f"L_net={net_l*100:+.1f}cm  R_net={net_r*100:+.1f}cm  avg={((net_l+net_r)/2)*100:.1f}cm")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--run-name", default="e2e_walk_v1")
    parser.add_argument("--env", choices=list(ENV_CLASSES), default="gait",
                        help="Env/model: 'gait' = old urdf_f, 'gait_v2' = new urdf_f_v2 (pins its own model/pose/foot/reward).")
    parser.add_argument("--model-path", default=None,
                        help="Override robot MJCF path (e.g. urdf_f_v2 footprint contact model).")
    parser.add_argument("--pose-path", default=None,
                        help="Override standing pose JSON path.")
    parser.add_argument("--train-dir", default=None,
                        help="Override output dir (e.g. outputs/train/urdf_f_v2) to separate model runs.")
    parser.add_argument("--left-foot-body", default=None,
                        help="Left foot body name for contact sensing (default foot_L_1).")
    parser.add_argument("--right-foot-body", default=None,
                        help="Right foot body name (urdf_f_v2 uses foot_R_1; default foot_R_v1_1).")
    parser.add_argument("--nominal-roll-sign", type=float, default=None,
                        help="Attitude-stabilizer roll sign (urdf_f_v2 needs -1; default +1).")
    parser.add_argument("--nominal-pitch-sign", type=float, default=None,
                        help="Attitude-stabilizer pitch sign (default -1).")
    parser.add_argument("--nominal-kcom", type=float, default=None,
                        help="Attitude-stabilizer CoM feedback gain (default 1.0).")
    parser.add_argument("--total-timesteps", type=int, default=2_000_000)
    parser.add_argument("--n-envs", type=int, default=8)
    parser.add_argument("--checkpoint-freq", type=int, default=50_000)
    parser.add_argument("--flat-foot-weight", type=float, default=0.0,
                        help="Penalty weight for a grounded foot whose sole is not flat.")
    parser.add_argument("--swing-step-weight", type=float, default=0.0,
                        help="Reward weight for the swinging foot moving forward (breaks limp asymmetry).")
    parser.add_argument("--weight-shift-weight", type=float, default=0.0,
                        help="Reward weight for loading the stance foot so the swing foot can lift.")
    parser.add_argument("--swing-contact-weight", type=float, default=0.0,
                        help="Penalty weight for the swing foot staying in ground contact during its phase.")
    parser.add_argument("--foot-split-weight", type=float, default=0.0,
                        help="Penalty weight for a persistent fore-aft gap between the feet (anti-lunge).")
    parser.add_argument("--jitter-qpos", type=float, default=0.0,
                        help="Std of initial joint-angle jitter for robustness.")
    parser.add_argument("--jitter-qvel", type=float, default=0.0,
                        help="Std of initial joint-velocity jitter for robustness.")
    parser.add_argument("--ent-coef", type=float, default=0.01,
                        help="PPO entropy coefficient (fresh runs only); raise for more exploration.")
    parser.add_argument("--init-from", default=None,
                        help="Warm-start a fresh run from another run's policy+vecnorm (e.g. e2e_walk_v1).")
    parser.add_argument("--eval", action="store_true")
    args = parser.parse_args()

    ENV_CLASS = ENV_CLASSES[args.env]

    # Apply model/pose overrides onto module globals used by make_env_fn/evaluate.
    if args.model_path:
        MODEL_PATH = args.model_path
    if args.pose_path:
        POSE_PATH = args.pose_path
    if args.train_dir:
        TRAIN_DIR = Path(args.train_dir)
    if args.left_foot_body:
        LEFT_FOOT_BODY = args.left_foot_body
    if args.right_foot_body:
        RIGHT_FOOT_BODY = args.right_foot_body
    if args.nominal_roll_sign is not None:
        NOMINAL_ROLL_SIGN = args.nominal_roll_sign
    if args.nominal_pitch_sign is not None:
        NOMINAL_PITCH_SIGN = args.nominal_pitch_sign
    if args.nominal_kcom is not None:
        NOMINAL_KCOM = args.nominal_kcom

    if args.eval:
        evaluate(args.run_name)
    else:
        train(args.run_name, args.total_timesteps, args.n_envs, args.checkpoint_freq,
              flat_foot_weight=args.flat_foot_weight, swing_step_weight=args.swing_step_weight,
              weight_shift_weight=args.weight_shift_weight, swing_contact_weight=args.swing_contact_weight,
              foot_split_weight=args.foot_split_weight, jitter_qpos=args.jitter_qpos,
              jitter_qvel=args.jitter_qvel, ent_coef=args.ent_coef, init_from=args.init_from)
