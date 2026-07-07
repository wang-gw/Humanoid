"""Train forward walking policies with step placement reward.

Stage 1: left_clearance with forward swing reward (from wsr_v9 warm state)
Stage 2: left_return with landing position reward (from left_clearance end state)
Stage 3: weight_shift_left with right-foot forward swing reward (from symmetric standing)
Stage 4: right_return with landing position reward (from wsl end state)

Usage:
    python scripts/train_forward_walk.py --stage left_clearance --total-timesteps 200000
    python scripts/train_forward_walk.py --stage left_return --total-timesteps 200000
    python scripts/train_forward_walk.py --stage weight_shift_left --total-timesteps 200000
    python scripts/train_forward_walk.py --stage right_return --total-timesteps 200000
"""
from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path

import mujoco
import numpy as np

os.environ.setdefault("MUJOCO_GL", "egl")

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from envs.urdf_f_env import DEFAULT_MODEL, UrdfFEnv
from envs.warm_start_env import WarmStartEnv


TRAIN_DIR = ROOT / "outputs/train/urdf_f"
MODEL_PATH = ROOT / "envs/robots/urdf_f_link/URDF_F_link_virtual_stl_footprint_contact.xml"
POSE_PATH = ROOT / "configs/symmetric_standing_pose.json"


def run_policy_stage(env: UrdfFEnv, policy_path: Path, vecnorm_path: Path,
                     steps: int, task: str, **env_kwargs) -> mujoco.MjData:
    """Run a trained policy for given steps and return the final MjData."""
    from stable_baselines3 import PPO
    from stable_baselines3.common.vec_env import DummyVecEnv, VecNormalize

    old_task = env.task
    for k, v in env_kwargs.items():
        setattr(env, k, v)
    env.task = task

    dummy = DummyVecEnv([lambda: UrdfFEnv()])
    vecnorm = VecNormalize.load(str(vecnorm_path), dummy)
    vecnorm.training = False
    vecnorm.norm_reward = False
    model = PPO.load(str(policy_path), device="cpu")

    obs = env._obs()
    for _ in range(steps):
        obs_norm = vecnorm.normalize_obs(obs[None])[0]
        action, _ = model.predict(obs_norm, deterministic=True)
        obs, _, terminated, _, _ = env.step(action)
        if terminated:
            print(f"  [WARN] stage terminated early at step {_}")
            break

    env.task = old_task
    result = mujoco.MjData(env.model)
    mujoco.mj_copyData(result, env.model, env.data)
    env.prev_action[:] = 0.0
    return result


def get_wsr_v9_warm_data():
    """Run sequence up to end of wsr_v9 (100 steps) to get warm_data for left_clearance."""
    print("Capturing WSR_v9 warm state...")
    env = UrdfFEnv(
        model_path=MODEL_PATH,
        pose_path=POSE_PATH,
        task="weight_shift_left",
    )
    obs, _ = env.reset()

    # Stage 1: weight_shift_left (100 steps)
    warm = run_policy_stage(
        env,
        TRAIN_DIR / "weight_shift_left_rcp_150k/ppo_policy.zip",
        TRAIN_DIR / "weight_shift_left_rcp_150k/vecnormalize.pkl",
        steps=100, task="weight_shift_left",
        action_scale=0.15, upright_penalty_weight=24.0,
        action_penalty_weight=0.05, action_delta_penalty_weight=0.02,
        right_contact_penalty_weight=0.30,
    )

    # Stage 2: right_return (170 steps)
    warm = run_policy_stage(
        env,
        TRAIN_DIR / "right_return_from_wsl_rcp_nomstd_150k/ppo_policy.zip",
        TRAIN_DIR / "right_return_from_wsl_rcp_nomstd_150k/vecnormalize.pkl",
        steps=170, task="right_return",
        action_scale=0.12, upright_penalty_weight=36.0,
        action_penalty_weight=0.08, action_delta_penalty_weight=0.03,
        fall_penalty=20.0,
        termination_roll_limit=0.55,
    )
    env.base_z = float(env.data.qpos[2])

    # Stage 3: weight_shift_right / wsr_v9 (100 steps)
    warm = run_policy_stage(
        env,
        TRAIN_DIR / "weight_shift_right_lcp_v9_corrected_200k/ppo_policy.zip",
        TRAIN_DIR / "weight_shift_right_lcp_v9_corrected_200k/vecnormalize.pkl",
        steps=100, task="weight_shift_right",
        action_scale=0.15, upright_penalty_weight=24.0,
        action_penalty_weight=0.05, action_delta_penalty_weight=0.02,
        left_contact_penalty_weight=0.30,
        termination_roll_limit=0.55,
    )
    env.base_z = float(env.data.qpos[2])

    base_z = float(env.data.qpos[2])
    print(f"  WSR_v9 end: base_z={base_z:.4f}, qpos[1](Y)={env.data.qpos[1]:.4f}")
    return mujoco.MjData(env.model).__class__.__mro__[0], base_z, env


def get_wsr_v9_warm_data_v2():
    """Capture warm data at end of wsr_v9 by running the actual sequence."""
    print("Capturing WSR_v9 warm state...")
    from stable_baselines3 import PPO
    from stable_baselines3.common.vec_env import DummyVecEnv, VecNormalize

    env = UrdfFEnv(
        model_path=MODEL_PATH,
        pose_path=POSE_PATH,
        task="weight_shift_left",
    )
    env.reset()

    def run_stage(task, policy_zip, vecnorm_pkl, steps, **kwargs):
        for k, v in kwargs.items():
            setattr(env, k, v)
        env.task = task

        dummy = DummyVecEnv([lambda: UrdfFEnv()])
        vecnorm = VecNormalize.load(str(vecnorm_pkl), dummy)
        vecnorm.training = False
        vecnorm.norm_reward = False
        model = PPO.load(str(policy_zip), device="cpu")

        for _ in range(steps):
            obs = env._obs()
            obs_norm = vecnorm.normalize_obs(obs[None])[0]
            action, _ = model.predict(obs_norm, deterministic=True)
            _, _, terminated, _, _ = env.step(action)
            if terminated:
                print(f"  [WARN] {task} terminated at step {_}")
                break
        env.prev_action[:] = 0.0

    run_stage("weight_shift_left",
              TRAIN_DIR / "weight_shift_left_rcp_150k/ppo_policy.zip",
              TRAIN_DIR / "weight_shift_left_rcp_150k/vecnormalize.pkl",
              100, action_scale=0.15, upright_penalty_weight=24.0,
              action_penalty_weight=0.05, action_delta_penalty_weight=0.02,
              right_contact_penalty_weight=0.30, left_contact_penalty_weight=0.0,
              termination_roll_limit=0.55, termination_pitch_limit=0.55,
              step_target_length=0.0)

    env.base_z = float(env.data.qpos[2])

    run_stage("right_return",
              TRAIN_DIR / "right_return_from_wsl_rcp_nomstd_150k/ppo_policy.zip",
              TRAIN_DIR / "right_return_from_wsl_rcp_nomstd_150k/vecnormalize.pkl",
              170, action_scale=0.12, upright_penalty_weight=36.0,
              action_penalty_weight=0.08, action_delta_penalty_weight=0.03,
              fall_penalty=20.0, left_contact_penalty_weight=0.0,
              right_contact_penalty_weight=0.0, termination_roll_limit=0.55,
              termination_pitch_limit=0.55, step_target_length=0.0)

    env.base_z = float(env.data.qpos[2])

    run_stage("weight_shift_right",
              TRAIN_DIR / "weight_shift_right_lcp_v9_corrected_200k/ppo_policy.zip",
              TRAIN_DIR / "weight_shift_right_lcp_v9_corrected_200k/vecnormalize.pkl",
              100, action_scale=0.15, upright_penalty_weight=24.0,
              action_penalty_weight=0.05, action_delta_penalty_weight=0.02,
              left_contact_penalty_weight=0.30, right_contact_penalty_weight=0.0,
              termination_roll_limit=0.55, termination_pitch_limit=0.55,
              step_target_length=0.0)

    base_z = float(env.data.qpos[2])
    env.base_z = base_z
    warm_data = mujoco.MjData(env.model)
    mujoco.mj_copyData(warm_data, env.model, env.data)
    print(f"  WSR end: base_z={base_z:.4f}, Y={env.data.qpos[1]:.4f}")
    print(f"  left_foot_y={env._left_foot_y():.4f}, right_foot_y={env._right_foot_y():.4f}")
    return warm_data, base_z


def capture_cycle2_wsl_state():
    """Run full cycle 1 forward pipeline to capture cycle-2 wsl starting state."""
    from stable_baselines3 import PPO
    from stable_baselines3.common.vec_env import DummyVecEnv, VecNormalize

    print("Running cycle 1 to capture cycle-2 wsl starting state...")

    def load_pol(d):
        dummy = DummyVecEnv([lambda: UrdfFEnv()])
        vn = VecNormalize.load(str(TRAIN_DIR / d / "vecnormalize.pkl"), dummy)
        vn.training = False; vn.norm_reward = False
        return PPO.load(str(TRAIN_DIR / d / "ppo_policy.zip"), device="cpu"), vn

    env = UrdfFEnv(model_path=MODEL_PATH, pose_path=POSE_PATH, task="weight_shift_left")
    env.reset()

    NO = dict(step_target_length=0.0)
    S5 = dict(step_target_length=0.05, step_reward_weight=1.0)

    stages = [
        ("weight_shift_left_rcp_150k",              100, "weight_shift_left",
         dict(action_scale=0.15, upright_penalty_weight=24.0, action_penalty_weight=0.05,
              action_delta_penalty_weight=0.02, right_contact_penalty_weight=0.30, **NO)),
        ("right_swing_step5cm_200k",                 25, "right_clearance",
         dict(action_scale=0.15, upright_penalty_weight=20.0, action_penalty_weight=0.05,
              action_delta_penalty_weight=0.02, left_contact_penalty_weight=0.10,
              clearance_reward_weight=4.0, clearance_target=0.004, **S5)),
        ("right_return_step5cm_200k",               200, "right_return",
         dict(action_scale=0.12, upright_penalty_weight=36.0, action_penalty_weight=0.08,
              action_delta_penalty_weight=0.03, fall_penalty=20.0, **S5)),
        ("weight_shift_right_lcp_v9_corrected_200k", 100, "weight_shift_right",
         dict(action_scale=0.15, upright_penalty_weight=24.0, action_penalty_weight=0.05,
              action_delta_penalty_weight=0.02, left_contact_penalty_weight=0.30, **NO)),
        ("left_clearance_step5cm_200k",              25, "left_clearance",
         dict(action_scale=0.15, upright_penalty_weight=20.0, action_penalty_weight=0.05,
              action_delta_penalty_weight=0.02, right_contact_penalty_weight=0.10,
              clearance_reward_weight=4.0, clearance_target=0.004, **S5)),
        ("left_return_from_lclr_v2_200k",           200, "left_return",
         dict(action_scale=0.12, upright_penalty_weight=36.0, action_penalty_weight=0.08,
              action_delta_penalty_weight=0.03, fall_penalty=20.0, **NO)),
    ]

    for pol_dir, steps, task, kwargs in stages:
        model, vecnorm = load_pol(pol_dir)
        for k, v in kwargs.items():
            setattr(env, k, v)
        env.task = task; env.prev_action[:] = 0.0
        env.base_z = float(env.data.qpos[2])

        for step in range(steps):
            obs = env._obs()
            obs_n = vecnorm.normalize_obs(obs[None])[0]
            action, _ = model.predict(obs_n, deterministic=True)
            _, _, terminated, _, info = env.step(action)
            if terminated:
                print(f"  [WARN] {task} terminated at step {step}")
                break
        env.prev_action[:] = 0.0
        rclr = info.get("right_clearance", 0.0)
        lclr = info.get("left_clearance", 0.0)
        print(f"  {task:25s} done  Rclr={rclr*1e3:.1f}mm Lclr={lclr*1e3:.1f}mm  Y={env.data.qpos[1]:.4f}")

    base_z = float(env.data.qpos[2])
    env.base_z = base_z
    warm_data = mujoco.MjData(env.model)
    mujoco.mj_copyData(warm_data, env.model, env.data)
    print(f"\n  Cycle2-WSL warm state: base_z={base_z:.4f}")
    print(f"  left_foot_y={env._left_foot_y():.4f}  right_foot_y={env._right_foot_y():.4f}")
    return warm_data, base_z


def get_wsl_fwd_end_state():
    """Run wsl_fwd policy to capture warm state for right_swing_fwd."""
    from stable_baselines3 import PPO
    from stable_baselines3.common.vec_env import DummyVecEnv, VecNormalize

    print("Capturing WSL_fwd warm state...")
    warm_data_c2, base_z_c2 = capture_cycle2_wsl_state()

    env = WarmStartEnv(
        warm_data=warm_data_c2, warm_base_z=base_z_c2,
        jitter_qpos_std=0.0, jitter_qvel_std=0.0,
        model_path=MODEL_PATH, pose_path=POSE_PATH,
        task="weight_shift_left",
        action_scale=0.15, upright_penalty_weight=24.0,
        action_penalty_weight=0.05, action_delta_penalty_weight=0.02,
        right_contact_penalty_weight=0.30, step_target_length=0.0,
    )
    obs, _ = env.reset()

    dummy = DummyVecEnv([lambda: UrdfFEnv()])
    pol_path = TRAIN_DIR / "weight_shift_left_fwd_200k/ppo_policy.zip"
    if not pol_path.exists():
        raise FileNotFoundError(f"Train wsl_fwd first: {pol_path}")
    vecnorm = VecNormalize.load(str(TRAIN_DIR / "weight_shift_left_fwd_200k/vecnormalize.pkl"), dummy)
    vecnorm.training = False; vecnorm.norm_reward = False
    model = PPO.load(str(pol_path), device="cpu")

    max_rclr = 0.0
    for step in range(100):
        obs_n = vecnorm.normalize_obs(obs[None])[0]
        action, _ = model.predict(obs_n, deterministic=True)
        obs, _, term, _, info = env.step(action)
        max_rclr = max(max_rclr, info.get("right_clearance", 0.0))
        if term:
            print(f"  [WARN] wsl_fwd terminated at step {step}")
            break
    env.prev_action[:] = 0.0
    base_z = float(env.data.qpos[2])
    warm_data = mujoco.MjData(env.model)
    mujoco.mj_copyData(warm_data, env.model, env.data)
    print(f"  wsl_fwd end: maxRclr={max_rclr*1e3:.1f}mm  right_Y={env._right_foot_y():.4f}")
    return warm_data, base_z


def get_wsl_end_state():
    """Run wsl_rcp_150k (100 steps) to get warm_data for right_swing."""
    from stable_baselines3 import PPO
    from stable_baselines3.common.vec_env import DummyVecEnv, VecNormalize

    print("Capturing WSL_rcp warm state...")
    env = UrdfFEnv(model_path=MODEL_PATH, pose_path=POSE_PATH, task="weight_shift_left")
    env.reset()

    dummy = DummyVecEnv([lambda: UrdfFEnv()])
    vecnorm = VecNormalize.load(str(TRAIN_DIR / "weight_shift_left_rcp_150k/vecnormalize.pkl"), dummy)
    vecnorm.training = False; vecnorm.norm_reward = False
    model = PPO.load(str(TRAIN_DIR / "weight_shift_left_rcp_150k/ppo_policy.zip"), device="cpu")

    env.action_scale = 0.15; env.upright_penalty_weight = 24.0
    env.action_penalty_weight = 0.05; env.action_delta_penalty_weight = 0.02
    env.right_contact_penalty_weight = 0.30; env.step_target_length = 0.0

    for _ in range(100):
        obs = env._obs()
        obs_norm = vecnorm.normalize_obs(obs[None])[0]
        action, _ = model.predict(obs_norm, deterministic=True)
        _, _, terminated, _, info = env.step(action)
        if terminated:
            print(f"  [WARN] wsl terminated at step {_}")
            break
    env.prev_action[:] = 0.0
    base_z = float(env.data.qpos[2])
    env.base_z = base_z
    warm_data = mujoco.MjData(env.model)
    mujoco.mj_copyData(warm_data, env.model, env.data)
    print(f"  WSL end: base_z={base_z:.4f}, Rclr={info.get('right_clearance',0)*1000:.2f}mm")
    print(f"  right_foot_y={env._right_foot_y():.4f}, left_foot_y={env._left_foot_y():.4f}")
    return warm_data, base_z


def capture_rswing_end_state(step_target: float):
    """Run sequence through right_swing_step5cm to get warm_data for right_return."""
    from stable_baselines3 import PPO
    from stable_baselines3.common.vec_env import DummyVecEnv, VecNormalize

    run_name = f"right_swing_step{int(step_target*100)}cm_200k"
    policy_zip = TRAIN_DIR / run_name / "ppo_policy.zip"
    if not policy_zip.exists():
        raise FileNotFoundError(f"Train right_swing first: {policy_zip}")

    warm_data_wsl, base_z_wsl = get_wsl_end_state()
    env = WarmStartEnv(
        warm_data=warm_data_wsl, warm_base_z=base_z_wsl,
        jitter_qpos_std=0.0, jitter_qvel_std=0.0,
        model_path=MODEL_PATH, pose_path=POSE_PATH,
        task="right_clearance",
        action_scale=0.15, upright_penalty_weight=20.0,
        action_penalty_weight=0.05, action_delta_penalty_weight=0.02,
        left_contact_penalty_weight=0.10,
        clearance_reward_weight=4.0, clearance_target=0.004,
        step_target_length=step_target, step_reward_weight=1.0,
        termination_roll_limit=0.55,
    )
    obs, _ = env.reset()

    dummy = DummyVecEnv([lambda: UrdfFEnv()])
    vecnorm = VecNormalize.load(str(TRAIN_DIR / f"{run_name}/vecnormalize.pkl"), dummy)
    vecnorm.training = False; vecnorm.norm_reward = False
    model = PPO.load(str(policy_zip), device="cpu")

    max_rclr = 0.0
    for step in range(25):
        obs_norm = vecnorm.normalize_obs(obs[None])[0]
        action, _ = model.predict(obs_norm, deterministic=True)
        obs, _, terminated, _, info = env.step(action)
        max_rclr = max(max_rclr, info.get("right_clearance", 0.0))
        if terminated:
            print(f"  [WARN] right_swing terminated at step {step}")
            break

    base_z = float(env.data.qpos[2])
    warm_data = mujoco.MjData(env.model)
    mujoco.mj_copyData(warm_data, env.model, env.data)
    print(f"  right_swing end: base_z={base_z:.4f}, maxRclr={max_rclr*1000:.2f}mm")
    print(f"  right_foot_y={env._right_foot_y():.4f}, left_foot_y={env._left_foot_y():.4f}")
    return warm_data, base_z


def capture_lclr_end_state(step_target: float):
    """Run sequence through left_clearance_step5cm to get warm_data for left_return."""
    from stable_baselines3 import PPO
    from stable_baselines3.common.vec_env import DummyVecEnv, VecNormalize

    run_name = f"left_clearance_step{int(step_target*100)}cm_200k"
    policy_zip = TRAIN_DIR / run_name / "ppo_policy.zip"
    vecnorm_pkl = TRAIN_DIR / run_name / "vecnormalize.pkl"

    if not policy_zip.exists():
        raise FileNotFoundError(f"Train left_clearance first: {policy_zip}")

    warm_data_wsr, base_z_wsr = get_wsr_v9_warm_data_v2()

    env = WarmStartEnv(
        warm_data=warm_data_wsr,
        warm_base_z=base_z_wsr,
        jitter_qpos_std=0.0,
        jitter_qvel_std=0.0,
        model_path=MODEL_PATH,
        pose_path=POSE_PATH,
        task="left_clearance",
        action_scale=0.15,
        upright_penalty_weight=20.0,
        action_penalty_weight=0.05,
        action_delta_penalty_weight=0.02,
        right_contact_penalty_weight=0.10,
        clearance_reward_weight=4.0,
        clearance_target=0.004,
        step_target_length=step_target,
        step_reward_weight=1.0,
        termination_roll_limit=0.55,
    )
    obs, _ = env.reset()

    dummy = DummyVecEnv([lambda: UrdfFEnv()])
    vecnorm = VecNormalize.load(str(vecnorm_pkl), dummy)
    vecnorm.training = False
    vecnorm.norm_reward = False
    model = PPO.load(str(policy_zip), device="cpu")

    max_lclr = 0.0
    for step in range(25):
        obs_norm = vecnorm.normalize_obs(obs[None])[0]
        action, _ = model.predict(obs_norm, deterministic=True)
        obs, _, terminated, _, info = env.step(action)
        max_lclr = max(max_lclr, info.get("left_clearance", 0.0))
        if terminated:
            print(f"  [WARN] left_clearance terminated at step {step}")
            break

    base_z = float(env.data.qpos[2])
    warm_data = mujoco.MjData(env.model)
    mujoco.mj_copyData(warm_data, env.model, env.data)
    print(f"  left_clearance end: base_z={base_z:.4f}, maxLclr={max_lclr*1000:.2f}mm")
    print(f"  left_foot_y={env._left_foot_y():.4f}, right_foot_y={env._right_foot_y():.4f}")
    return warm_data, base_z


def capture_cycle2_lclr_end_state():
    """Run cycle1 + cycle2 through lclr to capture warm state for lret_fwd."""
    from stable_baselines3 import PPO
    from stable_baselines3.common.vec_env import DummyVecEnv, VecNormalize

    print("Running cycle1 + cycle2-through-lclr to capture lret_fwd warm state...")

    def load_pol(d):
        dummy = DummyVecEnv([lambda: UrdfFEnv()])
        vn = VecNormalize.load(str(TRAIN_DIR / d / "vecnormalize.pkl"), dummy)
        vn.training = False; vn.norm_reward = False
        return PPO.load(str(TRAIN_DIR / d / "ppo_policy.zip"), device="cpu"), vn

    env = UrdfFEnv(model_path=MODEL_PATH, pose_path=POSE_PATH, task="weight_shift_left")
    env.reset()

    NO = dict(step_target_length=0.0)
    S5 = dict(step_target_length=0.05, step_reward_weight=1.0)

    # Cycle 1 stages
    c1_stages = [
        ("weight_shift_left_rcp_150k",              100, "weight_shift_left",
         dict(action_scale=0.15, upright_penalty_weight=24.0, action_penalty_weight=0.05,
              action_delta_penalty_weight=0.02, right_contact_penalty_weight=0.30, **NO)),
        ("right_swing_step5cm_200k",                 25, "right_clearance",
         dict(action_scale=0.15, upright_penalty_weight=20.0, action_penalty_weight=0.05,
              action_delta_penalty_weight=0.02, left_contact_penalty_weight=0.10,
              clearance_reward_weight=4.0, clearance_target=0.004, **S5)),
        ("right_return_from_wsl_rcp_nomstd_150k",   170, "right_return",
         dict(action_scale=0.12, upright_penalty_weight=36.0, action_penalty_weight=0.08,
              action_delta_penalty_weight=0.03, fall_penalty=20.0, **NO)),
        ("weight_shift_right_lcp_v9_corrected_200k", 100, "weight_shift_right",
         dict(action_scale=0.15, upright_penalty_weight=24.0, action_penalty_weight=0.05,
              action_delta_penalty_weight=0.02, left_contact_penalty_weight=0.30, **NO)),
        ("left_clearance_step5cm_200k",              25, "left_clearance",
         dict(action_scale=0.15, upright_penalty_weight=20.0, action_penalty_weight=0.05,
              action_delta_penalty_weight=0.02, right_contact_penalty_weight=0.10,
              clearance_reward_weight=4.0, clearance_target=0.004, **S5)),
        ("left_return_from_lclr_v2_200k",           200, "left_return",
         dict(action_scale=0.12, upright_penalty_weight=36.0, action_penalty_weight=0.08,
              action_delta_penalty_weight=0.03, fall_penalty=20.0, **NO)),
    ]
    # Cycle 2 stages through lclr
    c2_stages = [
        ("weight_shift_left_fwd_200k",               100, "weight_shift_left",
         dict(action_scale=0.15, upright_penalty_weight=24.0, action_penalty_weight=0.05,
              action_delta_penalty_weight=0.02, right_contact_penalty_weight=0.30, **NO)),
        ("right_swing_fwd_step5cm_200k",              25, "right_clearance",
         dict(action_scale=0.15, upright_penalty_weight=20.0, action_penalty_weight=0.05,
              action_delta_penalty_weight=0.02, left_contact_penalty_weight=0.10,
              clearance_reward_weight=4.0, clearance_target=0.004, **S5)),
        ("right_return_from_wsl_rcp_nomstd_150k",    170, "right_return",
         dict(action_scale=0.12, upright_penalty_weight=36.0, action_penalty_weight=0.08,
              action_delta_penalty_weight=0.03, fall_penalty=20.0, **NO)),
        ("weight_shift_right_lcp_v9_corrected_200k", 100, "weight_shift_right",
         dict(action_scale=0.15, upright_penalty_weight=24.0, action_penalty_weight=0.05,
              action_delta_penalty_weight=0.02, left_contact_penalty_weight=0.30, **NO)),
        ("left_clearance_step5cm_200k",               25, "left_clearance",
         dict(action_scale=0.15, upright_penalty_weight=20.0, action_penalty_weight=0.05,
              action_delta_penalty_weight=0.02, right_contact_penalty_weight=0.10,
              clearance_reward_weight=4.0, clearance_target=0.004, **S5)),
    ]

    for stages, label in [(c1_stages, "cycle1"), (c2_stages, "cycle2")]:
        for pol_dir, steps, task, kwargs in stages:
            model, vecnorm = load_pol(pol_dir)
            for k, v in kwargs.items():
                setattr(env, k, v)
            env.task = task; env.prev_action[:] = 0.0
            env.base_z = float(env.data.qpos[2])
            for step in range(steps):
                obs_n = vecnorm.normalize_obs(env._obs()[None])[0]
                action, _ = model.predict(obs_n, deterministic=True)
                _, _, terminated, _, info = env.step(action)
                if terminated:
                    print(f"  [WARN] {task}({label}) terminated at step {step}")
                    break
            env.prev_action[:] = 0.0
            lclr = info.get("left_clearance", 0.0)
            rclr = info.get("right_clearance", 0.0)
            print(f"  {label}/{task:25s}  Rclr={rclr*1e3:.1f}mm Lclr={lclr*1e3:.1f}mm")

    base_z = float(env.data.qpos[2])
    env.base_z = base_z
    warm_data = mujoco.MjData(env.model)
    mujoco.mj_copyData(warm_data, env.model, env.data)
    print(f"\n  lret_fwd warm state: base_z={base_z:.4f}")
    print(f"  left_Y={env._left_foot_y():.4f}  right_Y={env._right_foot_y():.4f}")
    return warm_data, base_z


def _capture_policy_end_state(parent_warm_fn, pol_name, task, steps, **env_kwargs):
    """Generic: get warm_data from parent → run pol → return end state."""
    from stable_baselines3 import PPO
    from stable_baselines3.common.vec_env import DummyVecEnv, VecNormalize

    warm_data_parent, base_z_parent = parent_warm_fn()
    pol_path = TRAIN_DIR / pol_name / "ppo_policy.zip"
    if not pol_path.exists():
        raise FileNotFoundError(f"Train {pol_name} first: {pol_path}")

    env = WarmStartEnv(
        warm_data=warm_data_parent, warm_base_z=base_z_parent,
        jitter_qpos_std=0.0, jitter_qvel_std=0.0,
        model_path=MODEL_PATH, pose_path=POSE_PATH,
        task=task, **env_kwargs,
    )
    obs, _ = env.reset()

    dummy = DummyVecEnv([lambda: UrdfFEnv()])
    vecnorm = VecNormalize.load(str(TRAIN_DIR / pol_name / "vecnormalize.pkl"), dummy)
    vecnorm.training = False; vecnorm.norm_reward = False
    model = PPO.load(str(pol_path), device="cpu")

    max_rclr = max_lclr = 0.0
    for step in range(steps):
        obs_n = vecnorm.normalize_obs(obs[None])[0]
        action, _ = model.predict(obs_n, deterministic=True)
        obs, _, term, _, info = env.step(action)
        max_rclr = max(max_rclr, info.get("right_clearance", 0.0))
        max_lclr = max(max_lclr, info.get("left_clearance", 0.0))
        if term:
            print(f"  [WARN] {pol_name} terminated at step {step}")
            break
    env.prev_action[:] = 0.0
    base_z = float(env.data.qpos[2])
    warm_data = mujoco.MjData(env.model)
    mujoco.mj_copyData(warm_data, env.model, env.data)
    print(f"  {pol_name} end: Rclr={max_rclr*1e3:.1f}mm Lclr={max_lclr*1e3:.1f}mm")
    return warm_data, base_z


def train_with_warmstart(warm_data, base_z, task, run_name, total_timesteps,
                         n_envs=8, **env_kwargs):
    """Train a policy using WarmStartEnv."""
    from stable_baselines3 import PPO
    from stable_baselines3.common.monitor import Monitor
    from stable_baselines3.common.vec_env import DummyVecEnv, VecNormalize

    run_dir = TRAIN_DIR / run_name
    run_dir.mkdir(parents=True, exist_ok=True)
    print(f"\nTraining {run_name} for {total_timesteps} steps...")

    def make_env(rank: int):
        def _factory():
            env = WarmStartEnv(
                warm_data=warm_data,
                warm_base_z=base_z,
                jitter_qpos_std=0.002,
                jitter_qvel_std=0.05,
                model_path=MODEL_PATH,
                pose_path=POSE_PATH,
                task=task,
                **env_kwargs,
            )
            return Monitor(env)
        return _factory

    vec_env = DummyVecEnv([make_env(i) for i in range(n_envs)])
    vec_env = VecNormalize(vec_env, norm_obs=True, norm_reward=True, clip_obs=10.0)

    model = PPO(
        "MlpPolicy",
        vec_env,
        learning_rate=3e-4,
        n_steps=512,
        batch_size=64,
        n_epochs=10,
        gamma=0.99,
        gae_lambda=0.95,
        clip_range=0.2,
        ent_coef=0.005,
        verbose=1,
        device="auto",
    )

    model.learn(total_timesteps=total_timesteps)
    model.save(str(run_dir / "ppo_policy"))
    vec_env.save(str(run_dir / "vecnormalize.pkl"))
    print(f"Saved to {run_dir}")


def train_without_warmstart(task, run_name, total_timesteps, n_envs=8, **env_kwargs):
    """Train from symmetric standing (no warm start)."""
    from stable_baselines3 import PPO
    from stable_baselines3.common.monitor import Monitor
    from stable_baselines3.common.vec_env import DummyVecEnv, VecNormalize

    run_dir = TRAIN_DIR / run_name
    run_dir.mkdir(parents=True, exist_ok=True)
    print(f"\nTraining {run_name} for {total_timesteps} steps...")

    def make_env(rank: int):
        def _factory():
            env = UrdfFEnv(
                model_path=MODEL_PATH,
                pose_path=POSE_PATH,
                task=task,
                **env_kwargs,
            )
            return Monitor(env)
        return _factory

    vec_env = DummyVecEnv([make_env(i) for i in range(n_envs)])
    vec_env = VecNormalize(vec_env, norm_obs=True, norm_reward=True, clip_obs=10.0)

    model = PPO(
        "MlpPolicy",
        vec_env,
        learning_rate=3e-4,
        n_steps=512,
        batch_size=64,
        n_epochs=10,
        gamma=0.99,
        gae_lambda=0.95,
        clip_range=0.2,
        ent_coef=0.005,
        verbose=1,
        device="auto",
    )

    model.learn(total_timesteps=total_timesteps)
    model.save(str(run_dir / "ppo_policy"))
    vec_env.save(str(run_dir / "vecnormalize.pkl"))
    print(f"Saved to {run_dir}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--stage", required=True,
                        choices=["left_clearance", "left_return", "lret_fwd",
                                 "weight_shift_left", "wsl_fwd",
                                 "right_swing", "right_swing_fwd", "right_return", "right_return_fwd"])
    parser.add_argument("--total-timesteps", type=int, default=200_000)
    parser.add_argument("--n-envs", type=int, default=8)
    parser.add_argument("--step-target", type=float, default=0.05,
                        help="Step placement target length in meters (default 0.05 = 5cm)")
    parser.add_argument("--step-reward-weight", type=float, default=1.0)
    args = parser.parse_args()

    step_cm = int(args.step_target * 100)

    if args.stage == "left_clearance":
        warm_data, base_z = get_wsr_v9_warm_data_v2()
        train_with_warmstart(
            warm_data, base_z,
            task="left_clearance",
            run_name=f"left_clearance_step{step_cm}cm_200k",
            total_timesteps=args.total_timesteps,
            n_envs=args.n_envs,
            max_episode_steps=50,
            action_scale=0.15,
            upright_penalty_weight=20.0,
            action_penalty_weight=0.05,
            action_delta_penalty_weight=0.02,
            right_contact_penalty_weight=0.10,
            clearance_reward_weight=4.0,
            clearance_target=0.004,
            step_target_length=args.step_target,
            step_reward_weight=args.step_reward_weight,
            termination_roll_limit=0.55,
            termination_pitch_limit=0.55,
        )

    elif args.stage == "left_return":
        warm_data, base_z = capture_lclr_end_state(args.step_target)
        train_with_warmstart(
            warm_data, base_z,
            task="left_return",
            run_name=f"left_return_step{step_cm}cm_200k",
            total_timesteps=args.total_timesteps,
            n_envs=args.n_envs,
            max_episode_steps=250,
            action_scale=0.12,
            upright_penalty_weight=36.0,
            action_penalty_weight=0.08,
            action_delta_penalty_weight=0.03,
            fall_penalty=20.0,
            step_target_length=args.step_target,
            step_reward_weight=args.step_reward_weight,
            termination_roll_limit=0.55,
            termination_pitch_limit=0.55,
        )

    elif args.stage == "weight_shift_left":
        # Train from symmetric standing (no warm start needed for first cycle)
        train_without_warmstart(
            task="weight_shift_left",
            run_name=f"weight_shift_left_step{step_cm}cm_150k",
            total_timesteps=min(args.total_timesteps, 150_000),
            n_envs=args.n_envs,
            max_episode_steps=150,
            action_scale=0.15,
            upright_penalty_weight=24.0,
            action_penalty_weight=0.05,
            action_delta_penalty_weight=0.02,
            right_contact_penalty_weight=0.30,
            step_target_length=args.step_target,
            step_reward_weight=args.step_reward_weight,
            termination_roll_limit=0.55,
            termination_pitch_limit=0.55,
        )

    elif args.stage == "lret_fwd":
        warm_data, base_z = capture_cycle2_lclr_end_state()
        train_with_warmstart(
            warm_data, base_z,
            task="left_return",
            run_name="left_return_fwd_200k",
            total_timesteps=args.total_timesteps,
            n_envs=args.n_envs,
            max_episode_steps=250,
            action_scale=0.12,
            upright_penalty_weight=36.0,
            action_penalty_weight=0.08,
            action_delta_penalty_weight=0.03,
            fall_penalty=20.0,
            step_target_length=0.0,
            termination_roll_limit=0.55,
            termination_pitch_limit=0.55,
        )

    elif args.stage == "wsl_fwd":
        # wsl trained from cycle-2 starting state (after left_return of cycle 1)
        warm_data, base_z = capture_cycle2_wsl_state()
        train_with_warmstart(
            warm_data, base_z,
            task="weight_shift_left",
            run_name="weight_shift_left_fwd_200k",
            total_timesteps=args.total_timesteps,
            n_envs=args.n_envs,
            max_episode_steps=150,
            action_scale=0.15,
            upright_penalty_weight=24.0,
            action_penalty_weight=0.05,
            action_delta_penalty_weight=0.02,
            right_contact_penalty_weight=0.30,
            step_target_length=0.0,
            termination_roll_limit=0.55,
            termination_pitch_limit=0.55,
        )

    elif args.stage == "right_swing_fwd":
        # right_swing trained from wsl_fwd end state
        warm_data, base_z = get_wsl_fwd_end_state()
        train_with_warmstart(
            warm_data, base_z,
            task="right_clearance",
            run_name=f"right_swing_fwd_step{step_cm}cm_200k",
            total_timesteps=args.total_timesteps,
            n_envs=args.n_envs,
            max_episode_steps=50,
            action_scale=0.15,
            upright_penalty_weight=20.0,
            action_penalty_weight=0.05,
            action_delta_penalty_weight=0.02,
            left_contact_penalty_weight=0.10,
            clearance_reward_weight=4.0,
            clearance_target=0.004,
            step_target_length=args.step_target,
            step_reward_weight=args.step_reward_weight,
            termination_roll_limit=0.55,
            termination_pitch_limit=0.55,
        )

    elif args.stage == "right_return_fwd":
        # right_return trained from right_swing_fwd end state
        run_name_swing = f"right_swing_fwd_step{step_cm}cm_200k"
        warm_data, base_z = _capture_policy_end_state(
            parent_warm_fn=get_wsl_fwd_end_state,
            pol_name=run_name_swing,
            task="right_clearance",
            steps=25,
            action_scale=0.15, upright_penalty_weight=20.0,
            action_penalty_weight=0.05, action_delta_penalty_weight=0.02,
            left_contact_penalty_weight=0.10,
            clearance_reward_weight=4.0, clearance_target=0.004,
            step_target_length=args.step_target, step_reward_weight=args.step_reward_weight,
        )
        train_with_warmstart(
            warm_data, base_z,
            task="right_return",
            run_name=f"right_return_fwd_step{step_cm}cm_200k",
            total_timesteps=args.total_timesteps,
            n_envs=args.n_envs,
            max_episode_steps=220,
            action_scale=0.12,
            upright_penalty_weight=36.0,
            action_penalty_weight=0.08,
            action_delta_penalty_weight=0.03,
            fall_penalty=20.0,
            step_target_length=args.step_target,
            step_reward_weight=args.step_reward_weight,
            termination_roll_limit=0.55,
            termination_pitch_limit=0.55,
        )

    elif args.stage == "right_swing":
        warm_data, base_z = get_wsl_end_state()
        train_with_warmstart(
            warm_data, base_z,
            task="right_clearance",
            run_name=f"right_swing_step{step_cm}cm_200k",
            total_timesteps=args.total_timesteps,
            n_envs=args.n_envs,
            max_episode_steps=50,
            action_scale=0.15,
            upright_penalty_weight=20.0,
            action_penalty_weight=0.05,
            action_delta_penalty_weight=0.02,
            left_contact_penalty_weight=0.10,
            clearance_reward_weight=4.0,
            clearance_target=0.004,
            step_target_length=args.step_target,
            step_reward_weight=args.step_reward_weight,
            termination_roll_limit=0.55,
            termination_pitch_limit=0.55,
        )

    elif args.stage == "right_return":
        warm_data, base_z = capture_rswing_end_state(args.step_target)
        train_with_warmstart(
            warm_data, base_z,
            task="right_return",
            run_name=f"right_return_step{step_cm}cm_200k",
            total_timesteps=min(args.total_timesteps, 200_000),
            n_envs=args.n_envs,
            max_episode_steps=220,
            action_scale=0.12,
            upright_penalty_weight=36.0,
            action_penalty_weight=0.08,
            action_delta_penalty_weight=0.03,
            fall_penalty=20.0,
            step_target_length=args.step_target,
            step_reward_weight=args.step_reward_weight,
            termination_roll_limit=0.55,
            termination_pitch_limit=0.55,
        )


if __name__ == "__main__":
    main()
