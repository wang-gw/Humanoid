"""Plot per-joint torque (and speed) profiles from a trained E2E walking policy.

Usage:
    python3 scripts/plot_torque_profile.py --run e2e_walk_v5 --out outputs/torque_profile_v5.png
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv, VecNormalize

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from envs.gait_walking_env import GaitWalkingEnv

_ap = argparse.ArgumentParser()
_ap.add_argument("--run", default="e2e_walk_v5")
_ap.add_argument("--out", default=None)
_ap.add_argument("--max-steps", type=int, default=2000)
args = _ap.parse_args()

RUN_DIR = PROJECT_ROOT / "outputs/train/urdf_f" / args.run
OUT_PATH = args.out or str(PROJECT_ROOT / f"outputs/torque_profile_{args.run}.png")

MODEL_PATH = str(PROJECT_ROOT / "envs/robots/urdf_f_link/URDF_F_link_virtual_stl_footprint_contact.xml")
POSE_PATH  = str(PROJECT_ROOT / "configs/symmetric_standing_pose.json")

# ── Load policy ──────────────────────────────────────────────────────────────
policy_path = RUN_DIR / "ppo_policy.zip"
vecnorm_path = RUN_DIR / "vecnormalize.pkl"

# VecNormalize for obs normalization only
dummy = DummyVecEnv([lambda: GaitWalkingEnv()])
vn = VecNormalize.load(str(vecnorm_path), dummy)
vn.training = False
vn.norm_reward = False

model = PPO.load(str(policy_path), device="cpu")

# Raw env for stepping (same params as render script)
env = GaitWalkingEnv(
    model_path=MODEL_PATH, pose_path=POSE_PATH, task="walking",
    max_episode_steps=args.max_steps + 100, frame_skip=10,
    action_scale=0.15, upright_penalty_weight=20.0,
    action_penalty_weight=0.05, action_delta_penalty_weight=0.02,
    fall_penalty=20.0, gait_period_steps=400,
)

# ── Rollout ───────────────────────────────────────────────────────────────────
obs, _ = env.reset()
inner_env = env
actuator_names = inner_env.actuator_names

torques = []   # shape: (T, nu)
speeds  = []   # shape: (T, nu)
rewards = []

for step in range(args.max_steps):
    obs_n = vn.normalize_obs(obs[None])[0]
    action, _ = model.predict(obs_n, deterministic=True)
    obs, reward, term, trunc, info = env.step(action)

    torques.append(inner_env.data.actuator_force.copy())
    speeds.append(inner_env.data.qvel[inner_env.joint_dofadr].copy())
    rewards.append(float(reward))

    if term or trunc:
        print(f"Episode ended at step {step + 1} ({'fell' if term else 'truncated'})")
        break

torques = np.array(torques)   # (T, nu)
speeds  = np.array(speeds)    # (T, nu)
T = torques.shape[0]
steps_ax = np.arange(T)
time_ax  = steps_ax * 0.05   # control dt = 0.05 s

print(f"Total steps: {T}  ({T * 0.05:.1f} s)")
print(f"\n{'Joint':<25} {'Max |torque| [Nm]':>18} {'Max |speed| [rad/s]':>20}")
print("-" * 65)
for i, name in enumerate(actuator_names):
    print(f"{name:<25} {np.max(np.abs(torques[:, i])):>18.2f} {np.max(np.abs(speeds[:, i])):>20.3f}")

# ── Plot ──────────────────────────────────────────────────────────────────────
nu = len(actuator_names)
# Group joints: hip_pitch, hip_roll, knee_pitch, ankle_pitch, ankle_roll  (L then R)
# We'll do 5 rows (one per joint type), 2 columns (L / R)
joint_pairs = []
seen = set()
for name in actuator_names:
    base = name.replace("_L", "").replace("_R", "").replace("_left", "").replace("_right", "")
    if base not in seen:
        seen.add(base)
        joint_pairs.append(base)

# Build a 2-column layout: left joints | right joints
left_idx  = [i for i, n in enumerate(actuator_names) if "_L" in n or "_left" in n]
right_idx = [i for i, n in enumerate(actuator_names) if "_R" in n or "_right" in n]
# Fallback: first half / second half
if not left_idx:
    left_idx  = list(range(nu // 2))
    right_idx = list(range(nu // 2, nu))

n_rows = max(len(left_idx), len(right_idx))
fig = plt.figure(figsize=(16, 3.5 * n_rows))
fig.suptitle(f"Per-joint Torque Profile — {args.run}  ({T} steps / {T*0.05:.1f} s)",
             fontsize=14, fontweight="bold")

colors = {"L": "#2196F3", "R": "#F44336"}

gs = gridspec.GridSpec(n_rows, 2, figure=fig, hspace=0.55, wspace=0.35)

def _plot_joint(ax, idx, side_label):
    name = actuator_names[idx]
    t = torques[:, idx]
    peak = np.max(np.abs(t))
    ax.plot(time_ax, t, color=colors[side_label], linewidth=0.9)
    ax.axhline(0, color="gray", linewidth=0.5, linestyle="--")
    ax.fill_between(time_ax, t, 0, alpha=0.15, color=colors[side_label])
    ax.set_title(f"{name}  [peak {peak:.1f} Nm]", fontsize=9)
    ax.set_ylabel("Torque [Nm]", fontsize=8)
    ax.set_xlabel("Time [s]", fontsize=8)
    ax.tick_params(labelsize=7)
    ax.set_xlim(0, time_ax[-1])

for row, (li, ri) in enumerate(zip(left_idx, right_idx)):
    ax_l = fig.add_subplot(gs[row, 0])
    ax_r = fig.add_subplot(gs[row, 1])
    _plot_joint(ax_l, li, "L")
    _plot_joint(ax_r, ri, "R")

plt.savefig(OUT_PATH, dpi=150, bbox_inches="tight")
print(f"\nSaved → {OUT_PATH}")
