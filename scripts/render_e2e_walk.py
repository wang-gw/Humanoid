#!/usr/bin/env python3
"""Render e2e walking policy — side-tracking camera + distance overlay."""

from __future__ import annotations
import os, sys
os.environ["MUJOCO_GL"] = "egl"
sys.path.insert(0, str(__import__("pathlib").Path(__file__).resolve().parents[1]))

import numpy as np
import mujoco
import imageio
from PIL import Image, ImageDraw, ImageFont
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv, VecNormalize
from envs.gait_walking_env import GaitWalkingEnv

import argparse
_ap = argparse.ArgumentParser()
_ap.add_argument("--run", default="e2e_walk_v1")
_ap.add_argument("--out", default=None)
_args, _ = _ap.parse_known_args()

MODEL_PATH = "envs/robots/urdf_f_link/URDF_F_link_virtual_stl_footprint_contact.xml"
POSE_PATH  = "configs/symmetric_standing_pose.json"
RUN_DIR    = f"outputs/train/urdf_f/{_args.run}"
OUT_PATH   = _args.out or f"outputs/{_args.run}_tracking.mp4"
N_STEPS    = 2000
FPS        = 30
RENDER_EVERY = 3
W, H = 640, 480

def make_env():
    return GaitWalkingEnv(
        model_path=MODEL_PATH, pose_path=POSE_PATH, task="walking",
        max_episode_steps=N_STEPS + 100, frame_skip=10,
        action_scale=0.15, upright_penalty_weight=20.0,
        action_penalty_weight=0.05, action_delta_penalty_weight=0.02,
        fall_penalty=20.0, gait_period_steps=400,
    )

dummy = DummyVecEnv([make_env])
vn = VecNormalize.load(f"{RUN_DIR}/vecnormalize.pkl", dummy)
vn.training = False; vn.norm_reward = False
model = PPO.load(f"{RUN_DIR}/ppo_policy.zip", device="cpu")

env = make_env()
obs, _ = env.reset()
init_y = env.data.qpos[1]

renderer = mujoco.Renderer(env.model, height=H, width=W)

cam = mujoco.MjvCamera()
cam.type      = mujoco.mjtCamera.mjCAMERA_FREE
cam.azimuth   = 90    # 측면 (X축 방향에서 바라봄)
cam.elevation = -12
cam.distance  = 1.4   # 가깝게

opt = mujoco.MjvOption()

try:
    font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", 22)
except:
    font = ImageFont.load_default()

frames = []
for step in range(N_STEPS):
    obs_n = vn.normalize_obs(obs[None])[0]
    act, _ = model.predict(obs_n, deterministic=True)
    obs, _, term, trunc, info = env.step(act)

    if step % RENDER_EVERY == 0:
        robot_y = float(env.data.qpos[1])
        robot_z = float(env.data.qpos[2])

        # 카메라가 로봇 Y 위치를 따라감 (측면 추적)
        cam.lookat[0] = 0.0
        cam.lookat[1] = robot_y
        cam.lookat[2] = robot_z + 0.05

        renderer.update_scene(env.data, camera=cam, scene_option=opt)
        frame = renderer.render()

        # PIL로 텍스트 오버레이
        img = Image.fromarray(frame)
        draw = ImageDraw.Draw(img)
        net = (robot_y - init_y) * 100
        t = step / 10.0  # 실제 시간 (frame_skip=10, dt=0.005 → 0.05s/step)
        draw.rectangle([(8, 8), (320, 70)], fill=(0, 0, 0, 180))
        draw.text((14, 12), f"Forward: {net:+.1f} cm", font=font, fill=(0, 255, 100))
        draw.text((14, 40), f"Time: {t:.1f} s", font=font, fill=(200, 200, 200))

        frames.append(np.array(img))

    if term or trunc:
        print(f"Step {step}: {'fell' if term else 'done'}")
        break

print(f"Saving {len(frames)} frames → {OUT_PATH}")
os.makedirs("outputs", exist_ok=True)
imageio.mimsave(OUT_PATH, frames, fps=FPS, quality=8)
net_final = (env.data.qpos[1] - init_y) * 100
print(f"Done.  Total forward: {net_final:+.1f} cm")
