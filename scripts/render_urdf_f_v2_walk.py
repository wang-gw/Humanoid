#!/usr/bin/env python3
"""Render a urdf_f_v2 policy — side-tracking camera + distance overlay.

Mirrors render_e2e_walk.py but uses GaitWalkingV2Env (pins the v2 model/pose/foot),
and accepts an explicit checkpoint so intermediate policies can be rendered.

Usage:
  python3 scripts/render_urdf_f_v2_walk.py \
      --policy outputs/train/urdf_f_v2/e2e_walk_v2model_v1/checkpoints/ppo_1200000_steps.zip \
      --vecnorm outputs/train/urdf_f_v2/e2e_walk_v2model_v1/checkpoints/vecnorm_1200000.pkl \
      --out outputs/e2e_walk_v2model_standing.mp4
"""
from __future__ import annotations
import os, sys, argparse
os.environ["MUJOCO_GL"] = "egl"
sys.path.insert(0, str(__import__("pathlib").Path(__file__).resolve().parents[1]))

import numpy as np
import mujoco
import imageio
from PIL import Image, ImageDraw, ImageFont
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv, VecNormalize
from envs.urdf_f_v2_env import GaitWalkingV2Env

ap = argparse.ArgumentParser()
ap.add_argument("--policy", required=True)
ap.add_argument("--vecnorm", required=True)
ap.add_argument("--out", default="outputs/urdf_f_v2/urdf_f_v2_tracking.mp4")
ap.add_argument("--steps", type=int, default=1000)
args = ap.parse_args()

N_STEPS, FPS, RENDER_EVERY, W, H = args.steps, 30, 3, 640, 480

def make_env():
    return GaitWalkingV2Env(task="walking", max_episode_steps=N_STEPS + 100,
                            frame_skip=10, action_scale=0.15, gait_period_steps=400)

vn = VecNormalize.load(args.vecnorm, DummyVecEnv([make_env]))
vn.training = False; vn.norm_reward = False
model = PPO.load(args.policy, device="cpu")

env = make_env()
obs, _ = env.reset()
init_y = env.data.qpos[1]

renderer = mujoco.Renderer(env.model, height=H, width=W)
cam = mujoco.MjvCamera()
cam.type = mujoco.mjtCamera.mjCAMERA_FREE
cam.azimuth, cam.elevation, cam.distance = 90, -12, 1.4
opt = mujoco.MjvOption()
try:
    font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", 22)
except Exception:
    font = ImageFont.load_default()

frames = []
for step in range(N_STEPS):
    obs_n = vn.normalize_obs(obs[None])[0]
    act, _ = model.predict(obs_n, deterministic=True)
    obs, _, term, trunc, info = env.step(act)
    if step % RENDER_EVERY == 0:
        robot_y = float(env.data.qpos[1]); robot_z = float(env.data.qpos[2])
        cam.lookat[0] = 0.0; cam.lookat[1] = robot_y; cam.lookat[2] = robot_z + 0.05
        renderer.update_scene(env.data, camera=cam, scene_option=opt)
        frame = renderer.render()
        img = Image.fromarray(frame); draw = ImageDraw.Draw(img)
        net = (robot_y - init_y) * 100; t = step / 20.0
        draw.rectangle([(8, 8), (330, 70)], fill=(0, 0, 0))
        draw.text((14, 12), f"Forward: {net:+.1f} cm", font=font, fill=(0, 255, 100))
        draw.text((14, 40), f"Time: {t:.1f} s", font=font, fill=(200, 200, 200))
        frames.append(np.array(img))
    if term or trunc:
        print(f"Step {step}: {'fell' if term else 'done'}")
        break

os.makedirs("outputs", exist_ok=True)
imageio.mimsave(args.out, frames, fps=FPS, quality=8)
net_final = (env.data.qpos[1] - init_y) * 100
print(f"Saved {len(frames)} frames -> {args.out}   (forward {net_final:+.1f} cm, {len(frames)*RENDER_EVERY*0.05:.1f}s)")
