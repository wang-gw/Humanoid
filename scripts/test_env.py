from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path
from typing import Any

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from envs import UnitreeEnv


def print_space(name: str, space: Any) -> None:
    print(f"[{name}] type={type(space).__name__}")
    if hasattr(space, "shape"):
        print(f"[{name}] shape={space.shape}")
    if hasattr(space, "dtype"):
        print(f"[{name}] dtype={space.dtype}")
    if hasattr(space, "low") and hasattr(space, "high"):
        low = np.asarray(space.low)
        high = np.asarray(space.high)
        print(f"[{name}] low(min/max)=({low.min():.3f}, {low.max():.3f})")
        print(f"[{name}] high(min/max)=({high.min():.3f}, {high.max():.3f})")


def build_home_action(env: UnitreeEnv) -> np.ndarray:
    nu = int(env.model.nu)
    if nu == 0:
        return np.zeros(0, dtype=np.float32)

    home_joint = np.asarray(env.home_joint_pos, dtype=np.float32)
    if home_joint.shape[0] < nu:
        action = 0.5 * (env.action_space.low + env.action_space.high)
    else:
        action = home_joint[:nu].copy()
    return np.clip(action, env.action_space.low, env.action_space.high).astype(np.float32)


def get_actuator_names(env: UnitreeEnv) -> list[str]:
    names: list[str] = []
    for i in range(int(env.model.nu)):
        n = None
        try:
            n = env._mujoco.mj_id2name(env.model, env._mujoco.mjtObj.mjOBJ_ACTUATOR, i)
        except Exception:
            n = None
        names.append(n if n else f"act_{i}")
    return names


def build_pulse_action(
    t: int,
    home_action: np.ndarray,
    low: np.ndarray,
    high: np.ndarray,
    hold_steps: int,
    ratio: float,
) -> tuple[np.ndarray, int, float]:
    nu = home_action.shape[0]
    action = home_action.copy()
    if nu == 0:
        return action, -1, 0.0

    segment = t // max(hold_steps, 1)
    if segment >= 2 * nu:
        return action, -1, 0.0

    idx = segment // 2
    sign = 1.0 if (segment % 2 == 0) else -1.0

    amp = ratio * float(high[idx] - low[idx])
    target = float(home_action[idx] + sign * amp)
    action[idx] = np.clip(target, float(low[idx]), float(high[idx]))
    return action, int(idx), float(sign)


def main() -> int:
    parser = argparse.ArgumentParser(description="Env test: home hold + actuator pulse test")
    parser.add_argument("--render-mode", default="human", choices=["human", "rgb_array"])
    parser.add_argument("--steps", type=int, default=600)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--robot", default="go1", choices=["go1", "g1"])
    parser.add_argument("--frame-skip", type=int, default=5)
    parser.add_argument("--sleep-sec", type=float, default=0.03)
    parser.add_argument("--policy", default="home", choices=["home", "sine", "random", "pulse"])
    parser.add_argument("--stage", default="stand", choices=["stand", "walk"])
    parser.add_argument("--reward", default="standing")
    parser.add_argument(
        "--base-controller",
        default="home",
        choices=["home", "trot", "humanoid_walk", "reference_motion", "footstep_ik"],
    )
    parser.add_argument("--zero-command", action="store_true")
    parser.add_argument("--print-every", type=int, default=1)
    parser.add_argument("--pulse-ratio", type=float, default=0.15)
    parser.add_argument("--pulse-hold-steps", type=int, default=60)
    args = parser.parse_args()

    env = UnitreeEnv(
        render_mode=args.render_mode,
        robot=args.robot,
        frame_skip=args.frame_skip,
        sleep_sec=args.sleep_sec,
        stage=args.stage,
        reward_name=args.reward,
        base_controller=args.base_controller,
    )

    print("=== Env Created ===")
    print(f"env_class={env.__class__.__name__}")
    print(f"render_mode={env.render_mode}")
    print(f"max_episode_steps={env.max_episode_steps}")
    print(f"frame_skip={env.frame_skip}")
    print(f"sleep_sec={env.sleep_sec}")
    print(f"stage={env.stage}")
    print(f"reward_module={env._reward_module.__class__.__name__}")
    print(f"metadata={env.metadata}")
    print(
        f"model(nq={env.model.nq}, nv={env.model.nv}, nu={env.model.nu}, "
        f"nbody={env.model.nbody}, ngeom={env.model.ngeom}, dt={env.model.opt.timestep})"
    )
    print_space("action_space", env.action_space)
    print_space("observation_space", env.observation_space)

    reset_options: dict[str, Any] | None = None
    if args.zero_command:
        reset_options = {"stage": args.stage, "command": np.zeros(3, dtype=np.float32)}

    try:
        obs, info = env.reset(seed=args.seed, options=reset_options)
    except RuntimeError as exc:
        print(f"[ERROR] render init failed: {exc}")
        env.close()
        return 1

    print("\n=== Reset ===")
    print(f"obs.shape={obs.shape}, obs.dtype={obs.dtype}")
    print(f"obs(min/max)=({obs.min():.3f}, {obs.max():.3f})")
    print(f"command={env.command}")

    center = 0.5 * (env.action_space.low + env.action_space.high)
    amp = 0.2 * (env.action_space.high - env.action_space.low)
    amp = np.where(np.isfinite(amp), amp, 0.0).astype(np.float32)
    home_action = build_home_action(env)
    actuator_names = get_actuator_names(env)

    print("\n=== Rollout ===")
    prev_seg = -1
    for t in range(args.steps):
        active_idx = -1
        active_sign = 0.0

        if args.policy == "random":
            action = env.action_space.sample()
        elif args.policy == "sine":
            phase = 0.2 * t
            action = center + amp * np.sin(
                phase + np.linspace(0.0, math.pi, env.action_space.shape[0], dtype=np.float32)
            )
        elif args.policy == "pulse":
            action, active_idx, active_sign = build_pulse_action(
                t=t,
                home_action=home_action,
                low=env.action_space.low,
                high=env.action_space.high,
                hold_steps=args.pulse_hold_steps,
                ratio=args.pulse_ratio,
            )
            seg = t // max(args.pulse_hold_steps, 1)
            if seg != prev_seg:
                prev_seg = seg
                if active_idx >= 0:
                    s = "+" if active_sign > 0 else "-"
                    print(
                        f"[PULSE] step={t} actuator={active_idx}:{actuator_names[active_idx]} sign={s} "
                        f"target={action[active_idx]:.4f} home={home_action[active_idx]:.4f}"
                    )
                else:
                    print(f"[PULSE] step={t} done. back to home action")
        else:
            action = home_action

        try:
            next_obs, reward, terminated, truncated, step_info = env.step(action)
        except RuntimeError as exc:
            print(f"[ERROR] render step failed: {exc}")
            env.close()
            return 1

        joint_pos = np.asarray(env.data.qpos[7:], dtype=np.float32)
        n = min(joint_pos.shape[0], env.home_joint_pos.shape[0])
        joint_dev = joint_pos[:n] - env.home_joint_pos[:n]

        force_val = 0.0
        if active_idx >= 0 and getattr(env.data, "actuator_force", None) is not None:
            force_val = float(env.data.actuator_force[active_idx])

        if t % max(args.print_every, 1) == 0:
            print(
                f"step={t:04d} reward={reward:.4f} term={terminated} trunc={truncated} "
                f"act_norm={np.linalg.norm(action):.4f} jdev_l2={np.linalg.norm(joint_dev):.4f} "
                f"height={float(step_info.get('trunk_height', 0.0)):.4f} "
                f"g_z={float(step_info.get('projected_gravity_z', 0.0)):.4f} "
                f"active={active_idx} force={force_val:.4f}"
            )

        if args.render_mode == "rgb_array":
            frame = env.render()
            if frame is not None:
                print(f"render(rgb_array) shape={frame.shape}, dtype={frame.dtype}")

        if terminated or truncated:
            print(f"Episode ended: reason={step_info.get('terminated_reason', '')}. resetting...")
            obs, info = env.reset(options=reset_options)
            if args.policy in {"home", "pulse"}:
                home_action = build_home_action(env)
        else:
            obs = next_obs

    env.close()
    print("\nDone.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

"""
# home 유지 확인
mjpython test.py --policy home --zero-command --steps 1000 --sleep-sec 0.03

# 액추에이터 하나씩 +/- 자극 자동 점검
mjpython test.py --policy pulse --zero-command --steps 1600 --pulse-hold-steps 60 --pulse-ratio 0.15 --sleep-sec 0.01
"""
