from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from envs.urdf_f_env import DEFAULT_MODEL, DEFAULT_POSE, UrdfFEnv


def main() -> int:
    parser = argparse.ArgumentParser(description="Smoke-test the URDF_F Gymnasium RL environment.")
    parser.add_argument("--model", type=Path, default=DEFAULT_MODEL)
    parser.add_argument("--pose-json", type=Path, default=DEFAULT_POSE)
    parser.add_argument(
        "--task",
        choices=["standing", "weight_shift_left", "right_unload", "right_clearance"],
        default="standing",
    )
    parser.add_argument("--steps", type=int, default=200)
    parser.add_argument("--policy", choices=["zero", "random"], default="zero")
    parser.add_argument("--seed", type=int, default=1)
    args = parser.parse_args()

    env = UrdfFEnv(model_path=args.model, pose_path=args.pose_json, task=args.task)
    obs, info = env.reset(seed=args.seed)
    rng = np.random.default_rng(args.seed)
    total_reward = 0.0
    last_info = info

    for _ in range(args.steps):
        if args.policy == "random":
            action = rng.uniform(-0.15, 0.15, size=env.action_space.shape).astype(np.float32)
        else:
            action = np.zeros(env.action_space.shape, dtype=np.float32)
        obs, reward, terminated, truncated, last_info = env.step(action)
        total_reward += float(reward)
        if terminated or truncated:
            break

    summary = {
        "model": str(Path(args.model).resolve()),
        "pose_json": str(Path(args.pose_json).resolve()),
        "task": args.task,
        "policy": args.policy,
        "steps_requested": args.steps,
        "steps_completed": int(last_info["step_count"]),
        "obs_shape": list(obs.shape),
        "action_shape": list(env.action_space.shape),
        "total_reward": total_reward,
        "terminated_reason": last_info.get("terminated_reason", ""),
        "final": {
            "sim_time": last_info.get("sim_time"),
            "base_z": last_info.get("base_z"),
            "roll": last_info.get("roll"),
            "pitch": last_info.get("pitch"),
            "left_force_ratio": last_info.get("left_force_ratio"),
            "right_force_ratio": last_info.get("right_force_ratio"),
            "left_contacts": last_info.get("left_contacts"),
            "right_contacts": last_info.get("right_contacts"),
            "right_clearance": last_info.get("right_clearance"),
            "max_abs_tau_cmd": last_info.get("max_abs_tau_cmd"),
            "reward_total": last_info.get("reward_total"),
        },
    }
    env.close()
    print(json.dumps(summary, indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
