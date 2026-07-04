from __future__ import annotations

import argparse
import csv
import json
import os
import sys
from dataclasses import dataclass, field
from pathlib import Path

import imageio.v2 as imageio
import numpy as np

os.environ.setdefault("MUJOCO_GL", "egl")

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from envs.urdf_f_env import DEFAULT_MODEL, UrdfFEnv


@dataclass(frozen=True)
class Stage:
    name: str
    task: str
    steps: int
    policy_path: Path | None
    vecnormalize_path: Path | None
    action_scale: float
    upright_penalty_weight: float
    action_penalty_weight: float
    action_delta_penalty_weight: float
    left_contact_penalty_weight: float = 0.0
    right_contact_penalty_weight: float = 0.0
    clearance_reward_weight: float = 2.0
    clearance_target: float = 0.002
    gated_clearance_reward: bool = False
    clearance_gate_roll: float = 0.12
    clearance_gate_pitch: float = 0.12
    fall_penalty: float = 0.0
    stability_excess_penalty_weight: float = 0.0
    termination_roll_limit: float = 0.55
    termination_pitch_limit: float = 0.55


def overlay_text(frame: np.ndarray, rows: list[str]) -> np.ndarray:
    try:
        import cv2
    except ImportError:
        return frame
    out = frame.copy()
    panel_h = 22 + 22 * len(rows)
    cv2.rectangle(out, (12, 12), (560, panel_h), (20, 20, 20), thickness=-1)
    cv2.rectangle(out, (12, 12), (560, panel_h), (220, 220, 220), thickness=1)
    y = 38
    for row in rows:
        cv2.putText(out, row, (24, y), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (245, 245, 245), 1, cv2.LINE_AA)
        y += 22
    return out


def load_policy(policy_path: Path, vecnormalize_path: Path):
    from stable_baselines3 import PPO
    from stable_baselines3.common.vec_env import DummyVecEnv, VecNormalize

    dummy = DummyVecEnv([lambda: UrdfFEnv()])
    vecnorm = VecNormalize.load(str(vecnormalize_path), dummy)
    vecnorm.training = False
    vecnorm.norm_reward = False
    model = PPO.load(str(policy_path), device="cpu")
    return model, vecnorm


def apply_stage_params(env: UrdfFEnv, stage: Stage) -> None:
    env.task = stage.task
    env.action_scale = stage.action_scale
    env.upright_penalty_weight = stage.upright_penalty_weight
    env.action_penalty_weight = stage.action_penalty_weight
    env.action_delta_penalty_weight = stage.action_delta_penalty_weight
    env.left_contact_penalty_weight = stage.left_contact_penalty_weight
    env.right_contact_penalty_weight = stage.right_contact_penalty_weight
    env.clearance_reward_weight = stage.clearance_reward_weight
    env.clearance_target = stage.clearance_target
    env.gated_clearance_reward = stage.gated_clearance_reward
    env.clearance_gate_roll = stage.clearance_gate_roll
    env.clearance_gate_pitch = stage.clearance_gate_pitch
    env.fall_penalty = stage.fall_penalty
    env.stability_excess_penalty_weight = stage.stability_excess_penalty_weight
    env.termination_roll_limit = stage.termination_roll_limit
    env.termination_pitch_limit = stage.termination_pitch_limit


def default_stages(clearance_steps: int = 100) -> list[Stage]:
    train = PROJECT_ROOT / "outputs/train/urdf_f"
    return [
        Stage(
            name="weight_shift_right",
            task="weight_shift_right",
            steps=100,
            policy_path=train / "weight_shift_right_sym_100k/ppo_policy.zip",
            vecnormalize_path=train / "weight_shift_right_sym_100k/vecnormalize.pkl",
            action_scale=0.15,
            upright_penalty_weight=24.0,
            action_penalty_weight=0.05,
            action_delta_penalty_weight=0.02,
            termination_roll_limit=0.55,
            termination_pitch_limit=0.55,
        ),
        Stage(
            name="left_clearance",
            task="left_clearance",
            steps=clearance_steps,
            policy_path=train / "left_clearance_from_wsr_150k/ppo_policy.zip",
            vecnormalize_path=train / "left_clearance_from_wsr_150k/vecnormalize.pkl",
            action_scale=0.12,
            upright_penalty_weight=24.0,
            action_penalty_weight=0.08,
            action_delta_penalty_weight=0.03,
            left_contact_penalty_weight=0.30,
            clearance_reward_weight=6.0,
            clearance_target=0.0002,
            gated_clearance_reward=True,
            clearance_gate_roll=0.10,
            clearance_gate_pitch=0.10,
            fall_penalty=30.0,
            termination_roll_limit=0.55,
            termination_pitch_limit=0.55,
        ),
        Stage(
            name="left_return",
            task="left_return",
            steps=170,
            policy_path=train / "left_return_sym_30k/ppo_policy.zip",
            vecnormalize_path=train / "left_return_sym_30k/vecnormalize.pkl",
            action_scale=0.12,
            upright_penalty_weight=24.0,
            action_penalty_weight=0.08,
            action_delta_penalty_weight=0.03,
            fall_penalty=20.0,
            termination_roll_limit=0.55,
            termination_pitch_limit=0.55,
        ),
        Stage(
            name="settle",
            task="standing",
            steps=100,
            policy_path=None,
            vecnormalize_path=None,
            action_scale=0.05,
            upright_penalty_weight=80.0,
            action_penalty_weight=0.12,
            action_delta_penalty_weight=0.06,
            termination_roll_limit=0.55,
            termination_pitch_limit=0.55,
        ),
    ]


def main() -> int:
    parser = argparse.ArgumentParser(description="Evaluate left step sequence.")
    parser.add_argument("--model", type=Path,
                        default=PROJECT_ROOT / "envs/robots/urdf_f_link/URDF_F_link_virtual_stl_footprint_contact.xml")
    parser.add_argument("--pose-json", type=Path,
                        default=PROJECT_ROOT / "configs/dynamic_standing_pose_actuator_dynamics_contact_5s.json")
    parser.add_argument("--out-dir", type=Path, default=Path("outputs/eval/urdf_f_sequence_left_step"))
    parser.add_argument("--seed", type=int, default=1)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--clearance-steps", type=int, default=100)
    args = parser.parse_args()

    out_dir = args.out_dir.resolve()
    out_dir.mkdir(parents=True, exist_ok=True)

    stages = default_stages(clearance_steps=args.clearance_steps)

    loaded: dict[str, tuple[object, object]] = {}
    for stage in stages:
        if stage.policy_path is None:
            continue
        if not stage.policy_path.exists() or not stage.vecnormalize_path.exists():
            raise FileNotFoundError(f"Missing policy for stage {stage.name}: {stage.policy_path}")
        loaded[stage.name] = load_policy(stage.policy_path.resolve(), stage.vecnormalize_path.resolve())

    env = UrdfFEnv(
        model_path=args.model,
        pose_path=args.pose_json,
        task=stages[0].task,
        render_mode="rgb_array",
        width=args.width,
        height=args.height,
    )
    obs, _ = env.reset(seed=args.seed)

    frames: list[np.ndarray] = []
    rows: list[dict] = []
    total_reward = 0.0
    terminated_reason = ""
    global_step = 0

    for stage in stages:
        apply_stage_params(env, stage)
        model_vec = loaded.get(stage.name)
        for local_step in range(stage.steps):
            if model_vec is None:
                action = np.zeros(env.action_space.shape, dtype=np.float32)
            else:
                policy, vecnorm = model_vec
                norm_obs = vecnorm.normalize_obs(obs.reshape(1, -1))
                action, _ = policy.predict(norm_obs, deterministic=True)
                action = np.asarray(action).reshape(env.action_space.shape).astype(np.float32)

            obs, reward, terminated, truncated, info = env.step(action)
            total_reward += float(reward)
            global_step += 1
            row = {
                "global_step": float(global_step),
                "stage": stage.name,
                "stage_step": float(local_step + 1),
                "sim_time": float(info["sim_time"]),
                "reward": float(reward),
                "roll": float(info["roll"]),
                "pitch": float(info["pitch"]),
                "base_z": float(info["base_z"]),
                "left_force_ratio": float(info["left_force_ratio"]),
                "right_force_ratio": float(info["right_force_ratio"]),
                "left_contacts": float(info["left_contacts"]),
                "right_contacts": float(info["right_contacts"]),
                "left_clearance": float(info["left_clearance"]),
                "right_clearance": float(info["right_clearance"]),
                "max_abs_tau_cmd": float(info["max_abs_tau_cmd"]),
                "terminated_reason": str(info["terminated_reason"]),
            }
            rows.append(row)

            frame = env.render()
            if frame is not None:
                frames.append(
                    overlay_text(
                        frame,
                        [
                            f"LEFT sequence  {stage.name}  step={global_step}",
                            f"roll={row['roll']:+.3f} pitch={row['pitch']:+.3f} z={row['base_z']:+.3f}",
                            f"Lcontact={row['left_contacts']:.0f} Lclear={row['left_clearance']:.4f} Lforce={row['left_force_ratio']:.2f}",
                            f"tau={row['max_abs_tau_cmd']:.2f} reward={row['reward']:.2f}",
                        ],
                    )
                )

            if terminated or truncated:
                terminated_reason = str(info["terminated_reason"])
                break
        if terminated_reason:
            break

    env.close()
    for _, vecnorm in loaded.values():
        vecnorm.close()

    mp4_path = out_dir / "sequence_evaluation.mp4"
    gif_path = out_dir / "sequence_evaluation.gif"
    if frames:
        imageio.mimsave(mp4_path, frames, fps=args.fps)
        gif_stride = max(1, int(args.fps / 12))
        imageio.mimsave(gif_path, frames[::gif_stride], duration=gif_stride / args.fps)
        imageio.imwrite(out_dir / "last_frame.png", frames[-1])

    csv_path = out_dir / "timeline.csv"
    if rows:
        with csv_path.open("w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=list(rows[0].keys()))
            writer.writeheader()
            writer.writerows(rows)

    summary = {
        "model": str(args.model.resolve()),
        "pose_json": str(args.pose_json.resolve()),
        "stages": [
            {
                "name": stage.name,
                "task": stage.task,
                "steps": stage.steps,
                "policy_path": str(stage.policy_path.resolve()) if stage.policy_path else "zero",
            }
            for stage in stages
        ],
        "steps_completed": len(rows),
        "total_reward": total_reward,
        "terminated_reason": terminated_reason,
        "final": rows[-1] if rows else {},
        "mp4": str(mp4_path),
        "gif": str(gif_path),
        "timeline_csv": str(csv_path),
    }
    (out_dir / "summary.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(json.dumps(summary, indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
