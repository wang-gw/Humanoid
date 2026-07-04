from __future__ import annotations

import argparse
import json
import os
import sys
from pathlib import Path

os.environ.setdefault("MUJOCO_GL", "egl")

import imageio.v2 as imageio
import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from envs.urdf_f_env import DEFAULT_MODEL, DEFAULT_POSE, UrdfFEnv


def overlay_text(frame: np.ndarray, rows: list[str]) -> np.ndarray:
    try:
        import cv2
    except ImportError:
        return frame
    out = frame.copy()
    panel_h = 22 + 22 * len(rows)
    cv2.rectangle(out, (12, 12), (470, panel_h), (20, 20, 20), thickness=-1)
    cv2.rectangle(out, (12, 12), (470, panel_h), (220, 220, 220), thickness=1)
    y = 38
    for row in rows:
        cv2.putText(out, row, (24, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (245, 245, 245), 1, cv2.LINE_AA)
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


def main() -> int:
    parser = argparse.ArgumentParser(description="Evaluate and render a URDF_F standing/RL policy.")
    parser.add_argument("--model", type=Path, default=DEFAULT_MODEL)
    parser.add_argument("--pose-json", type=Path, default=DEFAULT_POSE)
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
    parser.add_argument("--policy", choices=["zero", "random", "ppo"], default="zero")
    parser.add_argument("--policy-path", type=Path, default=Path(""))
    parser.add_argument("--vecnormalize-path", type=Path, default=Path(""))
    parser.add_argument("--steps", type=int, default=250)
    parser.add_argument("--seed", type=int, default=1)
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
    parser.add_argument("--out-dir", type=Path, default=Path("outputs/eval/urdf_f"))
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    args = parser.parse_args()

    run_name = f"{args.task}_{args.policy}_seed{args.seed}"
    out_dir = (args.out_dir / run_name).resolve()
    out_dir.mkdir(parents=True, exist_ok=True)

    env = UrdfFEnv(
        model_path=args.model,
        pose_path=args.pose_json,
        task=args.task,
        render_mode="rgb_array",
        width=args.width,
        height=args.height,
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
    obs, info = env.reset(seed=args.seed)
    rng = np.random.default_rng(args.seed)

    ppo_model = None
    vecnorm = None
    if args.policy == "ppo":
        if not args.policy_path.exists():
            raise FileNotFoundError(f"Policy not found: {args.policy_path}")
        if not args.vecnormalize_path.exists():
            raise FileNotFoundError(f"VecNormalize state not found: {args.vecnormalize_path}")
        ppo_model, vecnorm = load_policy(args.policy_path.resolve(), args.vecnormalize_path.resolve())

    frames: list[np.ndarray] = []
    rows: list[dict[str, float | str]] = []
    total_reward = 0.0
    final_info = info

    for step in range(args.steps):
        if args.policy == "zero":
            action = np.zeros(env.action_space.shape, dtype=np.float32)
        elif args.policy == "random":
            action = rng.uniform(-0.15, 0.15, size=env.action_space.shape).astype(np.float32)
        else:
            assert ppo_model is not None and vecnorm is not None
            norm_obs = vecnorm.normalize_obs(obs.reshape(1, -1))
            action, _ = ppo_model.predict(norm_obs, deterministic=True)
            action = np.asarray(action).reshape(env.action_space.shape).astype(np.float32)

        obs, reward, terminated, truncated, final_info = env.step(action)
        total_reward += float(reward)
        row = {
            "step": float(step + 1),
            "sim_time": float(final_info["sim_time"]),
            "reward": float(reward),
            "roll": float(final_info["roll"]),
            "pitch": float(final_info["pitch"]),
            "base_z": float(final_info["base_z"]),
            "left_force_ratio": float(final_info["left_force_ratio"]),
            "right_force_ratio": float(final_info["right_force_ratio"]),
            "left_contacts": float(final_info["left_contacts"]),
            "right_contacts": float(final_info["right_contacts"]),
            "left_clearance": float(final_info["left_clearance"]),
            "right_clearance": float(final_info["right_clearance"]),
            "max_abs_tau_cmd": float(final_info["max_abs_tau_cmd"]),
            "terminated_reason": str(final_info["terminated_reason"]),
        }
        rows.append(row)

        frame = env.render()
        if frame is not None:
            frames.append(
                overlay_text(
                    frame,
                    [
                        f"{args.task} / {args.policy}  t={row['sim_time']:.2f}s",
                        f"roll={row['roll']:+.3f}  pitch={row['pitch']:+.3f}  z={row['base_z']:+.3f}",
                        f"L={row['left_force_ratio']:.2f} R={row['right_force_ratio']:.2f} Lcl={row['left_clearance']:.4f} Rcl={row['right_clearance']:.4f}",
                        f"tau={row['max_abs_tau_cmd']:.2f}Nm  reward={row['reward']:.2f}",
                    ],
                )
            )

        if terminated or truncated:
            break

    env.close()
    if vecnorm is not None:
        vecnorm.close()

    mp4_path = out_dir / "evaluation.mp4"
    gif_path = out_dir / "evaluation.gif"
    if frames:
        imageio.mimsave(mp4_path, frames, fps=args.fps)
        gif_stride = max(1, int(args.fps / 12))
        imageio.mimsave(gif_path, frames[::gif_stride], duration=gif_stride / args.fps)
        imageio.imwrite(out_dir / "last_frame.png", frames[-1])

    import csv

    csv_path = out_dir / "timeline.csv"
    if rows:
        with csv_path.open("w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=list(rows[0].keys()))
            writer.writeheader()
            writer.writerows(rows)

    summary = {
        "model": str(args.model.resolve()),
        "pose_json": str(args.pose_json.resolve()),
        "task": args.task,
        "policy": args.policy,
        "policy_path": str(args.policy_path.resolve()) if args.policy_path != Path("") else "",
        "vecnormalize_path": str(args.vecnormalize_path.resolve()) if args.vecnormalize_path != Path("") else "",
        "steps_requested": args.steps,
        "steps_completed": len(rows),
        "total_reward": total_reward,
        "terminated_reason": final_info.get("terminated_reason", ""),
        "final": rows[-1] if rows else {},
        "mp4": str(mp4_path) if frames else "",
        "gif": str(gif_path) if frames else "",
        "timeline_csv": str(csv_path),
    }
    (out_dir / "summary.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(json.dumps(summary, indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
