from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from envs import UnitreeEnv
from rewards import ReferenceWalkingReward
from scripts.train import build_policy_kwargs


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="DAgger-style BC for full-action reference tracking.")
    parser.add_argument("--motion-path", required=True)
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--init-model", default="")
    parser.add_argument("--robot", default="g1", choices=["g1", "go1"])
    parser.add_argument("--reference-obs-scope", default="all", choices=["all", "lower_body", "legs_only"])
    parser.add_argument("--ref-joint-scope", default="lower_body", choices=["all", "lower_body", "legs_only"])
    parser.add_argument("--reference-time-scale", type=float, default=0.8)
    parser.add_argument("--reference-action-blend", type=float, default=0.9)
    parser.add_argument("--reference-random-start", action="store_true")
    parser.add_argument("--reference-start-time-range", type=float, nargs=2, default=None, metavar=("LOW", "HIGH"))
    parser.add_argument("--reference-start-time", type=float, default=0.0)
    parser.add_argument("--foot-state-obs", action="store_true")
    parser.add_argument("--command", type=float, nargs=3, default=(0.24, 0.0, 0.0))
    parser.add_argument("--reference-samples", type=int, default=12000)
    parser.add_argument("--skip-bootstrap-train", action="store_true")
    parser.add_argument("--dagger-iters", type=int, default=5)
    parser.add_argument("--rollout-samples", type=int, default=6000)
    parser.add_argument("--epochs-per-iter", type=int, default=25)
    parser.add_argument("--batch-size", type=int, default=512)
    parser.add_argument("--learning-rate", type=float, default=2e-4)
    parser.add_argument("--action-noise", type=float, default=0.01)
    parser.add_argument(
        "--action-noise-final",
        type=float,
        default=None,
        help="Linearly decay DAgger rollout noise to this value. Default keeps --action-noise constant.",
    )
    parser.add_argument("--eval-steps", type=int, default=1000)
    parser.add_argument("--max-episode-steps", type=int, default=1000)
    parser.add_argument("--balance-corrected-labels", action="store_true")
    parser.add_argument("--balance-correction-max", type=float, default=0.06)
    parser.add_argument("--balance-width-target", type=float, default=0.24)
    parser.add_argument("--balance-width-gain", type=float, default=0.35)
    parser.add_argument("--balance-side-target", type=float, default=0.11)
    parser.add_argument("--balance-side-gain", type=float, default=0.0)
    parser.add_argument("--balance-capture-gain", type=float, default=0.18)
    parser.add_argument("--balance-roll-gain", type=float, default=0.04)
    parser.add_argument("--balance-yaw-gain", type=float, default=0.06)
    parser.add_argument("--footstep-ik-labels", action="store_true")
    parser.add_argument("--footstep-ik-gain", type=float, default=0.45)
    parser.add_argument("--footstep-ik-damping", type=float, default=0.04)
    parser.add_argument("--footstep-ik-max-dq", type=float, default=0.08)
    parser.add_argument("--footstep-target-rel-x", type=float, default=0.10)
    parser.add_argument("--footstep-target-rel-x-min", type=float, default=0.06)
    parser.add_argument("--footstep-target-rel-x-max", type=float, default=0.18)
    parser.add_argument("--footstep-capture-x-gain", type=float, default=0.0)
    parser.add_argument("--footstep-capture-x-clip", type=float, default=0.08)
    parser.add_argument("--footstep-pitch-gain", type=float, default=0.0)
    parser.add_argument("--footstep-err-x-back-clip", type=float, default=0.12)
    parser.add_argument("--footstep-err-x-forward-clip", type=float, default=0.16)
    parser.add_argument("--footstep-target-side-y", type=float, default=0.12)
    parser.add_argument("--footstep-capture-y-clip", type=float, default=0.08)
    parser.add_argument("--footstep-lateral-drift-gain", type=float, default=0.0)
    parser.add_argument("--footstep-lateral-drift-clip", type=float, default=0.06)
    parser.add_argument("--footstep-target-z", type=float, default=0.045)
    parser.add_argument("--recovery-labels", action="store_true")
    parser.add_argument("--recovery-sample-filter", action="store_true")
    parser.add_argument("--recovery-min-step", type=int, default=700)
    parser.add_argument("--recovery-gravity-threshold", type=float, default=0.14)
    parser.add_argument("--recovery-yaw-threshold", type=float, default=0.25)
    parser.add_argument("--recovery-speed-threshold", type=float, default=0.36)
    parser.add_argument("--recovery-foot-behind-threshold", type=float, default=-0.20)
    parser.add_argument("--recovery-x-gain", type=float, default=0.0)
    parser.add_argument("--recovery-x-clip", type=float, default=0.10)
    parser.add_argument("--recovery-yaw-gain", type=float, default=0.0)
    parser.add_argument("--balance-phase-aware", action="store_true")
    parser.add_argument("--balance-swing-gain", type=float, default=0.10)
    parser.add_argument("--balance-stance-gain", type=float, default=0.65)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--policy-net-arch", type=int, nargs="*", default=[128, 128])
    parser.add_argument("--activation-fn", choices=["tanh", "relu", "elu"], default="elu")
    parser.add_argument("--log-std-init", type=float, default=-3.2)
    parser.add_argument("--device", default="cpu")
    return parser.parse_args()


def actor_mean(policy: object, obs_tensor: object) -> object:
    features = policy.extract_features(obs_tensor)
    if isinstance(features, tuple):
        features = features[0]
    latent_pi, _ = policy.mlp_extractor(features)
    return policy.action_net(latent_pi)


def make_env(args: argparse.Namespace) -> UnitreeEnv:
    reward = ReferenceWalkingReward(
        motion_path=args.motion_path,
        ref_joint_scope=args.ref_joint_scope,
        w_ref_joint=1.0,
    )
    return UnitreeEnv(
        render_mode=None,
        robot=args.robot,
        stage="walk",
        reward_name="reference_walking",
        reward_module=reward,
        residual_action=False,
        base_controller="reference_motion",
        command_low=tuple(args.command),
        command_high=tuple(args.command),
        reference_obs_path=args.motion_path,
        reference_obs=True,
        reference_obs_scope=args.reference_obs_scope,
        reference_error_obs=True,
        reference_init_pose_on_reset=True,
        reference_random_start=args.reference_random_start,
        reference_start_time_range=tuple(args.reference_start_time_range)
        if args.reference_start_time_range is not None
        else None,
        reference_start_time=args.reference_start_time,
        reference_time_scale=args.reference_time_scale,
        reference_action_blend=args.reference_action_blend,
        foot_state_obs=args.foot_state_obs,
        fall_height_ratio=0.55 if args.robot == "g1" else 0.85,
        max_episode_steps=int(args.max_episode_steps),
        max_forward_speed=10.0,
    )


def yaw_from_wxyz(quat: np.ndarray) -> float:
    w, x, y, z = [float(v) for v in quat[:4]]
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return float(np.arctan2(siny_cosp, cosy_cosp))


def wrap_angle(angle: float) -> float:
    return float((angle + np.pi) % (2.0 * np.pi) - np.pi)


def actuator_index(env: UnitreeEnv) -> dict[str, int]:
    return {str(env.model.actuator(i).name): i for i in range(int(env.model.nu))}


def foot_position(env: UnitreeEnv, side: str) -> np.ndarray | None:
    try:
        site_id = int(env.model.site(f"{side}_foot").id)
        return np.asarray(env.data.site_xpos[site_id], dtype=np.float32).copy()
    except Exception:
        return None


def balance_corrected_target(
    env: UnitreeEnv,
    reference_action: np.ndarray,
    args: argparse.Namespace,
    *,
    start_y: float,
    start_yaw: float,
    use_recovery_labels: bool = True,
) -> np.ndarray:
    if not args.balance_corrected_labels or env.robot != "g1":
        return reference_action.copy().astype(np.float32)

    corrected = reference_action.copy().astype(np.float32)
    idx = actuator_index(env)

    trunk = np.asarray(env.data.xpos[env._trunk_body_id], dtype=np.float32).copy()
    base_y = float(trunk[1])
    base_z = float(max(trunk[2], 0.35))
    base_vy = float(env.data.qvel[1])
    omega = float(np.sqrt(9.81 / base_z))
    capture_y = base_y + base_vy / max(omega, 1e-6)

    left = foot_position(env, "left")
    right = foot_position(env, "right")
    foot_mid_y = float(0.5 * (left[1] + right[1])) if left is not None and right is not None else float(start_y)
    foot_width = float(abs(left[1] - right[1])) if left is not None and right is not None else args.balance_width_target
    capture_error = float(capture_y - foot_mid_y)
    lateral_drift = float(base_y - start_y)
    projected_gravity = np.asarray(env._get_projected_gravity(), dtype=np.float32)
    roll_like = float(projected_gravity[1]) if projected_gravity.shape[0] > 1 else 0.0
    yaw = yaw_from_wxyz(np.asarray(env.data.qpos[3:7], dtype=np.float32))
    yaw_error = wrap_angle(yaw - start_yaw)

    max_corr = max(float(args.balance_correction_max), 0.0)
    width_shortfall = max(float(args.balance_width_target) - foot_width, 0.0)
    width_corr = float(np.clip(args.balance_width_gain * width_shortfall, 0.0, max_corr))
    side_corr_left = 0.0
    side_corr_right = 0.0
    if left is not None and right is not None and float(args.balance_side_gain) > 0.0:
        left_rel_y = float(left[1] - trunk[1])
        right_rel_y = float(right[1] - trunk[1])
        side_target = max(float(args.balance_side_target), 0.0)
        side_corr_left = float(
            np.clip(args.balance_side_gain * (side_target - left_rel_y), -max_corr, max_corr)
        )
        side_corr_right = float(
            np.clip(args.balance_side_gain * (-side_target - right_rel_y), -max_corr, max_corr)
        )
    capture_corr = float(
        np.clip(
            args.balance_capture_gain * capture_error
            + 0.25 * args.balance_capture_gain * lateral_drift
            + args.balance_roll_gain * roll_like,
            -max_corr,
            max_corr,
        )
    )
    yaw_gain = float(args.balance_yaw_gain)
    if use_recovery_labels and args.recovery_labels:
        yaw_gain += float(args.recovery_yaw_gain)
    yaw_corr = float(np.clip(yaw_gain * yaw_error, -max_corr, max_corr))

    contacts = np.asarray(env._get_foot_contacts(), dtype=np.float32)[:2]
    ref_contacts = np.asarray(env.get_reference_contact(), dtype=np.float32)[:2]
    contact_score = contacts if contacts.shape[0] >= 2 and float(np.sum(contacts)) > 0.5 else ref_contacts
    if contact_score.shape[0] < 2:
        contact_score = np.asarray([1.0, 1.0], dtype=np.float32)
    left_stance = float(contact_score[0]) >= float(contact_score[1])
    right_stance = float(contact_score[1]) >= float(contact_score[0])
    if left_stance and right_stance:
        phase_sin = float(np.sin(float(env.get_reference_time()) * 2.0 * np.pi))
        left_stance = phase_sin <= 0.0
        right_stance = not left_stance

    # MuJoCo G1 sign probe:
    # common positive hip-roll shifts the pelvis toward negative y; ankle roll
    # uses the opposite sign. Width correction is antisymmetric to preserve
    # gait phase while discouraging narrow/crossed stepping.
    if args.balance_phase_aware:
        stance_gain = float(args.balance_stance_gain)
        swing_gain = float(args.balance_swing_gain)
        left_hip = width_corr + side_corr_left
        right_hip = -width_corr + side_corr_right
        left_ankle = 0.0
        right_ankle = 0.0
        if left_stance:
            left_hip += stance_gain * capture_corr
            left_ankle += -0.7 * stance_gain * capture_corr
            right_hip += swing_gain * capture_corr
        if right_stance:
            right_hip += stance_gain * capture_corr
            right_ankle += -0.7 * stance_gain * capture_corr
            left_hip += swing_gain * capture_corr
        corrections = (
            ("left_hip_roll_joint", left_hip),
            ("right_hip_roll_joint", right_hip),
            ("left_ankle_roll_joint", left_ankle),
            ("right_ankle_roll_joint", right_ankle),
            ("waist_yaw_joint", -yaw_corr),
            ("left_hip_yaw_joint", -0.35 * yaw_corr),
            ("right_hip_yaw_joint", -0.35 * yaw_corr),
        )
    else:
        corrections = (
            ("left_hip_roll_joint", capture_corr + width_corr + side_corr_left),
            ("right_hip_roll_joint", capture_corr - width_corr + side_corr_right),
            ("left_ankle_roll_joint", -0.5 * capture_corr),
            ("right_ankle_roll_joint", -0.5 * capture_corr),
            ("waist_yaw_joint", -yaw_corr),
            ("left_hip_yaw_joint", -0.35 * yaw_corr),
            ("right_hip_yaw_joint", -0.35 * yaw_corr),
        )

    for name, value in corrections:
        act_i = idx.get(name)
        if act_i is not None:
            corrected[act_i] += float(np.clip(value, -max_corr, max_corr))

    if args.footstep_ik_labels:
        corrected = apply_footstep_ik_label(
            env,
            corrected,
            args,
            start_y=start_y,
            use_recovery_labels=use_recovery_labels,
        )

    return np.clip(corrected, env._ctrl_low, env._ctrl_high).astype(np.float32)


def is_recovery_state(env: UnitreeEnv, args: argparse.Namespace, *, start_yaw: float) -> bool:
    if not args.recovery_sample_filter:
        return True
    if int(getattr(env, "_step_count", 0)) < int(args.recovery_min_step):
        return False
    projected_gravity = np.asarray(env._get_projected_gravity(), dtype=np.float32)
    gravity_xy = float(np.linalg.norm(projected_gravity[:2])) if projected_gravity.shape[0] >= 2 else 0.0
    yaw = yaw_from_wxyz(np.asarray(env.data.qpos[3:7], dtype=np.float32))
    yaw_error = abs(wrap_angle(yaw - start_yaw))
    base_vx = float(env.data.qvel[0])
    cmd_vx = float(np.asarray(env.command, dtype=np.float32)[0]) if np.asarray(env.command).size else 0.0
    overspeed = base_vx > max(float(args.recovery_speed_threshold), cmd_vx + 0.12)
    foot_behind = False
    trunk = np.asarray(env.data.xpos[env._trunk_body_id], dtype=np.float32)
    for side in ("left", "right"):
        foot = foot_position(env, side)
        if foot is not None and float(foot[0] - trunk[0]) < float(args.recovery_foot_behind_threshold):
            foot_behind = True
            break
    return (
        gravity_xy > float(args.recovery_gravity_threshold)
        or yaw_error > float(args.recovery_yaw_threshold)
        or bool(overspeed)
        or foot_behind
    )


def choose_swing_side(env: UnitreeEnv) -> str:
    ref_contact = np.asarray(env.get_reference_contact(), dtype=np.float32)[:2]
    if ref_contact.shape[0] >= 2 and abs(float(ref_contact[0] - ref_contact[1])) > 0.1:
        return "left" if float(ref_contact[0]) < float(ref_contact[1]) else "right"
    contacts = np.asarray(env._get_foot_contacts(), dtype=np.float32)[:2]
    if contacts.shape[0] >= 2 and abs(float(contacts[0] - contacts[1])) > 0.1:
        return "left" if float(contacts[0]) < float(contacts[1]) else "right"
    left = foot_position(env, "left")
    right = foot_position(env, "right")
    if left is not None and right is not None:
        return "left" if float(left[2]) >= float(right[2]) else "right"
    return "left"


def apply_footstep_ik_label(
    env: UnitreeEnv,
    action: np.ndarray,
    args: argparse.Namespace,
    *,
    start_y: float,
    use_recovery_labels: bool = True,
) -> np.ndarray:
    swing = choose_swing_side(env)
    foot = foot_position(env, swing)
    if foot is None:
        return action

    try:
        trunk = np.asarray(env.data.xpos[env._trunk_body_id], dtype=np.float32).copy()
        base_z = float(max(trunk[2], 0.35))
        base_vx = float(env.data.qvel[0])
        base_vy = float(env.data.qvel[1])
        omega = float(np.sqrt(9.81 / base_z))
        capture_x = float(trunk[0] + base_vx / max(omega, 1e-6))
        x_capture_error = float(capture_x - (float(trunk[0]) + float(args.footstep_target_rel_x)))
        x_offset = float(args.footstep_target_rel_x) + float(args.footstep_capture_x_gain) * float(
            np.clip(x_capture_error, -args.footstep_capture_x_clip, args.footstep_capture_x_clip)
        )
        projected_gravity = np.asarray(env._get_projected_gravity(), dtype=np.float32)
        if use_recovery_labels and args.recovery_labels:
            cmd_vx = float(np.asarray(env.command, dtype=np.float32)[0]) if np.asarray(env.command).size else 0.0
            overspeed = max(float(base_vx) - max(cmd_vx, 0.0), 0.0)
            pitch_like = abs(float(projected_gravity[0])) if projected_gravity.shape[0] > 0 else 0.0
            recovery_drive = overspeed + 0.5 * pitch_like
            x_offset += float(args.recovery_x_gain) * float(
                np.clip(recovery_drive, 0.0, float(args.recovery_x_clip))
            )
        x_offset = float(np.clip(x_offset, args.footstep_target_rel_x_min, args.footstep_target_rel_x_max))
        capture_y = float(trunk[1] + base_vy / max(omega, 1e-6))
        capture_offset = float(np.clip(capture_y - float(trunk[1]), -args.footstep_capture_y_clip, args.footstep_capture_y_clip))
        lateral_drift = float(trunk[1] - start_y)
        lateral_offset = -float(args.footstep_lateral_drift_gain) * float(
            np.clip(lateral_drift, -args.footstep_lateral_drift_clip, args.footstep_lateral_drift_clip)
        )
        side_sign = 1.0 if swing == "left" else -1.0
        target_rel_y = side_sign * float(args.footstep_target_side_y) + capture_offset + lateral_offset
        min_side = 0.06
        if swing == "left":
            target_rel_y = max(target_rel_y, min_side)
        else:
            target_rel_y = min(target_rel_y, -min_side)
        target = np.asarray(
            [
                float(trunk[0]) + x_offset,
                float(trunk[1]) + target_rel_y,
                max(float(args.footstep_target_z), float(foot[2])),
            ],
            dtype=np.float32,
        )
        err = target - foot
        if projected_gravity.shape[0] > 0 and float(args.footstep_pitch_gain) > 0.0:
            err[0] += float(args.footstep_pitch_gain) * float(projected_gravity[0])
        err[0] = np.clip(err[0], -float(args.footstep_err_x_back_clip), float(args.footstep_err_x_forward_clip))
        err[1] = np.clip(err[1], -0.12, 0.12)
        err[2] = np.clip(err[2], -0.02, 0.05)
    except Exception:
        return action

    if float(np.linalg.norm(err[:2])) < 1e-3:
        return action

    site_id = int(env.model.site(f"{swing}_foot").id)
    jacp = np.zeros((3, int(env.model.nv)), dtype=np.float64)
    jacr = np.zeros((3, int(env.model.nv)), dtype=np.float64)
    env._mujoco.mj_jacSite(env.model, env.data, jacp, jacr, site_id)

    prefix = "left" if swing == "left" else "right"
    joint_names = [
        f"{prefix}_hip_pitch_joint",
        f"{prefix}_hip_roll_joint",
        f"{prefix}_hip_yaw_joint",
        f"{prefix}_knee_joint",
        f"{prefix}_ankle_pitch_joint",
        f"{prefix}_ankle_roll_joint",
    ]
    idx = actuator_index(env)
    act_ids = [idx[name] for name in joint_names if name in idx]
    if not act_ids:
        return action
    qvel_cols = [6 + act_i for act_i in act_ids if 6 + act_i < int(env.model.nv)]
    if len(qvel_cols) != len(act_ids):
        return action

    j = jacp[:, qvel_cols].astype(np.float64)
    weighted_err = np.asarray(err, dtype=np.float64) * float(args.footstep_ik_gain)
    damping = max(float(args.footstep_ik_damping), 1e-6)
    lhs = j @ j.T + (damping * damping) * np.eye(3)
    try:
        dq = j.T @ np.linalg.solve(lhs, weighted_err)
    except np.linalg.LinAlgError:
        return action
    dq = np.clip(dq, -float(args.footstep_ik_max_dq), float(args.footstep_ik_max_dq))

    out = action.copy().astype(np.float32)
    for act_i, delta in zip(act_ids, dq):
        out[act_i] += float(delta)
    return out


def train_supervised(model: object, obs: np.ndarray, targets: np.ndarray, args: argparse.Namespace, epochs: int) -> list[float]:
    import torch

    observations = torch.as_tensor(obs, dtype=torch.float32, device=model.device)
    target_tensor = torch.as_tensor(targets, dtype=torch.float32, device=model.device)
    optimizer = torch.optim.Adam(model.policy.parameters(), lr=args.learning_rate)
    n = int(observations.shape[0])
    losses: list[float] = []
    for _ in range(epochs):
        perm = torch.randperm(n, device=model.device)
        epoch_losses: list[float] = []
        for start in range(0, n, args.batch_size):
            idx = perm[start : start + args.batch_size]
            pred = actor_mean(model.policy, observations[idx])
            loss = torch.mean((pred - target_tensor[idx]) ** 2)
            optimizer.zero_grad(set_to_none=True)
            loss.backward()
            torch.nn.utils.clip_grad_norm_(model.policy.parameters(), 1.0)
            optimizer.step()
            epoch_losses.append(float(loss.detach().cpu()))
        losses.append(float(np.mean(epoch_losses)))
    return losses


def scheduled_noise(args: argparse.Namespace, iteration: int) -> float:
    if args.action_noise_final is None or args.dagger_iters <= 1:
        return float(args.action_noise)
    alpha = float(iteration) / float(max(args.dagger_iters - 1, 1))
    return float((1.0 - alpha) * args.action_noise + alpha * args.action_noise_final)


def evaluate_rollout(model: object, args: argparse.Namespace, seed: int) -> dict[str, Any]:
    env = make_env(args)
    obs, _ = env.reset(seed=seed)
    final_info: dict[str, Any] = {}
    xs: list[float] = []
    ys: list[float] = []
    yaws: list[float] = []
    vxs: list[float] = []
    total_reward = 0.0
    done = False
    for step in range(int(args.eval_steps)):
        action, _ = model.predict(obs, deterministic=True)
        obs, reward, terminated, truncated, info = env.step(action)
        final_info = dict(info)
        total_reward += float(reward)
        xs.append(float(info.get("base_pos_x", 0.0)))
        ys.append(float(info.get("base_pos_y", 0.0)))
        yaws.append(yaw_from_wxyz(np.asarray(env.data.qpos[3:7], dtype=np.float32)))
        vxs.append(float(info.get("base_lin_vel_x", 0.0)))
        if terminated or truncated:
            done = True
            break
    env.close()

    vx = np.asarray(vxs, dtype=np.float64)
    steps = len(xs)
    final_x = float(xs[-1]) if xs else 0.0
    negative_vx_fraction = float(np.mean(vx < 0.0)) if vx.size else 0.0
    mean_vx = float(np.mean(vx)) if vx.size else 0.0
    max_abs_yaw = float(np.max(np.abs(np.asarray(yaws, dtype=np.float64)))) if yaws else 0.0
    final_abs_y = abs(float(ys[-1])) if ys else 0.0
    # Step count alone can select policies that survive by stalling. Prefer
    # long rollouts that still move forward and do not spend much time moving
    # backward. This is only a checkpoint-selection score, not a reward.
    selection_score = float(
        steps
        + 80.0 * final_x
        + 120.0 * mean_vx
        - 250.0 * negative_vx_fraction
        - 35.0 * max_abs_yaw
        - 80.0 * final_abs_y
    )
    return {
        "steps": steps,
        "done": done,
        "terminated_reason": str(final_info.get("terminated_reason", "")),
        "final_x": final_x,
        "mean_vx": mean_vx,
        "negative_vx_fraction": negative_vx_fraction,
        "max_abs_yaw": max_abs_yaw,
        "final_abs_y": final_abs_y,
        "selection_score": selection_score,
        "total_reward": total_reward,
    }


def save_checkpoint(model: object, out: Path, name: str) -> str:
    path = out / f"{name}.zip"
    model.save(str(path))
    return str(path)


def main() -> int:
    args = parse_args()
    out = Path(args.output_dir)
    out.mkdir(parents=True, exist_ok=True)

    try:
        import torch
        from stable_baselines3 import PPO
    except ImportError as exc:
        raise SystemExit("[ERROR] Install torch and stable-baselines3 first.") from exc

    rng = np.random.default_rng(args.seed)
    torch.manual_seed(args.seed)
    np.random.seed(args.seed)

    env = make_env(args)
    if args.init_model:
        model = PPO.load(args.init_model, env=env, device=args.device)
    else:
        model = PPO(
            policy="MlpPolicy",
            env=env,
            learning_rate=args.learning_rate,
            n_steps=512,
            batch_size=min(args.batch_size, 512),
            n_epochs=1,
            gamma=0.99,
            policy_kwargs=build_policy_kwargs(args, torch),
            verbose=0,
            seed=args.seed,
            device=args.device,
        )

    obs_rows: list[np.ndarray] = []
    target_rows: list[np.ndarray] = []

    obs, _ = env.reset(seed=args.seed)
    start_y = float(env.data.xpos[env._trunk_body_id][1])
    start_yaw = yaw_from_wxyz(np.asarray(env.data.qpos[3:7], dtype=np.float32))
    for _ in range(args.reference_samples):
        reference_target = env.get_reference_action()
        target = balance_corrected_target(
            env,
            reference_target,
            args,
            start_y=start_y,
            start_yaw=start_yaw,
            use_recovery_labels=False,
        )
        obs_rows.append(np.asarray(obs, dtype=np.float32).copy())
        target_rows.append(np.asarray(target, dtype=np.float32).copy())
        obs, _, terminated, truncated, _ = env.step(reference_target)
        if terminated or truncated:
            obs, _ = env.reset()
            start_y = float(env.data.xpos[env._trunk_body_id][1])
            start_yaw = yaw_from_wxyz(np.asarray(env.data.qpos[3:7], dtype=np.float32))

    all_losses: list[float] = []
    iteration_summaries: list[dict[str, Any]] = []
    best_eval: dict[str, Any] | None = None
    best_model_path = out / "best_model.zip"

    if args.skip_bootstrap_train:
        bootstrap_loss = None
    else:
        all_losses += train_supervised(model, np.asarray(obs_rows), np.asarray(target_rows), args, args.epochs_per_iter)
        bootstrap_loss = all_losses[-1]
    bootstrap_path = save_checkpoint(model, out, "iter_00_bootstrap_model")
    bootstrap_eval = evaluate_rollout(model, args, args.seed + 1000)
    bootstrap_record = {
        "iteration": 0,
        "model_path": bootstrap_path,
        "loss": bootstrap_loss,
        "samples": len(obs_rows),
        "noise": 0.0,
        "collection_mean_ep_len": None,
        "collection_max_ep_len": None,
        "eval": bootstrap_eval,
    }
    iteration_summaries.append(bootstrap_record)
    best_eval = bootstrap_record
    model.save(str(best_model_path))
    print(
        f"[DAGGER] bootstrap loss={bootstrap_loss if bootstrap_loss is not None else 'skipped'} samples={len(obs_rows)} "
        f"eval_steps={bootstrap_eval['steps']} reason={bootstrap_eval['terminated_reason']}"
    )

    for it in range(args.dagger_iters):
        obs, _ = env.reset(seed=args.seed + it + 1)
        start_y = float(env.data.xpos[env._trunk_body_id][1])
        start_yaw = yaw_from_wxyz(np.asarray(env.data.qpos[3:7], dtype=np.float32))
        ep_lens: list[int] = []
        ep_len = 0
        added_samples = 0
        noise_std = scheduled_noise(args, it)
        for _ in range(args.rollout_samples):
            reference_target = env.get_reference_action()
            target = balance_corrected_target(
                env,
                reference_target,
                args,
                start_y=start_y,
                start_yaw=start_yaw,
            )
            if is_recovery_state(env, args, start_yaw=start_yaw):
                obs_rows.append(np.asarray(obs, dtype=np.float32).copy())
                target_rows.append(np.asarray(target, dtype=np.float32).copy())
                added_samples += 1
            action, _ = model.predict(obs, deterministic=True)
            action = np.asarray(action, dtype=np.float32)
            if noise_std > 0:
                action = action + rng.normal(0.0, noise_std, size=action.shape).astype(np.float32)
            action = np.clip(action, env.action_space.low, env.action_space.high)
            obs, _, terminated, truncated, _ = env.step(action)
            ep_len += 1
            if terminated or truncated:
                ep_lens.append(ep_len)
                ep_len = 0
                obs, _ = env.reset()
                start_y = float(env.data.xpos[env._trunk_body_id][1])
                start_yaw = yaw_from_wxyz(np.asarray(env.data.qpos[3:7], dtype=np.float32))
        if ep_len:
            ep_lens.append(ep_len)
        if added_samples <= 0:
            print(f"[DAGGER] iter={it + 1} recovery filter added no samples; skipping supervised update")
            continue
        losses = train_supervised(model, np.asarray(obs_rows), np.asarray(target_rows), args, args.epochs_per_iter)
        all_losses += losses
        iter_model_path = save_checkpoint(model, out, f"iter_{it + 1:02d}_model")
        iter_eval = evaluate_rollout(model, args, args.seed + 1001 + it)
        record = {
            "iteration": it + 1,
            "model_path": iter_model_path,
            "loss": losses[-1],
            "samples": len(obs_rows),
            "added_samples": added_samples,
            "noise": noise_std,
            "collection_mean_ep_len": float(np.mean(ep_lens)) if ep_lens else 0.0,
            "collection_max_ep_len": max(ep_lens) if ep_lens else 0,
            "eval": iter_eval,
        }
        iteration_summaries.append(record)
        if best_eval is None or float(iter_eval["selection_score"]) > float(best_eval["eval"]["selection_score"]):
            best_eval = record
            model.save(str(best_model_path))
        print(
            f"[DAGGER] iter={it + 1} loss={losses[-1]:.6f} "
            f"noise={noise_std:.4f} "
            f"samples={len(obs_rows)} added={added_samples} "
            f"mean_ep_len={float(np.mean(ep_lens)) if ep_lens else 0:.1f} "
            f"max_ep_len={max(ep_lens) if ep_lens else 0} "
            f"eval_steps={iter_eval['steps']} reason={iter_eval['terminated_reason']}"
        )

    obs_eval = np.asarray(obs_rows[-min(len(obs_rows), 4096):], dtype=np.float32)
    target_eval = np.asarray(target_rows[-obs_eval.shape[0]:], dtype=np.float32)
    with torch.no_grad():
        pred = actor_mean(model.policy, torch.as_tensor(obs_eval, dtype=torch.float32, device=model.device))
        tgt = torch.as_tensor(target_eval, dtype=torch.float32, device=model.device)
        mae = torch.mean(torch.abs(pred - tgt)).item()
        mse = torch.mean((pred - tgt) ** 2).item()

    model_path = out / "dagger_model.zip"
    model.save(str(model_path))
    summary = {
        "model_path": str(model_path),
        "best_model_path": str(best_model_path),
        "best_iteration": None if best_eval is None else best_eval["iteration"],
        "best_eval": None if best_eval is None else best_eval["eval"],
        "samples": len(obs_rows),
        "final_loss": all_losses[-1] if all_losses else None,
        "eval_action_mae": mae,
        "eval_action_mse": mse,
        "motion_path": args.motion_path,
        "reference_obs_scope": args.reference_obs_scope,
        "reference_time_scale": args.reference_time_scale,
        "foot_state_obs": args.foot_state_obs,
        "balance_corrected_labels": args.balance_corrected_labels,
        "balance_correction_max": args.balance_correction_max,
        "balance_width_target": args.balance_width_target,
        "balance_side_target": args.balance_side_target,
        "balance_side_gain": args.balance_side_gain,
        "balance_phase_aware": args.balance_phase_aware,
        "iteration_summaries": iteration_summaries,
    }
    (out / "summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    print(json.dumps(summary, indent=2))
    env.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
