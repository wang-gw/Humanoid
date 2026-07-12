"""Walking reward for the urdf_f_v2 model — the v2 curriculum's own playground.

This module owns the reward shaping for the new-geometry model so it can diverge
freely without touching the frozen urdf_f (old model) reward in
`envs/urdf_f_env.py`. It starts as a faithful copy of that model's `walking`
task reward; edit the marked block below to tune the v2 gait curriculum.

`GaitWalkingV2Env._reward` delegates here. It implements only the `walking` task
(the only task the v2 env uses); other tasks fall back to the base via the env.
"""
from __future__ import annotations

import numpy as np

from envs.urdf_f_env import quat_to_rpy


def compute_walking_reward(env, action: np.ndarray) -> tuple[float, dict[str, float]]:
    """Reward for one v2 walking step. Signature mirrors `UrdfFEnv._reward`."""
    q = env.data.qpos[env.joint_qposadr]
    qd = env.data.qvel[env.joint_dofadr]
    roll, pitch, _ = quat_to_rpy(np.asarray(env.data.qpos[3:7], dtype=np.float64))
    stats = env._contact_stats()
    clearance = env._right_foot_clearance()

    # ── Generic reward frame (shared shape with base; model-agnostic) ──────────
    upright_penalty = roll * roll + pitch * pitch
    stability_excess_penalty = (
        max(abs(roll) - env.clearance_gate_roll, 0.0) ** 2
        + max(abs(pitch) - env.clearance_gate_pitch, 0.0) ** 2
    )
    vel_penalty = float(np.mean(qd * qd))
    action_penalty = float(np.mean(np.square(action)))
    action_delta_penalty = float(np.mean(np.square(action - env.prev_action))) if action.size else 0.0
    posture_penalty = float(np.mean(np.square(q - env.nominal_q)))
    stable_for_clearance = (
        abs(roll) <= env.clearance_gate_roll and abs(pitch) <= env.clearance_gate_pitch
    )
    swing_step_reward = 0.0
    weight_shift_reward = 0.0
    swing_contact_penalty = 0.0
    foot_split_penalty = 0.0

    # ══════════════ v2 WALKING REWARD — tune the gait curriculum here ══════════
    vel_y = float(env.data.qvel[1])
    vel_reward = np.clip(vel_y / 0.05, -0.5, 1.0) * 3.0
    left_clr = env._left_foot_clearance()
    right_clr = env._right_foot_clearance()
    left_foot_y = env._left_foot_y()
    right_foot_y = env._right_foot_y()
    dt = env.frame_skip * float(env.model.opt.timestep)

    phase = env._gait_phase()
    right_swing = np.sin(phase) > 0
    swing_clr = right_clr if right_swing else left_clr
    stance_contacts = stats["left_contacts"] if right_swing else stats["right_contacts"]
    gait_reward = np.clip(swing_clr / 0.004, 0.0, 1.5)
    contact_reward = min(stance_contacts, 1.0)

    if env.swing_step_reward_weight > 0.0:
        swing_dy = (right_foot_y - env._prev_right_foot_y) if right_swing \
            else (left_foot_y - env._prev_left_foot_y)
        swing_speed = swing_dy / max(dt, 1e-6)
        swing_step_reward = env.swing_step_reward_weight * float(
            np.clip(swing_speed / max(env.swing_step_target_speed, 1e-6), -0.5, 1.0)
        )
    if env.weight_shift_reward_weight > 0.0:
        stance_ratio = stats["left_force_ratio"] if right_swing else stats["right_force_ratio"]
        weight_shift_reward = env.weight_shift_reward_weight * float(
            np.clip((stance_ratio - 0.5) / 0.5, 0.0, 1.0)
        )
    if env.swing_contact_penalty_weight > 0.0:
        swing_contacts = stats["right_contacts"] if right_swing else stats["left_contacts"]
        swing_contact_penalty = env.swing_contact_penalty_weight * min(swing_contacts, 1.0)

    both_air = (left_clr > 0.005) and (right_clr > 0.005)
    flat_foot_penalty = env._flat_foot_penalty(
        stats["left_contacts"] > 0, stats["right_contacts"] > 0
    )
    if env.foot_split_penalty_weight > 0.0:
        split = abs(left_foot_y - right_foot_y)
        foot_split_penalty = env.foot_split_penalty_weight * float(
            np.clip((split - env.foot_split_deadzone) / max(env.foot_split_scale, 1e-6), 0.0, 1.0)
        )

    task_reward = (
        vel_reward + gait_reward + swing_step_reward + weight_shift_reward
        - (2.0 if both_air else 0.0) - flat_foot_penalty - swing_contact_penalty
        - foot_split_penalty
    )
    env._prev_left_foot_y = left_foot_y
    env._prev_right_foot_y = right_foot_y
    # ═══════════════════════ end v2 walking reward ════════════════════════════

    reward = (
        1.0
        + contact_reward
        + task_reward
        - env.upright_penalty_weight * upright_penalty
        - 0.02 * vel_penalty
        - env.action_penalty_weight * action_penalty
        - env.action_delta_penalty_weight * action_delta_penalty
        - 0.3 * posture_penalty
        - env.right_contact_penalty_weight * min(stats["right_contacts"], 4.0)
        - env.left_contact_penalty_weight * min(stats["left_contacts"], 4.0)
        - env.stability_excess_penalty_weight * stability_excess_penalty
        - (env.pose_tracking_penalty_weight - 0.3) * posture_penalty
    )
    terminated, _ = env._terminated()
    if terminated:
        reward -= env.fall_penalty

    info = {
        "reward_total": float(reward),
        "reward_contact": float(contact_reward),
        "reward_task": float(task_reward),
        "penalty_upright": float(upright_penalty),
        "penalty_joint_vel": float(vel_penalty),
        "penalty_action": float(action_penalty),
        "penalty_action_delta": float(action_delta_penalty),
        "penalty_right_contacts": float(min(stats["right_contacts"], 4.0)),
        "penalty_left_contacts": float(min(stats["left_contacts"], 4.0)),
        "penalty_fall": float(env.fall_penalty if terminated else 0.0),
        "penalty_stability_excess": float(stability_excess_penalty),
        "penalty_posture": float(posture_penalty),
        "penalty_flat_foot": float(flat_foot_penalty),
        "reward_swing_step": float(swing_step_reward),
        "reward_weight_shift": float(weight_shift_reward),
        "penalty_swing_contact": float(swing_contact_penalty),
        "penalty_foot_split": float(foot_split_penalty),
        "left_sole_tilt": float(env._foot_sole_tilt(env._left_foot_body_id)),
        "right_sole_tilt": float(env._foot_sole_tilt(env._right_foot_body_id)),
        "clearance_reward_gated": 0.0,
        "left_clearance": float(env._left_foot_clearance()),
        "right_clearance": float(clearance),
        **{key: float(value) for key, value in stats.items()},
    }
    return float(reward), info
