from __future__ import annotations

from typing import Any

import numpy as np

from policies.common import home_action


def _humanoid_foot_rel_x(env: Any, side: str) -> float:
    try:
        site_id = int(env.model.site(f"{side}_foot").id)
        foot_pos = env.data.site_xpos[site_id]
        trunk_pos = env.data.xpos[env._trunk_body_id]
        return float(foot_pos[0] - trunk_pos[0])
    except Exception:
        return 0.0


def _humanoid_foot_rel_z(env: Any, side: str) -> float:
    try:
        site_id = int(env.model.site(f"{side}_foot").id)
        return float(env.data.site_xpos[site_id][2])
    except Exception:
        return 0.0


def _humanoid_foot_rel_y(env: Any, side: str) -> float:
    try:
        site_id = int(env.model.site(f"{side}_foot").id)
        foot_pos = env.data.site_xpos[site_id]
        trunk_pos = env.data.xpos[env._trunk_body_id]
        return float(foot_pos[1] - trunk_pos[1])
    except Exception:
        return 0.0


def nominal_action(env: Any, controller: str) -> np.ndarray:
    if controller == "home":
        return home_action(env)
    if controller == "humanoid_walk":
        return humanoid_walk_action(env)
    if controller == "footstep_ik":
        return footstep_ik_action(env)
    if controller == "footstep_ik_capture":
        return footstep_ik_capture_action(env)
    if controller == "footstep_ik_landing":
        return footstep_ik_landing_action(env)
    if controller == "footstep_ik_landing_governed":
        return footstep_ik_landing_governed_action(env)
    if controller == "footstep_ik_landing_governed_revbrake":
        return footstep_ik_landing_governed_action(env, brake_sign=-1.0)
    if controller == "footstep_ik_lipm_capture":
        return footstep_ik_lipm_capture_action(env)
    if controller == "footstep_ik_forward":
        return footstep_ik_forward_action(env)
    if controller == "footstep_ik_walkform":
        return footstep_ik_walkform_action(env)
    if controller == "footstep_ik_landing_strong":
        return footstep_ik_landing_strong_action(env)
    if controller == "footstep_ik_landing_upright":
        return footstep_ik_landing_upright_action(env)
    if controller == "footstep_ik_landing_lateral":
        return footstep_ik_landing_lateral_action(env)
    if controller == "footstep_ik_landing_lateral_soft":
        return footstep_ik_landing_lateral_action(env, gain=0.22, ankle_gain=0.16)
    if controller == "footstep_ik_landing_lateral_mid":
        return footstep_ik_landing_lateral_action(env, gain=0.28, ankle_gain=0.20)
    if controller == "footstep_cartesian":
        return footstep_cartesian_action(env)
    if controller == "footstep_capture":
        return footstep_capture_action(env)
    if controller == "footstep_capture_aggressive":
        return footstep_capture_aggressive_action(env)
    if controller == "footstep_capture_earlycatch":
        return footstep_capture_earlycatch_action(env)
    if controller == "footstep_capture_swingcatch":
        return footstep_capture_swingcatch_action(env)
    if controller == "footstep_capture_speedbrake":
        return footstep_capture_speedbrake_action(env)
    if controller == "footstep_capture_placement":
        return footstep_capture_placement_action(env)
    if controller == "footstep_capture_pitchcatch":
        return footstep_capture_pitchcatch_action(env)
    if controller == "footstep_capture_toelift":
        return footstep_capture_toelift_action(env)
    if controller == "footstep_capture_highclearance":
        return footstep_capture_highclearance_action(env)
    if controller == "footstep_march":
        return footstep_march_action(env)
    if controller == "footstep_capture_lipm":
        return footstep_capture_lipm_action(env)
    if controller == "footstep_capture_lipm_swingcatch":
        return footstep_capture_lipm_swingcatch_action(env)
    if controller == "footstep_capture_lipm_aggressive":
        return footstep_capture_lipm_aggressive_action(env)
    if controller == "footstep_bounded":
        return footstep_bounded_action(env)
    if controller == "reference_motion":
        return reference_motion_action(env)
    if controller == "reference_left_forward":
        return reference_left_forward_action(env)
    if controller == "reference_commanded":
        return reference_commanded_action(env)
    if controller == "reference_stabilized":
        return reference_stabilized_action(env)
    if controller == "reference_capture":
        return reference_capture_action(env)
    if controller == "reference_capture_soft":
        return reference_capture_action(env, placement_gain=0.12, lift_gain=0.55, brake_gain=0.05)
    if controller == "reference_yaw_stable":
        return reference_yaw_stable_action(env)
    if controller == "reference_yaw_stable_softroll":
        return reference_yaw_stable_action(env, roll_gain=0.015)
    if controller == "reference_yaw_stable_midroll":
        return reference_yaw_stable_action(env, roll_gain=0.030)
    if controller == "reference_yaw_stable_velroll":
        return reference_yaw_stable_action(
            env,
            base_y_gain=0.30,
            base_y_vel_gain=0.075,
            roll_gain=0.030,
        )
    if controller == "reference_yaw_stable_grounded":
        return reference_yaw_stable_action(env, roll_gain=0.030, ground_guard=True)
    if controller == "reference_yaw_stable_footcap":
        return reference_yaw_stable_action(env, roll_gain=0.030, foot_capture_gain=0.070)
    if controller == "reference_yaw_stable_footcap_rev":
        return reference_yaw_stable_action(env, roll_gain=0.030, foot_capture_gain=-0.070)
    if controller == "reference_yaw_stable_footcap_revsoft":
        return reference_yaw_stable_action(env, roll_gain=0.030, foot_capture_gain=-0.025)
    if controller == "reference_yaw_stable_footcap_revsoft_softroll":
        return reference_yaw_stable_action(env, roll_gain=0.015, foot_capture_gain=-0.025)
    if controller == "reference_yaw_stable_footcap_scheduled":
        return reference_yaw_stable_action(
            env,
            roll_gain=0.015,
            foot_capture_gain=-0.014,
            foot_capture_boost_gain=-0.038,
            foot_capture_boost_threshold=0.34,
        )
    if controller == "reference_yaw_stable_supportcap":
        return reference_yaw_stable_action(
            env,
            roll_gain=0.015,
            foot_capture_gain=-0.014,
            support_lateral_gain=0.34,
        )
    if controller == "reference_yaw_stable_supportcap_soft":
        return reference_yaw_stable_action(
            env,
            roll_gain=0.015,
            foot_capture_gain=-0.014,
            support_lateral_gain=0.18,
            support_lateral_deadband=0.08,
        )
    if controller == "reference_yaw_stable_supportcap_tiny":
        return reference_yaw_stable_action(
            env,
            roll_gain=0.015,
            foot_capture_gain=-0.014,
            support_lateral_gain=0.11,
            support_lateral_deadband=0.10,
        )
    if controller == "reference_yaw_stable_supportcap_late":
        return reference_yaw_stable_action(
            env,
            roll_gain=0.015,
            foot_capture_gain=-0.014,
            support_lateral_gain=0.30,
            support_lateral_deadband=0.12,
            support_lateral_activation_y=0.22,
            support_lateral_activation_roll=0.42,
        )
    if controller == "reference_yaw_stable_stanceguard":
        return reference_yaw_stable_action(
            env,
            roll_gain=0.015,
            foot_capture_gain=-0.014,
            stance_contact_guard=True,
        )
    if controller == "reference_yaw_stable_stanceguard_latecap":
        return reference_yaw_stable_action(
            env,
            roll_gain=0.015,
            foot_capture_gain=-0.014,
            stance_contact_guard=True,
            support_lateral_gain=0.24,
            support_lateral_deadband=0.13,
            support_lateral_activation_y=0.24,
            support_lateral_activation_roll=0.46,
        )
    if controller == "reference_yaw_stable_stanceguard_revcap":
        return reference_yaw_stable_action(
            env,
            roll_gain=0.015,
            foot_capture_gain=-0.014,
            stance_contact_guard=True,
            support_lateral_gain=0.24,
            support_lateral_sign=-1.0,
            support_lateral_deadband=0.09,
        )
    if controller == "reference_yaw_stable_stanceguard_revcap_soft":
        return reference_yaw_stable_action(
            env,
            roll_gain=0.015,
            foot_capture_gain=-0.014,
            stance_contact_guard=True,
            support_lateral_gain=0.14,
            support_lateral_sign=-1.0,
            support_lateral_deadband=0.11,
        )
    if controller == "reference_yaw_stable_stanceguard_width":
        return reference_yaw_stable_action(
            env,
            roll_gain=0.015,
            foot_capture_gain=-0.014,
            stance_contact_guard=True,
            foot_width_guard_gain=0.30,
            foot_width_guard_target=0.20,
        )
    if controller == "reference_yaw_stable_stanceguard_width_lift":
        return reference_yaw_stable_action(
            env,
            roll_gain=0.015,
            foot_capture_gain=-0.014,
            stance_contact_guard=True,
            foot_width_guard_gain=0.30,
            foot_width_guard_target=0.20,
            swing_clearance_gain=1.0,
            swing_clearance_target=0.105,
            swing_forward_target=-0.035,
        )
    if controller == "reference_yaw_stable_stanceguard_width_rightlift":
        return reference_yaw_stable_action(
            env,
            roll_gain=0.015,
            foot_capture_gain=-0.014,
            stance_contact_guard=True,
            foot_width_guard_gain=0.30,
            foot_width_guard_target=0.20,
            swing_clearance_gain=1.0,
            swing_clearance_right_gain=1.55,
            swing_clearance_target=0.110,
            swing_forward_target=-0.025,
        )
    if controller == "reference_yaw_stable_stanceguard_width_rightstance":
        return reference_yaw_stable_action(
            env,
            roll_gain=0.015,
            foot_capture_gain=-0.014,
            stance_contact_guard=True,
            stance_contact_right_gain=1.45,
            foot_width_guard_gain=0.34,
            foot_width_guard_target=0.20,
            swing_clearance_gain=0.95,
            swing_clearance_right_gain=1.35,
            swing_clearance_target=0.105,
            swing_forward_target=-0.025,
        )
    if controller == "reference_yaw_stable_stanceguard_width_rightstance_tight":
        return reference_yaw_stable_action(
            env,
            roll_gain=0.015,
            foot_capture_gain=-0.014,
            stance_contact_guard=True,
            stance_contact_right_gain=1.45,
            foot_width_guard_gain=0.55,
            foot_width_guard_target=0.16,
            swing_clearance_gain=0.95,
            swing_clearance_right_gain=1.35,
            swing_clearance_target=0.105,
            swing_forward_target=-0.025,
        )
    if controller == "reference_yaw_stable_stanceguard_width_rightstance_lat":
        return reference_yaw_stable_action(
            env,
            roll_gain=0.015,
            foot_capture_gain=-0.014,
            support_lateral_gain=0.10,
            support_lateral_sign=-1.0,
            support_lateral_deadband=0.08,
            support_lateral_activation_y=0.05,
            support_lateral_activation_roll=0.08,
            stance_contact_guard=True,
            stance_contact_right_gain=1.35,
            foot_width_guard_gain=0.34,
            foot_width_guard_target=0.20,
            swing_clearance_gain=0.95,
            swing_clearance_right_gain=1.30,
            swing_clearance_target=0.105,
            swing_forward_target=-0.025,
        )
    if controller == "reference_yaw_stable_stanceguard_width_rightstance_speed":
        return reference_yaw_stable_action(
            env,
            roll_gain=0.015,
            foot_capture_gain=-0.014,
            stance_contact_guard=True,
            stance_contact_right_gain=1.45,
            foot_width_guard_gain=0.34,
            foot_width_guard_target=0.20,
            swing_clearance_gain=0.95,
            swing_clearance_right_gain=1.35,
            swing_clearance_target=0.105,
            swing_forward_target=-0.025,
            speed_brake_gain=0.38,
        )
    if controller == "reference_yaw_stable_stanceguard_width_rightstance_governed":
        return reference_yaw_stable_action(
            env,
            roll_gain=0.015,
            foot_capture_gain=-0.014,
            stance_contact_guard=True,
            stance_contact_right_gain=1.45,
            foot_width_guard_gain=0.34,
            foot_width_guard_target=0.20,
            swing_clearance_gain=0.95,
            swing_clearance_right_gain=1.35,
            swing_clearance_target=0.105,
            swing_forward_target=-0.025,
            catch_overspeed_gain=0.25,
            speed_brake_gain=0.42,
            speed_brake_start_time=1.45,
            speed_brake_threshold=0.22,
            speed_brake_max=0.32,
        )
    if controller == "reference_yaw_stable_stanceguard_width_rightstance_decoupled":
        return reference_yaw_stable_action(
            env,
            roll_gain=0.015,
            foot_capture_gain=-0.014,
            stance_contact_guard=True,
            stance_contact_right_gain=1.45,
            foot_width_guard_gain=0.34,
            foot_width_guard_target=0.20,
            swing_clearance_gain=0.95,
            swing_clearance_right_gain=1.35,
            swing_clearance_target=0.105,
            swing_forward_target=-0.025,
            catch_overspeed_gain=0.25,
        )
    if controller == "reference_yaw_stable_stanceguard_width_rightstance_ground":
        return reference_yaw_stable_action(
            env,
            roll_gain=0.015,
            foot_capture_gain=-0.014,
            ground_guard=True,
            stance_contact_guard=True,
            stance_contact_right_gain=1.45,
            foot_width_guard_gain=0.34,
            foot_width_guard_target=0.20,
            swing_clearance_gain=0.90,
            swing_clearance_right_gain=1.30,
            swing_clearance_target=0.100,
            swing_forward_target=-0.025,
        )
    if controller == "reference_yaw_stable_stanceguard_width_rightstance_plant":
        return reference_yaw_stable_action(
            env,
            roll_gain=0.015,
            foot_capture_gain=-0.014,
            stance_contact_guard=True,
            stance_contact_right_gain=1.45,
            flight_plant_guard=True,
            foot_width_guard_gain=0.34,
            foot_width_guard_target=0.20,
            swing_clearance_gain=0.90,
            swing_clearance_right_gain=1.30,
            swing_clearance_target=0.100,
            swing_forward_target=-0.025,
        )
    if controller == "reference_yaw_stable_stanceguard_width_soft":
        return reference_yaw_stable_action(
            env,
            roll_gain=0.015,
            foot_capture_gain=-0.014,
            stance_contact_guard=True,
            foot_width_guard_gain=0.18,
            foot_width_guard_target=0.21,
        )
    if controller == "reference_yaw_stable_stanceguard_width_pitch":
        return reference_yaw_stable_action(
            env,
            roll_gain=0.015,
            foot_capture_gain=-0.014,
            stance_contact_guard=True,
            foot_width_guard_gain=0.30,
            foot_width_guard_target=0.20,
            pitch_catch_gain=0.85,
        )
    if controller == "reference_yaw_stable_stanceguard_width_speed":
        return reference_yaw_stable_action(
            env,
            roll_gain=0.015,
            foot_capture_gain=-0.014,
            stance_contact_guard=True,
            foot_width_guard_gain=0.30,
            foot_width_guard_target=0.20,
            speed_brake_gain=0.80,
        )
    if controller == "reference_yaw_stable_stanceguard_width_vel":
        return reference_yaw_stable_action(
            env,
            base_y_vel_gain=0.12,
            roll_gain=0.015,
            foot_capture_gain=-0.014,
            stance_contact_guard=True,
            foot_width_guard_gain=0.30,
            foot_width_guard_target=0.20,
        )
    if controller == "reference_yaw_stable_speedbrake":
        return reference_yaw_stable_action(
            env,
            roll_gain=0.015,
            foot_capture_gain=-0.014,
            speed_brake_gain=1.0,
        )
    if controller == "reference_yaw_stable_pitchcatch":
        return reference_yaw_stable_action(
            env,
            roll_gain=0.015,
            foot_capture_gain=-0.014,
            pitch_catch_gain=1.0,
        )
    if controller == "reference_yaw_stable_revgovernor":
        return reference_yaw_stable_action(
            env,
            roll_gain=0.015,
            foot_capture_gain=-0.014,
            speed_governor_hip_brake_sign=1.0,
        )
    if controller == "reference_yaw_stable_supportcap_rev":
        return reference_yaw_stable_action(
            env,
            roll_gain=0.015,
            foot_capture_gain=-0.014,
            support_lateral_gain=0.34,
            support_lateral_sign=-1.0,
        )
    if controller == "reference_yaw_stable_revroll":
        return reference_yaw_stable_action(env, roll_gain=-0.035)
    if controller == "reference_lift_only":
        return reference_capture_action(env, placement_gain=0.0, lift_gain=0.45, brake_gain=0.04)
    if controller == "reference_clearance":
        return reference_clearance_action(env)
    if controller == "reference_footplace":
        return reference_footplace_action(env)
    if controller == "reference_latecatch":
        return reference_latecatch_action(env)
    if controller == "reference_speedgoverned":
        return reference_speedgoverned_action(env)
    if controller == "reference_landing":
        return reference_landing_action(env)
    if controller == "reference_hipgoverned":
        return reference_hipgoverned_action(env)
    if controller == "reference_support_stable":
        return reference_support_stable_action(env)
    if controller == "reference_pelvis_stable":
        return reference_pelvis_stable_action(env)
    if controller == "reference_phase_footstep":
        return reference_phase_footstep_action(env)
    raise ValueError(f"Unknown G1 controller '{controller}'.")


def reference_motion_action(env: Any) -> np.ndarray:
    get_reference_action = getattr(env, "get_reference_action", None)
    if callable(get_reference_action):
        return get_reference_action()
    return home_action(env)


def reference_left_forward_action(env: Any) -> np.ndarray:
    base = reference_motion_action(env)
    if env.model.nu < 29 or env.stage != "walk":
        return base

    get_reference_contact = getattr(env, "get_reference_contact", None)
    ref_contact = (
        np.asarray(get_reference_contact(), dtype=np.float32)
        if callable(get_reference_contact)
        else np.zeros(2, dtype=np.float32)
    )
    if ref_contact.shape[0] >= 2 and float(np.max(ref_contact)) > 0.0:
        left_swing = float(np.clip(1.0 - ref_contact[0], 0.0, 1.0))
        right_stance = float(np.clip(ref_contact[1], 0.0, 1.0))
    else:
        raw = float(np.sin(env.get_gait_phase()))
        left_swing = max(raw, 0.0)
        right_stance = 1.0 if left_swing > 0.0 else 0.0

    left_rel_x = _humanoid_foot_rel_x(env, "left")
    left_foot_z = _humanoid_foot_rel_z(env, "left")
    forward_need = float(np.clip((0.055 - left_rel_x) / 0.22, 0.0, 1.0)) * left_swing
    lift_need = float(np.clip((0.105 - left_foot_z) / 0.10, 0.0, 1.0)) * left_swing
    if forward_need > 1e-6 or lift_need > 1e-6:
        need = max(forward_need, 0.65 * lift_need)
        base[0] += float(np.clip(0.115 * need, 0.0, 0.115))
        base[3] += float(np.clip(0.085 * max(need, lift_need), 0.0, 0.085))
        base[4] += float(np.clip(0.040 * lift_need, 0.0, 0.040))

    if right_stance > 0.5:
        # Keep the right stance leg from folding while left foot searches for
        # a forward touchdown.
        base[9] -= float(np.clip(0.035 * right_stance, 0.0, 0.035))
        base[10] -= float(np.clip(0.018 * right_stance, 0.0, 0.018))

    projected_gravity = env._get_projected_gravity()
    pitch_like = float(np.clip(projected_gravity[0], -0.6, 0.6))
    base[14] += float(np.clip(-0.08 * pitch_like, -0.08, 0.08))
    return np.clip(base, env._ctrl_low, env._ctrl_high).astype(np.float32)


def _base_yaw_wxyz(env: Any) -> float:
    try:
        quat = np.asarray(env.data.qpos[3:7], dtype=np.float64)
        w, x, y, z = quat
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return float(np.arctan2(siny_cosp, cosy_cosp))
    except Exception:
        return 0.0


def _apply_g1_yaw_lateral_stabilizer(
    env: Any,
    base: np.ndarray,
    *,
    gain: float = 0.5,
    yaw_direction: float = 1.0,
    lateral_direction: float = 1.0,
    base_y_gain: float = 0.24,
    base_y_vel_gain: float = 0.0,
    roll_gain: float = 0.045,
    ground_guard: bool = False,
    flight_plant_guard: bool = False,
    foot_capture_gain: float = 0.0,
    foot_capture_boost_gain: float = 0.0,
    foot_capture_boost_threshold: float = 0.35,
    support_lateral_gain: float = 0.0,
    support_lateral_sign: float = 1.0,
    support_lateral_deadband: float = 0.0,
    support_lateral_activation_y: float = 0.0,
    support_lateral_activation_roll: float = 0.0,
    stance_contact_guard: bool = False,
    stance_contact_right_gain: float = 1.0,
    speed_brake_gain: float = 0.0,
    pitch_catch_gain: float = 0.0,
) -> np.ndarray:
    if env.model.nu < 29:
        return base
    projected_gravity = env._get_projected_gravity()
    roll_like = float(np.clip(projected_gravity[1], -0.6, 0.6))
    yaw = float(np.clip(_base_yaw_wxyz(env), -0.9, 0.9))
    yaw_rate = float(np.clip(env.data.qvel[5], -2.5, 2.5)) if getattr(env, "data", None) is not None else 0.0
    base_y = float(env.data.xpos[env._trunk_body_id][1]) if getattr(env, "data", None) is not None else 0.0
    base_y_vel = float(np.clip(env.data.qvel[1], -2.0, 2.0)) if getattr(env, "data", None) is not None else 0.0

    lateral = float(
        np.clip(
            lateral_direction
            * gain
            * (-base_y_gain * base_y - base_y_vel_gain * base_y_vel - roll_gain * roll_like),
            -0.060,
            0.060,
        )
    )
    base[1] += lateral
    base[7] += lateral
    base[5] -= 0.20 * lateral
    base[11] -= 0.20 * lateral

    yaw_corr = float(np.clip(yaw_direction * gain * (-0.12 * yaw - 0.025 * yaw_rate), -0.075, 0.075))
    base[2] += yaw_corr
    base[8] += yaw_corr
    base[12] += float(np.clip(yaw_direction * gain * (-0.18 * yaw - 0.04 * yaw_rate), -0.12, 0.12))
    return base


def reference_yaw_stable_action(
    env: Any,
    *,
    base_y_gain: float = 0.24,
    base_y_vel_gain: float = 0.0,
    roll_gain: float = 0.045,
    ground_guard: bool = False,
    flight_plant_guard: bool = False,
    foot_capture_gain: float = 0.0,
    foot_capture_boost_gain: float = 0.0,
    foot_capture_boost_threshold: float = 0.35,
    support_lateral_gain: float = 0.0,
    support_lateral_sign: float = 1.0,
    support_lateral_deadband: float = 0.0,
    support_lateral_activation_y: float = 0.0,
    support_lateral_activation_roll: float = 0.0,
    stance_contact_guard: bool = False,
    stance_contact_right_gain: float = 1.0,
    speed_brake_gain: float = 0.0,
    speed_brake_start_time: float = 0.0,
    speed_brake_threshold: float = 0.06,
    speed_brake_max: float = 0.45,
    pitch_catch_gain: float = 0.0,
    catch_overspeed_gain: float = 1.30,
    speed_governor_hip_brake_sign: float = -1.0,
    foot_width_guard_gain: float = 0.0,
    foot_width_guard_target: float = 0.20,
    swing_clearance_gain: float = 0.0,
    swing_clearance_right_gain: float = 1.15,
    swing_clearance_target: float = 0.105,
    swing_forward_target: float = -0.035,
) -> np.ndarray:
    base = reference_speedgoverned_action(env, hip_brake_sign=speed_governor_hip_brake_sign)
    if env.model.nu < 29 or env.stage != "walk":
        return base
    get_reference_contact = getattr(env, "get_reference_contact", None)
    ref_contact = (
        np.asarray(get_reference_contact(), dtype=np.float32)
        if callable(get_reference_contact)
        else np.zeros(2, dtype=np.float32)
    )
    actual_contact = np.asarray(env._get_foot_contacts(), dtype=np.float32)[:2]
    phase = env.get_gait_phase()
    raw = float(np.sin(phase))
    if ref_contact.shape[0] >= 2 and float(np.max(ref_contact)) > 0.0:
        left_swing = float(np.clip(1.0 - ref_contact[0], 0.0, 1.0))
        right_swing = float(np.clip(1.0 - ref_contact[1], 0.0, 1.0))
    else:
        left_swing = max(0.0, raw)
        right_swing = max(0.0, -raw)
    left_contact = float(actual_contact[0]) if actual_contact.shape[0] > 0 else 0.0
    right_contact = float(actual_contact[1]) if actual_contact.shape[0] > 1 else 0.0

    vx = float(env.data.qvel[0]) if getattr(env, "data", None) is not None else 0.0
    cmd_vx = float(getattr(env, "command", np.zeros(3, dtype=np.float32))[0])
    overspeed = max(vx - cmd_vx - 0.08, 0.0)
    projected_gravity = env._get_projected_gravity()
    gravity_z = float(projected_gravity[2])
    tilt_need = max(gravity_z + 0.78, 0.0)
    left_rel = _humanoid_foot_rel_x(env, "left")
    right_rel = _humanoid_foot_rel_x(env, "right")
    both_behind = max(-0.12 - min(left_rel, right_rel), 0.0)
    catch_need = float(
        np.clip(float(catch_overspeed_gain) * overspeed + 1.05 * tilt_need + 2.8 * both_behind, 0.0, 1.0)
    )
    if catch_need > 1e-6:
        for hip_i, knee_i, ankle_i, swing, contact, rel_x in (
            (0, 3, 4, left_swing, left_contact, left_rel),
            (6, 9, 10, right_swing, right_contact, right_rel),
        ):
            lag = float(np.clip((-0.04 - rel_x) / 0.34, 0.0, 1.0))
            gate = float(np.clip(0.35 + 0.55 * swing + 0.25 * (1.0 - contact), 0.0, 1.0))
            need = catch_need * max(lag, 0.25) * gate
            base[hip_i] -= float(np.clip(0.088 * need, 0.0, 0.088))
            base[knee_i] += float(np.clip(0.075 * need, 0.0, 0.075))
            base[ankle_i] += float(np.clip(0.034 * need, 0.0, 0.034))
        base[14] += float(np.clip(-0.10 * catch_need - 0.10 * overspeed, -0.16, 0.03))
    if abs(foot_capture_gain) > 1e-9:
        base_y = float(env.data.xpos[env._trunk_body_id][1])
        base_y_vel = float(np.clip(env.data.qvel[1], -2.0, 2.0))
        roll_like = float(np.clip(projected_gravity[1], -0.8, 0.8))
        capture_y = float(np.clip(0.85 * base_y + 0.24 * base_y_vel + 0.20 * roll_like, -1.0, 1.0))
        boost = float(np.clip((abs(capture_y) - foot_capture_boost_threshold) / 0.35, 0.0, 1.0))
        capture_gain = foot_capture_gain + foot_capture_boost_gain * boost
        left_gate = float(np.clip(left_swing + 0.35 * (1.0 - left_contact), 0.0, 1.0))
        right_gate = float(np.clip(right_swing + 0.35 * (1.0 - right_contact), 0.0, 1.0))
        base[1] += float(np.clip(capture_gain * capture_y * left_gate, -0.085, 0.085))
        base[7] += float(np.clip(capture_gain * capture_y * right_gate, -0.085, 0.085))
        base[5] -= float(np.clip(0.35 * capture_gain * capture_y * left_gate, -0.040, 0.040))
        base[11] -= float(np.clip(0.35 * capture_gain * capture_y * right_gate, -0.040, 0.040))
    if stance_contact_guard and ref_contact.shape[0] >= 2:
        for side, hip_i, knee_i, ankle_i, ref_c, actual_c in (
            ("left", 0, 3, 4, float(ref_contact[0]), left_contact),
            ("right", 6, 9, 10, float(ref_contact[1]), right_contact),
        ):
            if ref_c <= 0.5:
                continue
            foot_z = _humanoid_foot_rel_z(env, side)
            miss = (1.0 - actual_c) * float(np.clip((foot_z - 0.040) / 0.090, 0.0, 1.0))
            if miss <= 1e-6:
                continue
            # In this G1 XML, negative knee/ankle pitch lowers a high foot.
            # Apply only when the reference says stance but MuJoCo contact is
            # missing, so normal swing clearance is preserved.
            side_gain = float(stance_contact_right_gain) if side == "right" else 1.0
            base[knee_i] -= float(np.clip(0.135 * side_gain * miss, 0.0, 0.165))
            base[ankle_i] -= float(np.clip(0.055 * side_gain * miss, 0.0, 0.070))
            base[hip_i] -= float(np.clip(0.030 * side_gain * miss, 0.0, 0.040))
    if foot_width_guard_gain > 1e-9:
        target = max(float(foot_width_guard_target), 0.08)
        left_rel_y = _humanoid_foot_rel_y(env, "left")
        right_rel_y = _humanoid_foot_rel_y(env, "right")
        left_gate = float(np.clip(0.20 + left_swing + 0.35 * (1.0 - left_contact), 0.0, 1.0))
        right_gate = float(np.clip(0.20 + right_swing + 0.35 * (1.0 - right_contact), 0.0, 1.0))
        left_excess = max(left_rel_y - target, 0.0)
        right_excess = max(-target - right_rel_y, 0.0)
        base[1] -= float(np.clip(foot_width_guard_gain * left_excess * left_gate, 0.0, 0.070))
        base[7] += float(np.clip(foot_width_guard_gain * right_excess * right_gate, 0.0, 0.070))
    if swing_clearance_gain > 1e-9:
        z_target = max(float(swing_clearance_target), 0.04)
        x_target = float(swing_forward_target)
        for side, hip_i, knee_i, ankle_i, swing, contact, side_gain in (
            ("left", 0, 3, 4, left_swing, left_contact, 1.0),
            ("right", 6, 9, 10, right_swing, right_contact, swing_clearance_right_gain),
        ):
            if swing <= 1e-6:
                continue
            foot_z = _humanoid_foot_rel_z(env, side)
            rel_x = _humanoid_foot_rel_x(env, side)
            lift_need = float(np.clip((z_target - foot_z) / 0.10, 0.0, 1.0))
            forward_need = float(np.clip((x_target - rel_x) / 0.26, 0.0, 1.0))
            drag_gate = float(np.clip(0.35 + 0.65 * (1.0 - contact), 0.35, 1.0))
            need = swing_clearance_gain * side_gain * max(lift_need, 0.55 * forward_need) * drag_gate
            if need <= 1e-6:
                continue
            # In this G1 XML, negative hip pitch advances the foot; positive
            # knee/ankle pitch increases swing clearance. Keep the correction
            # below the residual budget so PPO can still refine it.
            base[hip_i] -= float(np.clip(0.035 * forward_need * need, 0.0, 0.040))
            base[knee_i] += float(np.clip((0.020 + 0.075 * lift_need) * need, 0.0, 0.095))
            base[ankle_i] += float(np.clip((0.010 + 0.035 * lift_need) * need, 0.0, 0.045))
    if support_lateral_gain > 1e-9:
        base_y = float(env.data.xpos[env._trunk_body_id][1])
        base_y_vel = float(np.clip(env.data.qvel[1], -2.0, 2.0))
        roll_like = float(np.clip(projected_gravity[1], -0.8, 0.8))
        raw_lateral_err = float(np.clip(-base_y - 0.12 * base_y_vel, -0.35, 0.35))
        err_abs = abs(raw_lateral_err)
        lateral_err = 0.0
        if err_abs > support_lateral_deadband:
            lateral_err = float(np.sign(raw_lateral_err) * (err_abs - support_lateral_deadband))
        activation = 1.0
        if support_lateral_activation_y > 0.0 or support_lateral_activation_roll > 0.0:
            y_need = (abs(base_y) - support_lateral_activation_y) / 0.25
            roll_need = (abs(roll_like) - support_lateral_activation_roll) / 0.25
            activation = float(np.clip(max(y_need, roll_need), 0.0, 1.0))
        corr = float(
            np.clip(
                activation * (support_lateral_sign * support_lateral_gain * lateral_err - 0.035 * roll_like),
                -0.105,
                0.105,
            )
        )
        left_support = max(left_contact, 1.0 - left_swing)
        right_support = max(right_contact, 1.0 - right_swing)
        base[1] += corr * (0.60 + 0.40 * left_support)
        base[7] += corr * (0.60 + 0.40 * right_support)
        base[5] -= 0.30 * corr * (0.70 + 0.30 * left_support)
        base[11] -= 0.30 * corr * (0.70 + 0.30 * right_support)
    if speed_brake_gain > 1e-9:
        time_gate = 1.0 if float(env.data.time) >= float(speed_brake_start_time) else 0.0
        brake = float(
            np.clip(
                time_gate * speed_brake_gain * max(vx - cmd_vx - float(speed_brake_threshold), 0.0),
                0.0,
                max(float(speed_brake_max), 0.0),
            )
        )
        if brake > 0.0:
            # Negative hip pitch moves the G1 foot forward in this XML, so use
            # positive hip pitch to reduce runaway forward placement. Keep the
            # correction small enough not to turn the gait into backstepping.
            base[0] += float(np.clip(0.20 * brake, 0.0, 0.09))
            base[6] += float(np.clip(0.20 * brake, 0.0, 0.09))
            base[14] += float(np.clip(-0.16 * brake, -0.08, 0.0))
    if pitch_catch_gain > 1e-9:
        pitch_like = float(np.clip(projected_gravity[0], -0.9, 0.9))
        pitch_need = float(np.clip((pitch_like - 0.34) / 0.42, 0.0, 1.0))
        speed_need = float(np.clip((vx - max(cmd_vx, 0.0) - 0.18) / 0.85, 0.0, 1.0))
        catch_gate = float(np.clip(pitch_catch_gain * max(pitch_need, speed_need), 0.0, 1.0))
        if catch_gate > 1e-6:
            for hip_i, knee_i, ankle_i, swing, contact, rel_x in (
                (0, 3, 4, left_swing, left_contact, left_rel),
                (6, 9, 10, right_swing, right_contact, right_rel),
            ):
                behind = float(np.clip((-0.18 - rel_x) / 0.42, 0.0, 1.0))
                foot_gate = float(np.clip(0.45 + 0.45 * swing + 0.20 * (1.0 - contact), 0.0, 1.0))
                need = catch_gate * max(behind, 0.25) * foot_gate
                # Negative hip pitch advances the foot in the local G1 XML.
                base[hip_i] -= float(np.clip(0.145 * need, 0.0, 0.145))
                base[knee_i] += float(np.clip(0.090 * need, 0.0, 0.090))
                base[ankle_i] += float(np.clip(0.040 * need, 0.0, 0.040))
            base[14] += float(np.clip(-0.11 * catch_gate - 0.07 * speed_need, -0.14, 0.0))
    gain = float(getattr(env, "gait_yaw_turn_scale", 0.0))
    if gain <= 0.0:
        gain = 0.5
    yaw_direction = float(getattr(env, "gait_yaw_turn_direction", 1.0))
    yaw_direction = 1.0 if yaw_direction >= 0.0 else -1.0
    base = _apply_g1_yaw_lateral_stabilizer(
        env,
        base,
        gain=gain,
        yaw_direction=yaw_direction,
        lateral_direction=yaw_direction,
        base_y_gain=base_y_gain,
        base_y_vel_gain=base_y_vel_gain,
        roll_gain=roll_gain,
    )
    if ground_guard:
        height = float(env.data.xpos[env._trunk_body_id][2])
        contact_count = left_contact + right_contact
        flight_need = float(np.clip((0.75 - contact_count) / 0.75, 0.0, 1.0))
        low_need = float(np.clip((0.52 - height) / 0.20, 0.0, 1.0))
        plant_need = max(flight_need, low_need)
        if plant_need > 1e-6:
            base[3] -= float(np.clip(0.105 * plant_need, 0.0, 0.105))
            base[9] -= float(np.clip(0.105 * plant_need, 0.0, 0.105))
            base[4] += float(np.clip(0.045 * plant_need, 0.0, 0.045))
            base[10] += float(np.clip(0.045 * plant_need, 0.0, 0.045))
    if flight_plant_guard:
        contact_count = left_contact + right_contact
        flight_need = float(np.clip((0.75 - contact_count) / 0.75, 0.0, 1.0))
        if flight_need > 1e-6:
            base[3] -= float(np.clip(0.120 * flight_need, 0.0, 0.120))
            base[9] -= float(np.clip(0.120 * flight_need, 0.0, 0.120))
            base[4] -= float(np.clip(0.055 * flight_need, 0.0, 0.055))
            base[10] -= float(np.clip(0.055 * flight_need, 0.0, 0.055))
            base[0] -= float(np.clip(0.025 * flight_need, 0.0, 0.025))
            base[6] -= float(np.clip(0.025 * flight_need, 0.0, 0.025))
    return np.clip(base, env._ctrl_low, env._ctrl_high).astype(np.float32)


def reference_commanded_action(env: Any) -> np.ndarray:
    get_reference_action = getattr(env, "get_reference_action", None)
    if not callable(get_reference_action):
        return home_action(env)
    nominal_speed = float(getattr(env, "gait_command_reference_speed", 0.30))
    cmd_vx = abs(float(getattr(env, "command", np.zeros(3, dtype=np.float32))[0]))
    speed_scale = float(np.clip(cmd_vx / max(nominal_speed, 1e-6), 0.35, 1.15))
    original_scale = float(getattr(env, "reference_time_scale", 1.0))
    try:
        env.reference_time_scale = original_scale * speed_scale
        return get_reference_action()
    finally:
        env.reference_time_scale = original_scale


def reference_stabilized_action(env: Any) -> np.ndarray:
    base = reference_motion_action(env)
    if env.model.nu < 29 or env.stage != "walk":
        return base

    # Startup transition: the source clip starts in the middle of walking, not
    # from quiet standing. For the first step, keep the nominal near home,
    # create an explicit left swing over a right stance, then hand over to the
    # cyclic reference. This prevents the initial "unsupported swing -> backstep"
    # failure seen when frame 0 is replayed immediately.
    startup_time = float(getattr(env, "nominal_ramp_time", 0.0))
    if float(env.data.time) < startup_time:
        home = env.home_joint_pos[: int(env.model.nu)].copy().astype(np.float32)
        phase = float(np.clip(float(env.data.time) / startup_time, 0.0, 1.0))
        alpha = phase * phase * (3.0 - 2.0 * phase)
        out = home + alpha * (base - home)
        swing = float(np.sin(np.pi * np.clip((phase - 0.12) / 0.76, 0.0, 1.0)))
        support = 1.0 - 0.35 * swing
        # Left swing: lift first, then let the reference take over.
        out[0] -= 0.030 * swing
        out[3] += 0.180 * swing
        out[4] += 0.075 * swing
        # Right stance: stay flatter and slightly extended so the heel does not
        # peel up before the first left-foot swing is established.
        out[9] -= 0.045 * support
        out[10] -= 0.025 * support
        out[14] -= 0.030 * alpha
        return np.clip(out, env._ctrl_low, env._ctrl_high).astype(np.float32)

    get_reference_contact = getattr(env, "get_reference_contact", None)
    ref_contact = (
        np.asarray(get_reference_contact(), dtype=np.float32)
        if callable(get_reference_contact)
        else np.zeros(2, dtype=np.float32)
    )
    left_swing = 0.0
    right_swing = 0.0
    if ref_contact.shape[0] >= 2 and float(np.max(ref_contact)) > 0.0:
        left_swing = float(np.clip(1.0 - ref_contact[0], 0.0, 1.0))
        right_swing = float(np.clip(1.0 - ref_contact[1], 0.0, 1.0))
    else:
        phase = env.get_gait_phase()
        left_swing = max(0.0, float(np.sin(phase)))
        right_swing = max(0.0, float(-np.sin(phase)))
    if left_swing > 0.5 and right_swing > 0.5:
        phase = env.get_gait_phase()
        if np.sin(phase) >= 0.0:
            right_swing = 0.0
        else:
            left_swing = 0.0

    # Retargeted replay moves forward, so keep it as the main prior. The
    # corrections below target the observed failure modes: swing toe drag and
    # late crouch/height collapse.
    base[3] += 0.04 * left_swing
    base[4] += 0.02 * left_swing
    base[0] += -0.010 * left_swing
    # The k12 replay's right foot is the weaker side, so give it a slightly
    # larger but still conservative swing-clearance bias.
    base[9] += 0.06 * right_swing
    base[10] += 0.03 * right_swing
    base[6] += -0.012 * right_swing

    trunk_height = 0.75
    try:
        trunk_height = float(env.data.xpos[env._trunk_body_id][2])
    except Exception:
        pass
    height_target = 0.62
    height_err = float(np.clip(height_target - trunk_height, 0.0, 0.18))
    if height_err > 0.0:
        # Positive G1 knee pitch increases crouch in the current XML. Reduce
        # knee flexion as the pelvis approaches the fall-height boundary.
        knee_unbend = 1.6 * height_err
        base[3] -= knee_unbend * (1.0 - 0.35 * left_swing)
        base[9] -= knee_unbend * (1.0 - 0.35 * right_swing)
        base[4] += 0.45 * knee_unbend
        base[10] += 0.45 * knee_unbend

    projected_gravity = env._get_projected_gravity()
    pitch_like = float(np.clip(projected_gravity[0], -0.5, 0.5))
    vx = float(env.data.qvel[0]) if getattr(env, "data", None) is not None else 0.0
    cmd_vx = float(env.command[0])
    overspeed = max(vx - cmd_vx - 0.12, 0.0)
    base[14] += -0.08 * pitch_like - 0.10 * overspeed

    return np.clip(base, env._ctrl_low, env._ctrl_high).astype(np.float32)


def reference_capture_action(
    env: Any,
    *,
    placement_gain: float = 0.42,
    lift_gain: float = 1.0,
    brake_gain: float = 0.12,
) -> np.ndarray:
    base = reference_motion_action(env)
    if env.model.nu < 29 or env.stage != "walk":
        return base

    get_reference_contact = getattr(env, "get_reference_contact", None)
    ref_contact = (
        np.asarray(get_reference_contact(), dtype=np.float32)
        if callable(get_reference_contact)
        else np.zeros(2, dtype=np.float32)
    )
    actual_contact = np.asarray(env._get_foot_contacts(), dtype=np.float32)[:2]
    phase = env.get_gait_phase()
    raw = float(np.sin(phase))
    if ref_contact.shape[0] >= 2 and float(np.max(ref_contact)) > 0.0:
        left_ref_swing = float(np.clip(1.0 - ref_contact[0], 0.0, 1.0))
        right_ref_swing = float(np.clip(1.0 - ref_contact[1], 0.0, 1.0))
    else:
        left_ref_swing = max(0.0, raw)
        right_ref_swing = max(0.0, -raw)

    left_swing = max(left_ref_swing, float(1.0 - actual_contact[0]) if actual_contact.shape[0] > 0 else 0.0)
    right_swing = max(right_ref_swing, float(1.0 - actual_contact[1]) if actual_contact.shape[0] > 1 else 0.0)
    if left_swing > 0.5 and right_swing > 0.5:
        if raw >= 0.0:
            right_swing = 0.0
        else:
            left_swing = 0.0

    vx = float(env.data.qvel[0]) if getattr(env, "data", None) is not None else 0.0
    cmd_vx = float(env.command[0])
    projected_gravity = env._get_projected_gravity()
    gravity_z = float(projected_gravity[2])
    pitch_like = float(np.clip(projected_gravity[0], -0.5, 0.5))
    overspeed = max(vx - cmd_vx - 0.10, 0.0)
    tilt_need = max(gravity_z + 0.80, 0.0)

    desired_x = float(np.clip(-0.08 + 0.18 * overspeed + 0.12 * tilt_need, -0.08, 0.08))
    clearance_target = 0.10

    for side, hip_i, knee_i, ankle_i, swing in (
        ("left", 0, 3, 4, left_swing),
        ("right", 6, 9, 10, right_swing),
    ):
        if swing <= 1e-6:
            continue
        rel_x = _humanoid_foot_rel_x(env, side)
        foot_z = _humanoid_foot_rel_z(env, side)
        placement_err = max(desired_x - rel_x, 0.0)
        lift_err = max(clearance_target - foot_z, 0.0)
        base[hip_i] += float(np.clip(-placement_gain * placement_err * swing, -0.08, 0.0))
        base[knee_i] += float(
            np.clip((0.04 + lift_gain * (0.18 * lift_err + 0.04 * placement_err)) * swing, 0.0, 0.10)
        )
        base[ankle_i] += float(np.clip((0.015 + lift_gain * 0.10 * lift_err) * swing, 0.0, 0.05))

    # Keep the stabilizer deliberately small; this prior should catch the feet,
    # not fight the whole retargeted motion.
    height = float(env.data.xpos[env._trunk_body_id][2])
    height_err = max(0.66 - height, 0.0)
    if height_err > 0.0:
        unbend = float(np.clip(1.2 * height_err, 0.0, 0.12))
        base[3] -= unbend * (1.0 - 0.5 * left_swing)
        base[9] -= unbend * (1.0 - 0.5 * right_swing)
    base[14] += float(np.clip(-0.10 * pitch_like - brake_gain * overspeed - 0.08 * tilt_need, -0.18, 0.08))

    return np.clip(base, env._ctrl_low, env._ctrl_high).astype(np.float32)


def reference_clearance_action(env: Any) -> np.ndarray:
    """Reference replay with explicit swing-foot clearance bias.

    This intentionally changes less than the capture controller: keep the
    retargeted k12 phase/speed, but do not let a reference swing foot drag near
    the ground or stay far behind the pelvis.
    """
    base = reference_motion_action(env)
    if env.model.nu < 29 or env.stage != "walk":
        return base

    get_reference_contact = getattr(env, "get_reference_contact", None)
    ref_contact = (
        np.asarray(get_reference_contact(), dtype=np.float32)
        if callable(get_reference_contact)
        else np.zeros(2, dtype=np.float32)
    )
    phase = env.get_gait_phase()
    raw = float(np.sin(phase))
    if ref_contact.shape[0] >= 2 and float(np.max(ref_contact)) > 0.0:
        left_swing = float(np.clip(1.0 - ref_contact[0], 0.0, 1.0))
        right_swing = float(np.clip(1.0 - ref_contact[1], 0.0, 1.0))
    else:
        left_swing = max(0.0, raw)
        right_swing = max(0.0, -raw)
    if left_swing > 0.5 and right_swing > 0.5:
        if raw >= 0.0:
            right_swing = 0.0
        else:
            left_swing = 0.0

    clearance_target = 0.118
    behind_start = -0.30
    for side, hip_i, knee_i, ankle_i, swing, side_gain in (
        ("left", 0, 3, 4, left_swing, 0.25),
        ("right", 6, 9, 10, right_swing, 1.0),
    ):
        if swing <= 1e-6:
            continue
        foot_z = _humanoid_foot_rel_z(env, side)
        rel_x = _humanoid_foot_rel_x(env, side)
        lift_need = float(np.clip((clearance_target - foot_z) / 0.10, 0.0, 1.0))
        behind_need = float(np.clip((behind_start - rel_x) / 0.22, 0.0, 1.0))
        need = side_gain * max(lift_need, 0.65 * behind_need)
        if need <= 1e-6:
            continue

        # For this XML, increasing knee pitch raises the swing foot. A small
        # negative hip-pitch term brings the foot forward without changing the
        # whole reference clock or stance foot.
        base[hip_i] += float(np.clip(-0.035 * side_gain * behind_need * swing, -0.045, 0.0))
        base[knee_i] += float(np.clip((0.020 + 0.080 * need) * swing, 0.0, 0.10))
        base[ankle_i] += float(np.clip((0.012 + 0.040 * need) * swing, 0.0, 0.052))

    height = float(env.data.xpos[env._trunk_body_id][2])
    if height < 0.62:
        unbend = float(np.clip(1.4 * (0.62 - height), 0.0, 0.10))
        base[3] -= unbend * (1.0 - 0.4 * left_swing)
        base[9] -= unbend * (1.0 - 0.4 * right_swing)

    projected_gravity = env._get_projected_gravity()
    pitch_like = float(np.clip(projected_gravity[0], -0.5, 0.5))
    vx = float(env.data.qvel[0]) if getattr(env, "data", None) is not None else 0.0
    cmd_vx = float(env.command[0])
    overspeed = max(vx - cmd_vx - 0.02, 0.0)
    base[14] += float(np.clip(-0.06 * pitch_like - 0.08 * overspeed, -0.12, 0.06))

    return np.clip(base, env._ctrl_low, env._ctrl_high).astype(np.float32)


def reference_footplace_action(env: Any) -> np.ndarray:
    """Reference replay with late, conservative foot-placement correction.

    This is intended for the replay-contact-relabelled reference. The nominal
    reference should own the gait; this layer only catches the repeated failure
    where a swing foot remains behind the pelvis or the trunk starts pitching.
    """
    base = reference_motion_action(env)
    if env.model.nu < 29 or env.stage != "walk":
        return base

    get_reference_contact = getattr(env, "get_reference_contact", None)
    ref_contact = (
        np.asarray(get_reference_contact(), dtype=np.float32)
        if callable(get_reference_contact)
        else np.zeros(2, dtype=np.float32)
    )
    actual_contact = np.asarray(env._get_foot_contacts(), dtype=np.float32)[:2]
    phase = env.get_gait_phase()
    raw = float(np.sin(phase))
    if ref_contact.shape[0] >= 2 and float(np.max(ref_contact)) > 0.0:
        left_swing = float(np.clip(1.0 - ref_contact[0], 0.0, 1.0))
        right_swing = float(np.clip(1.0 - ref_contact[1], 0.0, 1.0))
    else:
        left_swing = max(0.0, raw)
        right_swing = max(0.0, -raw)
    if left_swing > 0.5 and right_swing > 0.5:
        if raw >= 0.0:
            right_swing = 0.0
        else:
            left_swing = 0.0

    vx = float(env.data.qvel[0]) if getattr(env, "data", None) is not None else 0.0
    cmd_vx = float(env.command[0])
    projected_gravity = env._get_projected_gravity()
    pitch_like = float(np.clip(projected_gravity[0], -0.5, 0.5))
    gravity_z = float(projected_gravity[2])
    overspeed_threshold = max(cmd_vx + 0.05, 0.12)
    overspeed = max(vx - overspeed_threshold, 0.0)
    instability = float(np.clip(0.55 * overspeed + 0.7 * max(gravity_z + 0.82, 0.0), 0.0, 1.0))

    # Desired swing foot x is intentionally modest. Earlier capture variants
    # destabilized the replay by moving the foot too far forward too early.
    desired_x = float(np.clip(-0.12 + 0.10 * instability, -0.12, -0.02))
    clearance_target = 0.105
    for side, hip_i, knee_i, ankle_i, swing, side_gain in (
        ("left", 0, 3, 4, left_swing, 0.55),
        ("right", 6, 9, 10, right_swing, 1.0),
    ):
        if swing <= 1e-6:
            continue
        rel_x = _humanoid_foot_rel_x(env, side)
        foot_z = _humanoid_foot_rel_z(env, side)
        foot_contact = 0.0
        if side == "left" and actual_contact.shape[0] > 0:
            foot_contact = float(actual_contact[0])
        if side == "right" and actual_contact.shape[0] > 1:
            foot_contact = float(actual_contact[1])

        behind_need = float(np.clip((desired_x - rel_x) / 0.24, 0.0, 1.0))
        drag_need = float(np.clip((clearance_target - foot_z) / 0.08, 0.0, 1.0)) if foot_contact > 0.5 else 0.0
        need = side_gain * max(behind_need, 0.5 * drag_need)
        if need <= 1e-6:
            continue

        base[hip_i] += float(np.clip(-0.040 * need * swing, -0.050, 0.0))
        base[knee_i] += float(np.clip((0.020 + 0.060 * need) * swing, 0.0, 0.080))
        base[ankle_i] += float(np.clip((0.010 + 0.035 * need) * swing, 0.0, 0.045))

    height = float(env.data.xpos[env._trunk_body_id][2])
    if height < 0.63:
        unbend = float(np.clip(1.0 * (0.63 - height), 0.0, 0.08))
        base[3] -= unbend * (1.0 - 0.4 * left_swing)
        base[9] -= unbend * (1.0 - 0.4 * right_swing)

    base[14] += float(np.clip(-0.05 * pitch_like - 0.07 * overspeed, -0.10, 0.05))
    return np.clip(base, env._ctrl_low, env._ctrl_high).astype(np.float32)


def reference_latecatch_action(env: Any) -> np.ndarray:
    """Preserve reference replay, only catching clear late-fall foot lag.

    The V99 footplace controller proved that stronger placement can lift feet
    but also kills forward progression. This variant is intentionally sparse:
    if the replay is still healthy, it is exactly reference_motion. It adds a
    small catch-step bias only after measurable overspeed/tilt or when both
    feet trail the pelvis.
    """
    base = reference_motion_action(env)
    if env.model.nu < 29 or env.stage != "walk":
        return base

    get_reference_contact = getattr(env, "get_reference_contact", None)
    ref_contact = (
        np.asarray(get_reference_contact(), dtype=np.float32)
        if callable(get_reference_contact)
        else np.zeros(2, dtype=np.float32)
    )
    actual_contact = np.asarray(env._get_foot_contacts(), dtype=np.float32)[:2]
    phase = env.get_gait_phase()
    raw = float(np.sin(phase))
    if ref_contact.shape[0] >= 2 and float(np.max(ref_contact)) > 0.0:
        left_swing = float(np.clip(1.0 - ref_contact[0], 0.0, 1.0))
        right_swing = float(np.clip(1.0 - ref_contact[1], 0.0, 1.0))
    else:
        left_swing = max(0.0, raw)
        right_swing = max(0.0, -raw)
    if left_swing > 0.5 and right_swing > 0.5:
        if raw >= 0.0:
            right_swing = 0.0
        else:
            left_swing = 0.0

    vx = float(env.data.qvel[0]) if getattr(env, "data", None) is not None else 0.0
    cmd_vx = float(env.command[0])
    projected_gravity = env._get_projected_gravity()
    gravity_z = float(projected_gravity[2])
    pitch_like = float(np.clip(projected_gravity[0], -0.5, 0.5))
    overspeed = max(vx - cmd_vx - 0.18, 0.0)
    tilt_need = max(gravity_z + 0.80, 0.0)
    left_rel_x = _humanoid_foot_rel_x(env, "left")
    right_rel_x = _humanoid_foot_rel_x(env, "right")
    both_behind_need = max(min(left_rel_x, right_rel_x) * -1.0 - 0.20, 0.0)
    catch_need = float(np.clip(1.5 * overspeed + 1.2 * tilt_need + 1.8 * both_behind_need, 0.0, 1.0))
    if catch_need <= 1e-6:
        return base

    for side, hip_i, knee_i, ankle_i, swing, rel_x, contact, side_gain in (
        (
            "left",
            0,
            3,
            4,
            left_swing,
            left_rel_x,
            float(actual_contact[0]) if actual_contact.shape[0] > 0 else 0.0,
            0.65,
        ),
        (
            "right",
            6,
            9,
            10,
            right_swing,
            right_rel_x,
            float(actual_contact[1]) if actual_contact.shape[0] > 1 else 0.0,
            1.0,
        ),
    ):
        del side
        swing_gate = float(np.clip(swing + 0.20 * (1.0 - contact), 0.0, 1.0))
        lag = float(np.clip((-0.08 - rel_x) / 0.28, 0.0, 1.0))
        need = side_gain * catch_need * max(swing_gate, 0.35 * lag)
        if need <= 1e-6:
            continue
        base[hip_i] += float(np.clip(-0.030 * need, -0.035, 0.0))
        base[knee_i] += float(np.clip(0.040 * need, 0.0, 0.055))
        base[ankle_i] += float(np.clip(0.020 * need, 0.0, 0.030))

    # Small speed brake only after the same late-catch gate is active.
    base[14] += float(np.clip((-0.04 * pitch_like - 0.05 * overspeed) * catch_need, -0.055, 0.030))
    return np.clip(base, env._ctrl_low, env._ctrl_high).astype(np.float32)


def reference_speedgoverned_action(env: Any, *, hip_brake_sign: float = -1.0) -> np.ndarray:
    """Reference replay with a simple speed/height governor.

    High-blend replay produces real swing clearance, but V101 showed it also
    turns into a short ballistic fall. This controller keeps the same reference
    when the base speed is near command, then weakens the reference and unbends
    the stance legs as speed or height leaves the recoverable range.
    """
    if env.model.nu < 29 or env.stage != "walk":
        return reference_motion_action(env)

    vx = float(env.data.qvel[0]) if getattr(env, "data", None) is not None else 0.0
    cmd_vx = float(getattr(env, "command", np.zeros(3, dtype=np.float32))[0])
    overspeed = max(vx - cmd_vx - 0.06, 0.0)
    height = float(env.data.xpos[env._trunk_body_id][2])
    height_drop = max(0.68 - height, 0.0)
    scale = float(np.clip(1.0 - 2.35 * overspeed - 1.25 * height_drop, 0.22, 1.0))

    old_blend = float(getattr(env, "reference_action_blend", 1.0))
    old_time_scale = float(getattr(env, "reference_time_scale", 1.0))
    try:
        env.reference_action_blend = old_blend * scale
        env.reference_time_scale = old_time_scale * float(np.clip(0.58 + 0.42 * scale, 0.50, 1.0))
        base = reference_motion_action(env)
    finally:
        env.reference_action_blend = old_blend
        env.reference_time_scale = old_time_scale

    get_reference_contact = getattr(env, "get_reference_contact", None)
    ref_contact = (
        np.asarray(get_reference_contact(), dtype=np.float32)
        if callable(get_reference_contact)
        else np.zeros(2, dtype=np.float32)
    )
    phase = env.get_gait_phase()
    raw = float(np.sin(phase))
    if ref_contact.shape[0] >= 2 and float(np.max(ref_contact)) > 0.0:
        left_swing = float(np.clip(1.0 - ref_contact[0], 0.0, 1.0))
        right_swing = float(np.clip(1.0 - ref_contact[1], 0.0, 1.0))
    else:
        left_swing = max(0.0, raw)
        right_swing = max(0.0, -raw)
    if left_swing > 0.5 and right_swing > 0.5:
        if raw >= 0.0:
            right_swing = 0.0
        else:
            left_swing = 0.0

    # As the pelvis drops, reduce crouch mostly on stance legs. Keep some
    # swing knee so clearance is not completely erased.
    if height_drop > 0.0:
        unbend = float(np.clip(1.8 * height_drop, 0.0, 0.16))
        base[3] -= unbend * (1.0 - 0.45 * left_swing)
        base[9] -= unbend * (1.0 - 0.45 * right_swing)
        base[4] += 0.35 * unbend * (1.0 - 0.25 * left_swing)
        base[10] += 0.35 * unbend * (1.0 - 0.25 * right_swing)

    projected_gravity = env._get_projected_gravity()
    pitch_like = float(np.clip(projected_gravity[0], -0.5, 0.5))
    brake = float(np.clip(0.18 * overspeed + 0.10 * height_drop, 0.0, 0.20))
    base[0] += float(hip_brake_sign) * brake * (1.0 - 0.3 * left_swing)
    base[6] += float(hip_brake_sign) * brake * (1.0 - 0.3 * right_swing)
    base[14] += float(np.clip(-0.06 * pitch_like - 0.10 * overspeed, -0.16, 0.06))
    return np.clip(base, env._ctrl_low, env._ctrl_high).astype(np.float32)


def reference_landing_action(env: Any) -> np.ndarray:
    """High-blend reference replay with late swing-foot landing assistance.

    V103 showed useful foot clearance, but the swing leg stayed high while the
    pelvis accelerated forward and dropped. This controller keeps the high
    reference intact early in swing, then reduces knee/ankle flexion when a
    non-contact foot is already high and the body is overspeeding or dropping.
    """
    base = reference_motion_action(env)
    if env.model.nu < 29 or env.stage != "walk":
        return base

    contacts = np.asarray(env._get_foot_contacts(), dtype=np.float32)[:2]
    vx = float(env.data.qvel[0]) if getattr(env, "data", None) is not None else 0.0
    cmd_vx = float(getattr(env, "command", np.zeros(3, dtype=np.float32))[0])
    overspeed = max(vx - cmd_vx - 0.20, 0.0)
    height = float(env.data.xpos[env._trunk_body_id][2])
    height_drop = max(0.62 - height, 0.0)
    projected_gravity = env._get_projected_gravity()
    gravity_z = float(projected_gravity[2])
    tilt_need = max(gravity_z + 0.68, 0.0)
    pitch_like = float(np.clip(projected_gravity[0], -0.5, 0.5))

    for side, hip_i, knee_i, ankle_i, contact in (
        ("left", 0, 3, 4, float(contacts[0]) if contacts.shape[0] > 0 else 0.0),
        ("right", 6, 9, 10, float(contacts[1]) if contacts.shape[0] > 1 else 0.0),
    ):
        foot_z = _humanoid_foot_rel_z(env, side)
        rel_x = _humanoid_foot_rel_x(env, side)
        airborne = 1.0 if contact < 0.5 else 0.0
        high_foot = float(np.clip((foot_z - 0.16) / 0.16, 0.0, 1.0))
        late_need = float(np.clip(0.75 * overspeed + 1.4 * height_drop + 0.8 * tilt_need, 0.0, 1.0))
        landing_need = airborne * high_foot * late_need
        if landing_need > 1e-6:
            # Negative knee/ankle pitch lowers the foot in the current G1 XML.
            # Keep this moderate so early swing clearance is preserved.
            base[knee_i] -= float(np.clip(0.11 * landing_need, 0.0, 0.11))
            base[ankle_i] -= float(np.clip(0.045 * landing_need, 0.0, 0.045))
            # If the foot is still behind the pelvis, bias it slightly forward
            # while descending so landing can become a catch step.
            behind = float(np.clip((-0.04 - rel_x) / 0.35, 0.0, 1.0))
            base[hip_i] -= float(np.clip(0.035 * landing_need * behind, 0.0, 0.035))
        elif contact > 0.5 and height_drop > 0.0:
            # Support leg extension after touchdown.
            support = float(np.clip(1.5 * height_drop + 0.35 * overspeed, 0.0, 1.0))
            base[knee_i] -= float(np.clip(0.07 * support, 0.0, 0.07))
            base[ankle_i] += float(np.clip(0.025 * support, 0.0, 0.025))

    brake = float(np.clip(0.08 * overspeed + 0.10 * height_drop + 0.04 * tilt_need, 0.0, 0.10))
    if brake > 0.0:
        base[0] -= brake
        base[6] -= brake
    base[14] += float(np.clip(-0.08 * pitch_like - 0.04 * overspeed - 0.05 * tilt_need, -0.12, 0.06))
    return np.clip(base, env._ctrl_low, env._ctrl_high).astype(np.float32)


def reference_hipgoverned_action(env: Any) -> np.ndarray:
    """High-clearance reference with hip/torso speed governance only.

    Scaling the whole reference action reduced foot clearance. This variant
    keeps knee/ankle swing from the high-blend reference, but pulls hip pitch
    and waist pitch toward a lower-blend reference when forward speed grows.
    """
    high = reference_motion_action(env)
    if env.model.nu < 29 or env.stage != "walk":
        return high

    old_blend = float(getattr(env, "reference_action_blend", 1.0))
    try:
        env.reference_action_blend = max(0.42, 0.78 * old_blend)
        low = reference_motion_action(env)
    finally:
        env.reference_action_blend = old_blend

    vx = float(env.data.qvel[0]) if getattr(env, "data", None) is not None else 0.0
    cmd_vx = float(getattr(env, "command", np.zeros(3, dtype=np.float32))[0])
    projected_gravity = env._get_projected_gravity()
    gravity_z = float(projected_gravity[2])
    pitch_like = float(np.clip(projected_gravity[0], -0.5, 0.5))
    overspeed = max(vx - cmd_vx - 0.18, 0.0)
    height = float(env.data.xpos[env._trunk_body_id][2])
    height_drop = max(0.58 - height, 0.0)
    govern = float(np.clip(0.85 * overspeed + 1.20 * height_drop + 0.45 * max(gravity_z + 0.65, 0.0), 0.0, 1.0))
    if govern <= 1e-6:
        return high

    out = high.copy()
    # Hip pitch and waist pitch drive most of the sagittal acceleration. Blend
    # only those channels down so knee/ankle swing clearance survives.
    for idx in (0, 6, 14, 15, 22):
        if idx < out.shape[0] and idx < low.shape[0]:
            out[idx] = (1.0 - govern) * high[idx] + govern * low[idx]
    brake = float(np.clip(0.06 * overspeed + 0.08 * height_drop, 0.0, 0.08))
    out[0] -= brake
    out[6] -= brake
    out[14] += float(np.clip(-0.06 * pitch_like - 0.04 * overspeed, -0.10, 0.06))
    return np.clip(out, env._ctrl_low, env._ctrl_high).astype(np.float32)


def reference_support_stable_action(env: Any) -> np.ndarray:
    """Reference replay with stance support and late swing landing.

    This controller is for the reversed G1 reference probe: the replay creates
    forward propulsion, but the pelvis drops during single support. Keep the
    reference gait intact early, then extend the contact leg and bring an
    over-high swing foot down when height/tilt leave the recoverable band.
    """
    base = reference_motion_action(env)
    if env.model.nu < 29 or env.stage != "walk":
        return base

    contacts = np.asarray(env._get_foot_contacts(), dtype=np.float32)[:2]
    height = float(env.data.xpos[env._trunk_body_id][2])
    height_drop = max(0.72 - height, 0.0)
    projected_gravity = env._get_projected_gravity()
    gravity_z = float(projected_gravity[2])
    tilt_need = max(gravity_z + 0.78, 0.0)
    vx = float(env.data.qvel[0]) if getattr(env, "data", None) is not None else 0.0
    cmd_vx = float(getattr(env, "command", np.zeros(3, dtype=np.float32))[0])
    overspeed = max(vx - cmd_vx - 0.18, 0.0)
    support_need = float(np.clip(2.20 * height_drop + 1.10 * tilt_need + 0.35 * overspeed, 0.0, 1.0))
    if support_need <= 1e-6:
        return base

    for side, hip_i, knee_i, ankle_i, contact in (
        ("left", 0, 3, 4, float(contacts[0]) if contacts.shape[0] > 0 else 0.0),
        ("right", 6, 9, 10, float(contacts[1]) if contacts.shape[0] > 1 else 0.0),
    ):
        foot_z = _humanoid_foot_rel_z(env, side)
        rel_x = _humanoid_foot_rel_x(env, side)
        is_stance = contact > 0.5 or foot_z < 0.065
        if is_stance:
            stance = support_need * float(np.clip((0.75 - height) / 0.24, 0.0, 1.0))
            base[knee_i] -= float(np.clip(0.15 * stance, 0.0, 0.15))
            base[ankle_i] += float(np.clip(0.055 * stance, 0.0, 0.055))
            base[hip_i] += float(np.clip(0.030 * stance, 0.0, 0.030))
            continue

        high_swing = float(np.clip((foot_z - 0.080) / 0.12, 0.0, 1.0))
        late_swing = support_need * high_swing
        if late_swing <= 1e-6:
            continue
        base[knee_i] -= float(np.clip(0.28 * late_swing, 0.0, 0.28))
        base[ankle_i] -= float(np.clip(0.10 * late_swing, 0.0, 0.10))
        behind = float(np.clip((-0.04 - rel_x) / 0.28, 0.0, 1.0))
        base[hip_i] -= float(np.clip(0.055 * late_swing * behind, 0.0, 0.055))

    pitch_like = float(np.clip(projected_gravity[0], -0.5, 0.5))
    base[14] += float(np.clip(-0.08 * pitch_like - 0.04 * tilt_need, -0.10, 0.05))
    return np.clip(base, env._ctrl_low, env._ctrl_high).astype(np.float32)


def reference_pelvis_stable_action(env: Any) -> np.ndarray:
    """Reference replay with stance-foot pelvis and yaw stabilization.

    The BONES/GEAR reference already has the desired left/right swing phase.
    The observed failure is lateral/yaw drift while that phase is replayed.
    Keep sagittal reference joints intact and only bias roll/yaw/support joints
    from measured stance-foot and trunk state.
    """
    base = reference_motion_action(env)
    if env.model.nu < 29 or env.stage != "walk":
        return base
    ground_guard = False

    get_reference_contact = getattr(env, "get_reference_contact", None)
    ref_contact = (
        np.asarray(get_reference_contact(), dtype=np.float32)
        if callable(get_reference_contact)
        else np.zeros(2, dtype=np.float32)
    )
    contacts = np.asarray(env._get_foot_contacts(), dtype=np.float32)[:2]

    if ref_contact.shape[0] >= 2 and float(np.max(ref_contact)) > 0.0:
        left_ref_stance = float(np.clip(ref_contact[0], 0.0, 1.0))
        right_ref_stance = float(np.clip(ref_contact[1], 0.0, 1.0))
    else:
        left_ref_stance = float(contacts[0]) if contacts.shape[0] > 0 else 0.0
        right_ref_stance = float(contacts[1]) if contacts.shape[0] > 1 else 0.0

    left_contact = float(contacts[0]) if contacts.shape[0] > 0 else 0.0
    right_contact = float(contacts[1]) if contacts.shape[0] > 1 else 0.0
    left_swing = float(np.clip(1.0 - left_ref_stance, 0.0, 1.0))
    right_swing = float(np.clip(1.0 - right_ref_stance, 0.0, 1.0))
    if left_swing > 0.5 and right_swing > 0.5:
        raw = float(np.sin(env.get_gait_phase()))
        if raw >= 0.0:
            right_swing = 0.0
        else:
            left_swing = 0.0
    left_stance = float(np.clip(0.65 * left_ref_stance + 0.35 * left_contact, 0.0, 1.0))
    right_stance = float(np.clip(0.65 * right_ref_stance + 0.35 * right_contact, 0.0, 1.0))
    stance_sum = max(left_stance + right_stance, 1e-6)
    left_stance /= stance_sum
    right_stance /= stance_sum

    left_rel_y = _humanoid_foot_rel_y(env, "left")
    right_rel_y = _humanoid_foot_rel_y(env, "right")
    left_err = float(np.clip(left_rel_y - 0.105, -0.45, 0.45))
    right_err = float(np.clip(right_rel_y + 0.105, -0.45, 0.45))
    stance_err = left_stance * left_err + right_stance * right_err

    try:
        base_y = float(env.data.xpos[env._trunk_body_id][1])
    except Exception:
        base_y = 0.0
    projected_gravity = env._get_projected_gravity()
    roll_like = float(np.clip(projected_gravity[1], -0.6, 0.6))
    pitch_like = float(np.clip(projected_gravity[0], -0.6, 0.6))
    yaw = float(np.clip(_base_yaw_wxyz(env), -1.2, 1.2))
    yaw_rate = float(np.clip(env.data.qvel[5], -3.0, 3.0)) if getattr(env, "data", None) is not None else 0.0
    height = float(env.data.xpos[env._trunk_body_id][2])
    height_drop = max(0.70 - height, 0.0)
    vx = float(env.data.qvel[0]) if getattr(env, "data", None) is not None else 0.0
    cmd_vx = float(getattr(env, "command", np.zeros(3, dtype=np.float32))[0])
    overspeed = max(vx - cmd_vx - 0.08, 0.0)
    left_rel_x = _humanoid_foot_rel_x(env, "left")
    right_rel_x = _humanoid_foot_rel_x(env, "right")
    left_foot_z = _humanoid_foot_rel_z(env, "left")
    right_foot_z = _humanoid_foot_rel_z(env, "right")

    # Positive stance_err means the stance foot is too far to body-left
    # relative to the pelvis. Probe showed negative right hip-roll can recover
    # negative-y drift, so use the same sign on the active stance leg.
    lateral_cmd = float(
        np.clip(
            -0.24 * stance_err - 0.10 * base_y - 0.05 * roll_like,
            -0.095,
            0.095,
        )
    )
    base[1] += left_stance * lateral_cmd
    base[7] += right_stance * lateral_cmd
    base[5] -= left_stance * 0.30 * lateral_cmd
    base[11] -= right_stance * 0.30 * lateral_cmd

    yaw_cmd = float(np.clip(-0.12 * yaw - 0.035 * yaw_rate, -0.085, 0.085))
    base[2] += yaw_cmd
    base[8] += yaw_cmd
    base[12] += float(np.clip(-0.18 * yaw - 0.050 * yaw_rate, -0.14, 0.14))

    support = float(np.clip(2.0 * height_drop + 0.7 * overspeed + 0.6 * max(abs(stance_err) - 0.18, 0.0), 0.0, 1.0))
    if support > 0.0:
        base[3] -= float(np.clip(0.09 * support * left_stance, 0.0, 0.09))
        base[9] -= float(np.clip(0.09 * support * right_stance, 0.0, 0.09))
        base[4] += float(np.clip(0.035 * support * left_stance, 0.0, 0.035))
        base[10] += float(np.clip(0.035 * support * right_stance, 0.0, 0.035))
    if ground_guard:
        contact_count = left_contact + right_contact
        flight_need = float(np.clip((0.75 - contact_count) / 0.75, 0.0, 1.0))
        low_need = float(np.clip((0.52 - height) / 0.20, 0.0, 1.0))
        plant_need = max(flight_need, low_need)
        if plant_need > 1e-6:
            # Emergency plant: when lateral roll lifts both feet, extend both
            # legs rather than asking the policy to find contact from flight.
            base[3] -= float(np.clip(0.105 * plant_need, 0.0, 0.105))
            base[9] -= float(np.clip(0.105 * plant_need, 0.0, 0.105))
            base[4] += float(np.clip(0.045 * plant_need, 0.0, 0.045))
            base[10] += float(np.clip(0.045 * plant_need, 0.0, 0.045))

    # Drag guard: if the reference says a foot is swinging but the MuJoCo foot
    # is still low or in contact, lift it immediately. This targets the
    # observed failure where the robot rubs the swing foot on the ground, fails
    # to transfer the pelvis forward, then backsteps.
    clearance_target = 0.100
    for knee_i, ankle_i, swing, contact, foot_z in (
        (3, 4, left_swing, left_contact, left_foot_z),
        (9, 10, right_swing, right_contact, right_foot_z),
    ):
        lift_err = max(clearance_target - foot_z, 0.0)
        drag_need = float(np.clip(0.90 * swing * contact + 1.0 * swing * lift_err, 0.0, 1.0))
        if drag_need > 1e-6:
            base[knee_i] += float(np.clip(0.070 * drag_need, 0.0, 0.070))
            base[ankle_i] += float(np.clip(0.030 * drag_need, 0.0, 0.030))

    # Sagittal capture: V423 fixed lateral support, but then the pelvis ran
    # ahead of both feet. When either swing foot trails the pelvis, preserve
    # the reference phase while biasing that leg into a catch step and braking
    # hip/waist drive. This is deliberately gated by reference swing so it does
    # not collapse back into a two-foot hop.
    both_behind = max(-0.18 - min(left_rel_x, right_rel_x), 0.0)
    tilt_need = max(float(projected_gravity[2]) + 0.78, 0.0)
    capture = float(np.clip(1.75 * both_behind + 0.75 * overspeed + 0.80 * tilt_need, 0.0, 1.0))
    for side, hip_i, knee_i, ankle_i, swing, stance, rel_x in (
        ("left", 0, 3, 4, left_swing, left_stance, left_rel_x),
        ("right", 6, 9, 10, right_swing, right_stance, right_rel_x),
    ):
        del side
        lag = float(np.clip((-0.06 - rel_x) / 0.42, 0.0, 1.0))
        step_need = capture * max(0.75 * swing, 0.30 * lag)
        if step_need > 1e-6:
            base[knee_i] += float(np.clip(0.145 * step_need, 0.0, 0.145))
            base[ankle_i] += float(np.clip(0.070 * step_need, 0.0, 0.070))
        brake = capture * stance * float(np.clip((vx - cmd_vx) / 0.35, 0.0, 1.0))
        if brake > 1e-6:
            base[knee_i] += float(np.clip(0.075 * brake, 0.0, 0.075))
            base[ankle_i] += float(np.clip(0.035 * brake, 0.0, 0.035))

    base[14] += float(
        np.clip(-0.07 * pitch_like - 0.13 * overspeed - 0.04 * height_drop - 0.070 * capture, -0.20, 0.05)
    )
    return np.clip(base, env._ctrl_low, env._ctrl_high).astype(np.float32)


def reference_phase_footstep_action(env: Any) -> np.ndarray:
    return _footstep_cartesian_action(
        env,
        capture=True,
        aggressive=False,
        swingcatch=True,
        highclearance=True,
        lipm_capture=True,
        reference_phase=True,
    )


def humanoid_walk_action(env: Any) -> np.ndarray:
    base = home_action(env)
    if env.model.nu < 29 or env.stage != "walk":
        return base

    phase = env.get_gait_phase()
    reference_speed = float(getattr(env, "gait_command_reference_speed", 0.35))
    max_scale = float(getattr(env, "gait_command_max_scale", 1.0))
    vx_scale = float(np.clip(abs(env.command[0]) / max(reference_speed, 1e-6), 0.0, max_scale))
    amp_scale = env.gait_command_scale * vx_scale
    if amp_scale <= 1e-6:
        return base

    # G1 actuator order follows the XML: left leg 0:6, right leg 6:12,
    # waist 12:15, left arm 15:22, right arm 22:29.
    left_swing = np.sin(phase)
    right_swing = np.sin(phase + np.pi)
    left_lift = max(0.0, left_swing)
    right_lift = max(0.0, right_swing)
    yaw_norm = float(np.clip(env.command[2] / 0.5, -1.0, 1.0))

    hip_pitch_amp = float(getattr(env, "gait_thigh_amp", 0.25))
    knee_amp = float(getattr(env, "gait_calf_amp", 0.35))
    ankle_amp = 0.45 * knee_amp
    hip_roll_amp = 0.045
    arm_pitch_amp = 0.45 * hip_pitch_amp
    yaw_amp = float(getattr(env, "gait_yaw_hip_amp", 0.0))

    # Left leg.
    base[0] += amp_scale * hip_pitch_amp * left_swing
    base[1] += hip_roll_amp * amp_scale * np.sin(phase + np.pi / 2.0)
    base[2] += yaw_amp * yaw_norm
    base[3] += amp_scale * knee_amp * left_lift
    base[4] += -amp_scale * ankle_amp * left_lift

    # Right leg.
    base[6] += amp_scale * hip_pitch_amp * right_swing
    base[7] += -hip_roll_amp * amp_scale * np.sin(phase + np.pi / 2.0)
    base[8] += -yaw_amp * yaw_norm
    base[9] += amp_scale * knee_amp * right_lift
    base[10] += -amp_scale * ankle_amp * right_lift

    # Torso and arms provide a weak counter-swing. Keep this conservative:
    # G1 has many upper-body DoFs and large arm motion destabilizes early PPO.
    base[12] += 0.05 * yaw_norm
    base[15] += -amp_scale * arm_pitch_amp * left_swing
    base[22] += -amp_scale * arm_pitch_amp * right_swing

    return np.clip(base, env._ctrl_low, env._ctrl_high).astype(np.float32)


def footstep_ik_action(env: Any) -> np.ndarray:
    """CPU-friendly humanoid support-walk prior.

    The GO1 controller works because the nominal gait already contains a
    reasonable stance/swing structure. This prior follows the same idea for G1:
    it starts from a crouched support posture, alternates swing legs, adds a
    small stance push-off, and uses light torso/velocity feedback. PPO should
    learn residual corrections instead of discovering the whole biped gait.
    """
    base = home_action(env)
    if env.model.nu < 29 or env.stage != "walk":
        return base

    phase = env.get_gait_phase()
    reference_speed = float(getattr(env, "gait_command_reference_speed", 0.18))
    max_scale = float(getattr(env, "gait_command_max_scale", 1.0))
    cmd_vx = float(env.command[0])
    vx_scale = float(np.clip(abs(cmd_vx) / max(reference_speed, 1e-6), 0.0, max_scale))
    amp_scale = float(getattr(env, "gait_command_scale", 1.0)) * vx_scale
    if amp_scale <= 1e-6:
        return base

    # Crouched support posture taken from the useful range of the retargeted
    # walking clip, not from the unstable straight-leg home pose.
    base_hip = -0.12
    base_knee = 0.26
    base_ankle = -0.13
    base_waist_pitch = 0.10
    base[0] += base_hip
    base[3] += base_knee
    base[4] += base_ankle
    base[6] += base_hip
    base[9] += base_knee
    base[10] += base_ankle
    base[14] += base_waist_pitch

    step_amp_base = float(getattr(env, "gait_thigh_amp", 0.20)) * amp_scale
    lift_amp = float(getattr(env, "gait_calf_amp", 0.28)) * amp_scale
    lateral_base = float(getattr(env, "gait_yaw_turn_scale", 0.0))
    if abs(lateral_base) <= 1e-9:
        lateral_base = 0.045
    lateral_amp = lateral_base * amp_scale
    ankle_pitch_amp = 0.55 * lift_amp

    # Smooth half-cycle swing profile for each leg. sin(phase)>0 means left
    # swing / right stance; sin(phase)<0 means right swing / left stance.
    raw = np.sin(phase)
    left_swing = max(0.0, raw)
    right_swing = max(0.0, -raw)
    left_stance = 1.0 - left_swing
    right_stance = 1.0 - right_swing
    double_support = 0.20 + 0.80 * abs(np.cos(phase))

    # Velocity feedback is deliberately small: it biases the prior to move,
    # while residual PPO still owns most of the correction budget.
    vx = float(env.data.qvel[0]) if getattr(env, "data", None) is not None else 0.0
    vx_err = float(np.clip(float(env.command[0]) - vx, -0.25, 0.25))
    vel_bias = 0.35 * vx_err

    projected_gravity = env._get_projected_gravity()
    pitch_like = float(np.clip(projected_gravity[0], -0.5, 0.5))
    roll_like = float(np.clip(projected_gravity[1], -0.5, 0.5))
    gravity_z = float(projected_gravity[2])

    overspeed = max(vx - float(env.command[0]) - 0.08, 0.0)
    speed_gate = float(np.clip(1.0 - 2.8 * overspeed, 0.20, 1.0))
    # When the trunk is already near the fall-tilt boundary, stop asking the
    # nominal gait to drive harder. Keep swing clearance, but reduce sagittal
    # stepping and push-off so the residual policy has a recoverable state.
    pitch_gate = float(np.clip((-0.50 - gravity_z) / 0.30, 0.20, 1.0))
    drive_gate = min(speed_gate, pitch_gate)
    step_amp = step_amp_base * drive_gate
    push_amp = float(getattr(env, "gait_yaw_forward_compensation", 0.25)) * step_amp * drive_gate
    arm_amp = 0.35 * step_amp
    brake_bias = 0.18 * overspeed + 0.10 * max(gravity_z + 0.70, 0.0)

    # Swing leg advances and clears. Stance leg extends behind the body and
    # plantar-flexes slightly for push-off. These are joint-space IK-like
    # targets, not a full Cartesian solver.
    sagittal_sign = float(getattr(env, "gait_yaw_turn_direction", 1.0))
    sagittal_sign = 1.0 if sagittal_sign >= 0.0 else -1.0
    # Positive hip-pitch commands in this XML did not create useful forward
    # progression in nominal sweeps. Keep the direction explicit so it can be
    # swept with --gait-yaw-turn-direction.
    forward_sign = -sagittal_sign
    left_hip_pitch = forward_sign * (
        step_amp * left_swing - (0.35 * step_amp + push_amp) * left_stance
    )
    right_hip_pitch = forward_sign * (
        step_amp * right_swing - (0.35 * step_amp + push_amp) * right_stance
    )
    left_knee = lift_amp * left_swing - 0.08 * lift_amp * left_stance
    right_knee = lift_amp * right_swing - 0.08 * lift_amp * right_stance
    left_ankle = forward_sign * (
        -ankle_pitch_amp * left_swing + 0.20 * ankle_pitch_amp * left_stance
    )
    right_ankle = forward_sign * (
        -ankle_pitch_amp * right_swing + 0.20 * ankle_pitch_amp * right_stance
    )

    # Support-side roll shifts the pelvis toward the stance foot. The sign is
    # matched to the asymmetric hip-roll actuator ranges.
    lateral_shift = np.sin(phase + np.pi / 2.0)
    yaw_norm = float(np.clip(env.command[2] / 0.5, -1.0, 1.0))
    yaw_amp = float(getattr(env, "gait_yaw_hip_amp", 0.0))

    # Left leg: hip pitch, roll, yaw, knee, ankle pitch, ankle roll.
    base[0] += left_hip_pitch - 0.50 * vel_bias - brake_bias - 0.05 * pitch_like
    base[1] += lateral_amp * lateral_shift - 0.04 * roll_like
    base[2] += yaw_amp * yaw_norm
    base[3] += left_knee
    base[4] += left_ankle
    base[5] += -0.35 * lateral_amp * lateral_shift

    # Right leg.
    base[6] += right_hip_pitch - 0.50 * vel_bias - brake_bias - 0.05 * pitch_like
    base[7] += lateral_amp * lateral_shift - 0.04 * roll_like
    base[8] += -yaw_amp * yaw_norm
    base[9] += right_knee
    base[10] += right_ankle
    base[11] += -0.35 * lateral_amp * lateral_shift

    # Keep torso slightly forward, but counteract excessive pitch drift.
    base[14] += 0.02 * amp_scale - 0.05 * vx_err - 0.12 * pitch_like - 0.12 * overspeed
    base[15] += -arm_amp * raw * double_support
    base[22] += arm_amp * raw * double_support

    return np.clip(base, env._ctrl_low, env._ctrl_high).astype(np.float32)


def footstep_ik_capture_action(env: Any) -> np.ndarray:
    """Footstep IK prior with measured capture/landing correction.

    This keeps the airborne footstep_ik structure used by the V74 seed, but
    adds a small measured correction for the observed failure: before falling,
    the feet trail too far behind the pelvis while trunk pitch/velocity grows.
    """
    base = footstep_ik_action(env)
    if env.model.nu < 29 or env.stage != "walk":
        return base

    phase = env.get_gait_phase()
    raw = float(np.sin(phase))
    left_swing = float(np.clip((raw - 0.05) / 0.75, 0.0, 1.0))
    right_swing = float(np.clip((-raw - 0.05) / 0.75, 0.0, 1.0))
    left_swing = float(0.5 - 0.5 * np.cos(np.pi * left_swing))
    right_swing = float(0.5 - 0.5 * np.cos(np.pi * right_swing))
    left_stance = 1.0 - left_swing
    right_stance = 1.0 - right_swing

    vx = float(env.data.qvel[0]) if getattr(env, "data", None) is not None else 0.0
    cmd_vx = float(env.command[0])
    projected_gravity = env._get_projected_gravity()
    gravity_z = float(projected_gravity[2])
    pitch_like = float(np.clip(projected_gravity[0], -0.6, 0.6))

    overspeed = max(vx - cmd_vx - 0.10, 0.0)
    pitch_need = max(gravity_z + 0.82, 0.0)
    capture_need = float(np.clip(1.4 * overspeed + 1.2 * pitch_need, 0.0, 1.0))

    left_rel = _humanoid_foot_rel_x(env, "left")
    right_rel = _humanoid_foot_rel_x(env, "right")
    target_rel = -0.18
    left_behind = max(target_rel - left_rel, 0.0)
    right_behind = max(target_rel - right_rel, 0.0)

    # Negative hip pitch moves the G1 foot forward in the measured kinematics.
    # Apply most of the correction to the swing foot, with a tiny all-feet
    # emergency component when both feet trail behind the pelvis.
    both_behind = float(left_rel < -0.28 and right_rel < -0.28)
    left_weight = left_swing + 0.08 * both_behind * left_stance
    right_weight = right_swing + 0.08 * both_behind * right_stance
    left_catch = float(np.clip(1.15 * left_behind * (0.35 + capture_need) * left_weight, 0.0, 0.26))
    right_catch = float(np.clip(1.15 * right_behind * (0.35 + capture_need) * right_weight, 0.0, 0.26))

    base[0] -= left_catch
    base[6] -= right_catch
    base[3] += float(np.clip(0.55 * left_catch + 0.04 * capture_need * left_swing, 0.0, 0.16))
    base[9] += float(np.clip(0.55 * right_catch + 0.04 * capture_need * right_swing, 0.0, 0.16))
    base[4] += float(np.clip(0.18 * left_catch, 0.0, 0.05))
    base[10] += float(np.clip(0.18 * right_catch, 0.0, 0.05))

    # Do not keep driving the trunk forward during capture.
    base[14] += float(np.clip(-0.08 * overspeed - 0.06 * pitch_need - 0.03 * pitch_like, -0.12, 0.05))

    return np.clip(base, env._ctrl_low, env._ctrl_high).astype(np.float32)


def footstep_ik_landing_action(env: Any) -> np.ndarray:
    """Footstep IK prior with early landing-target regulation.

    V74 already creates a real airborne step, but the swing foot trails too far
    behind the pelvis before contact. This controller starts from footstep_ik
    and regulates the swing foot toward a pelvis-relative landing window during
    the whole swing, rather than waiting for late capture.
    """
    base = footstep_ik_action(env)
    if env.model.nu < 29 or env.stage != "walk":
        return base

    phase = env.get_gait_phase()
    raw = float(np.sin(phase))
    left_phase = float(np.clip((raw - 0.03) / 0.82, 0.0, 1.0))
    right_phase = float(np.clip((-raw - 0.03) / 0.82, 0.0, 1.0))
    left_swing = float(0.5 - 0.5 * np.cos(np.pi * left_phase))
    right_swing = float(0.5 - 0.5 * np.cos(np.pi * right_phase))
    left_stance = 1.0 - left_swing
    right_stance = 1.0 - right_swing

    vx = float(env.data.qvel[0]) if getattr(env, "data", None) is not None else 0.0
    cmd_vx = float(env.command[0])
    projected_gravity = env._get_projected_gravity()
    pitch_like = float(np.clip(projected_gravity[0], -0.6, 0.6))
    gravity_z = float(projected_gravity[2])

    overspeed = max(vx - cmd_vx - 0.08, 0.0)
    tilt_need = max(gravity_z + 0.86, 0.0)
    capture_need = float(np.clip(0.7 + 1.0 * overspeed + 1.2 * tilt_need, 0.7, 1.6))

    left_rel = _humanoid_foot_rel_x(env, "left")
    right_rel = _humanoid_foot_rel_x(env, "right")
    # Successful early frames keep foot_rel_x around -0.05 to -0.20. Once a
    # foot drifts past -0.30, the pelvis is already running away.
    swing_target = -0.12 - 0.05 * min(max(vx, 0.0), 0.8)
    stance_floor = -0.28

    left_swing_err = max(swing_target - left_rel, 0.0)
    right_swing_err = max(swing_target - right_rel, 0.0)
    left_stance_err = max(stance_floor - left_rel, 0.0)
    right_stance_err = max(stance_floor - right_rel, 0.0)

    left_weight = left_swing + 0.18 * left_stance * float(right_swing > 0.55)
    right_weight = right_swing + 0.18 * right_stance * float(left_swing > 0.55)
    left_catch = float(np.clip((1.55 * left_swing_err * left_weight + 0.45 * left_stance_err * left_stance) * capture_need, 0.0, 0.34))
    right_catch = float(np.clip((1.55 * right_swing_err * right_weight + 0.45 * right_stance_err * right_stance) * capture_need, 0.0, 0.34))

    # Negative hip pitch moves the foot forward in this XML. Knee/ankle are
    # coupled to avoid toe drag while changing landing position.
    base[0] -= left_catch
    base[6] -= right_catch
    base[3] += float(np.clip(0.42 * left_catch + 0.035 * left_swing, 0.0, 0.18))
    base[9] += float(np.clip(0.42 * right_catch + 0.035 * right_swing, 0.0, 0.18))
    base[4] += float(np.clip(0.12 * left_catch + 0.010 * left_swing, 0.0, 0.055))
    base[10] += float(np.clip(0.12 * right_catch + 0.010 * right_swing, 0.0, 0.055))

    # Earlier trunk braking; late velocity runaway is the consistent terminal
    # mode of the airborne branch.
    base[14] += float(np.clip(-0.12 * overspeed - 0.08 * tilt_need - 0.035 * pitch_like, -0.16, 0.05))

    return np.clip(base, env._ctrl_low, env._ctrl_high).astype(np.float32)


def footstep_ik_landing_governed_action(env: Any, brake_sign: float = 1.0) -> np.ndarray:
    """Landing controller with earlier catch-step and speed governance.

    The forward nominal candidate can lift both feet, but it fails when the
    pelvis runs ahead of the feet. This layer keeps that high-step gait while
    enforcing a stricter landing window before the trunk reaches high speed.
    """
    base = footstep_ik_landing_action(env)
    if env.model.nu < 29 or env.stage != "walk":
        return base

    phase = env.get_gait_phase()
    raw = float(np.sin(phase))
    left_phase = float(np.clip((raw - 0.02) / 0.80, 0.0, 1.0))
    right_phase = float(np.clip((-raw - 0.02) / 0.80, 0.0, 1.0))
    left_swing = float(0.5 - 0.5 * np.cos(np.pi * left_phase))
    right_swing = float(0.5 - 0.5 * np.cos(np.pi * right_phase))
    left_stance = 1.0 - left_swing
    right_stance = 1.0 - right_swing

    vx = float(env.data.qvel[0]) if getattr(env, "data", None) is not None else 0.0
    cmd_vx = float(env.command[0])
    projected_gravity = env._get_projected_gravity()
    pitch_like = float(np.clip(projected_gravity[0], -0.7, 0.7))
    roll_like = float(np.clip(projected_gravity[1], -0.7, 0.7))
    gravity_z = float(projected_gravity[2])
    height = float(env.data.xpos[env._trunk_body_id][2])

    overspeed = max(vx - cmd_vx - 0.18, 0.0)
    tilt_need = max(gravity_z + 0.78, 0.0)
    height_drop = max(0.70 - height, 0.0)
    emergency = float(np.clip(1.40 * overspeed + 1.15 * tilt_need + 1.30 * height_drop, 0.0, 1.8))

    contacts = np.asarray(env._get_foot_contacts(), dtype=np.float32)[:2]
    target_rel = float(np.clip(-0.08 + 0.20 * min(max(overspeed, 0.0), 0.8), -0.10, 0.08))
    stance_floor = -0.28

    for side, hip_i, roll_i, knee_i, ankle_i, ankle_roll_i, swing, stance, contact in (
        (
            "left",
            0,
            1,
            3,
            4,
            5,
            left_swing,
            left_stance,
            float(contacts[0]) if contacts.shape[0] > 0 else 0.0,
        ),
        (
            "right",
            6,
            7,
            9,
            10,
            11,
            right_swing,
            right_stance,
            float(contacts[1]) if contacts.shape[0] > 1 else 0.0,
        ),
    ):
        rel_x = _humanoid_foot_rel_x(env, side)
        foot_z = _humanoid_foot_rel_z(env, side)
        swing_err = max(target_rel - rel_x, 0.0)
        stance_err = max(stance_floor - rel_x, 0.0)
        catch_gate = swing + 0.28 * stance * float(emergency > 0.25)
        catch = float(np.clip((1.65 * swing_err * catch_gate + 0.55 * stance_err * stance) * (0.75 + 0.55 * emergency), 0.0, 0.46))
        far_emergency = max(-0.34 - rel_x, 0.0)
        catch += float(np.clip(1.30 * far_emergency * (0.35 + emergency), 0.0, 0.22))
        if catch > 1e-6:
            base[hip_i] -= catch
            base[knee_i] += float(np.clip(0.38 * catch + 0.025 * swing, 0.0, 0.24))
            base[ankle_i] += float(np.clip(0.12 * catch + 0.010 * swing, 0.0, 0.085))

        airborne = 1.0 if contact < 0.5 else 0.0
        high_foot = float(np.clip((foot_z - 0.075) / 0.095, 0.0, 1.0))
        landing = airborne * high_foot * float(np.clip(1.25 * emergency, 0.0, 1.0))
        if landing > 1e-6:
            base[knee_i] -= float(np.clip(0.24 * landing, 0.0, 0.24))
            base[ankle_i] -= float(np.clip(0.10 * landing, 0.0, 0.10))
            base[hip_i] -= float(np.clip(0.050 * landing * max(swing, 0.35), 0.0, 0.050))

        # Small frontal-plane support correction. It should not dominate the
        # gait; it only counters the late drift that accompanies fall_tilt.
        lateral = float(np.clip(-0.06 * roll_like * (0.4 + stance), -0.045, 0.045))
        base[roll_i] += lateral
        base[ankle_roll_i] -= 0.45 * lateral

    brake = float(np.clip(0.28 * overspeed + 0.11 * tilt_need + 0.13 * height_drop, 0.0, 0.28))
    if brake > 0.0:
        # Reduce further backward push-off and keep the torso from chasing the
        # runaway velocity. Sign follows the existing landing controller.
        base[0] += float(brake_sign) * 0.45 * brake
        base[6] += float(brake_sign) * 0.45 * brake
        base[14] += float(np.clip(-0.18 * brake - 0.06 * pitch_like, -0.20, 0.04))

    return np.clip(base, env._ctrl_low, env._ctrl_high).astype(np.float32)


def footstep_ik_forward_action(env: Any) -> np.ndarray:
    """G1 sagittal gait with measured XML-forward sign.

    Kinematic and short dynamic probes of the local G1 XML show negative
    hip/knee/ankle pitch moves the body toward world +x. Earlier variants used
    positive knee flexion as "lift", which raised the foot but consistently
    drove the pelvis backward. This controller keeps the sign measured from the
    XML instead of reusing quadruped-style conventions.
    """
    base = home_action(env)
    if env.model.nu < 29 or env.stage != "walk":
        return base

    phase = env.get_gait_phase()
    raw = float(np.sin(phase))
    left_phase = float(np.clip((raw - 0.04) / 0.78, 0.0, 1.0))
    right_phase = float(np.clip((-raw - 0.04) / 0.78, 0.0, 1.0))
    left_swing = float(0.5 - 0.5 * np.cos(np.pi * left_phase))
    right_swing = float(0.5 - 0.5 * np.cos(np.pi * right_phase))
    left_stance = 1.0 - left_swing
    right_stance = 1.0 - right_swing

    reference_speed = float(getattr(env, "gait_command_reference_speed", 0.08))
    max_scale = float(getattr(env, "gait_command_max_scale", 1.0))
    cmd_vx = float(env.command[0])
    amp_scale = float(getattr(env, "gait_command_scale", 1.0)) * float(
        np.clip(abs(cmd_vx) / max(reference_speed, 1e-6), 0.0, max_scale)
    )
    if amp_scale <= 1e-6:
        return base

    vx = float(env.data.qvel[0]) if getattr(env, "data", None) is not None else 0.0
    projected_gravity = env._get_projected_gravity()
    pitch_like = float(np.clip(projected_gravity[0], -0.6, 0.6))
    roll_like = float(np.clip(projected_gravity[1], -0.6, 0.6))
    gravity_z = float(projected_gravity[2])
    yaw_abs = abs(float(_base_yaw_wxyz(env)))
    height = float(env.data.xpos[env._trunk_body_id][2])

    step_amp = float(getattr(env, "gait_thigh_amp", 0.20)) * amp_scale
    lift_amp = float(getattr(env, "gait_calf_amp", 0.34)) * amp_scale
    raw_push_amp = float(getattr(env, "gait_yaw_forward_compensation", 0.10)) * amp_scale
    lateral_amp = 0.040 * amp_scale

    overspeed = max(vx - cmd_vx - 0.02, 0.0)
    speed_emergency = max(vx - 0.35, 0.0)
    backward = max(cmd_vx - vx, 0.0)
    tilt_need = max(gravity_z + 0.82, 0.0)
    early_tilt_need = max(gravity_z + 0.94, 0.0)
    yaw_need = max(yaw_abs - 0.25, 0.0)
    height_drop = max(0.68 - height, 0.0)
    drive_gate = float(
        np.clip(
            1.0
            - 4.8 * overspeed
            - 2.2 * early_tilt_need
            - 1.7 * yaw_need
            - 2.0 * speed_emergency
            - 1.2 * height_drop,
            0.10,
            1.0,
        )
    )
    push_gate = float(
        np.clip(
            1.0
            - 7.0 * overspeed
            - 3.2 * early_tilt_need
            - 2.4 * yaw_need
            - 3.0 * speed_emergency
            - 1.8 * height_drop,
            0.0,
            1.0,
        )
    )
    push_amp = raw_push_amp * push_gate

    # Forward-biased support posture from the local probe. This is less of a
    # visual crouch than the older +knee pose, but it avoids the backward impulse
    # that dominated the videos.
    base[0] += -0.04
    base[3] += 0.08
    base[4] += -0.06
    base[6] += -0.04
    base[9] += 0.08
    base[10] += -0.06
    base[14] += 0.04

    # Negative pitch commands move the swing foot forward and slightly up in
    # this model. Keep stance nearly neutral so the foot does not scrape forward
    # under load.
    left_hip = -drive_gate * step_amp * left_swing - 0.15 * push_amp * left_stance
    right_hip = -drive_gate * step_amp * right_swing - 0.15 * push_amp * right_stance
    left_knee = -0.70 * lift_amp * left_swing - 0.02 * lift_amp * left_stance
    right_knee = -0.70 * lift_amp * right_swing - 0.02 * lift_amp * right_stance
    left_ankle = -0.42 * lift_amp * left_swing - 0.04 * lift_amp * left_stance
    right_ankle = -0.42 * lift_amp * right_swing - 0.04 * lift_amp * right_stance

    lateral_shift = np.cos(phase)
    base[0] += left_hip - 0.05 * pitch_like
    base[1] += lateral_amp * lateral_shift - 0.035 * roll_like
    base[3] += left_knee
    base[4] += left_ankle
    base[5] += -0.35 * lateral_amp * lateral_shift

    base[6] += right_hip - 0.05 * pitch_like
    base[7] += lateral_amp * lateral_shift - 0.035 * roll_like
    base[9] += right_knee
    base[10] += right_ankle
    base[11] += -0.35 * lateral_amp * lateral_shift

    # Catch-step: if a foot trails too far behind, move it forward using the
    # measured sign. Keep this moderate to avoid the backward-collapse seen in
    # the over-governed variant.
    for side, hip_i, knee_i, ankle_i, swing, stance in (
        ("left", 0, 3, 4, left_swing, left_stance),
        ("right", 6, 9, 10, right_swing, right_stance),
    ):
        rel_x = _humanoid_foot_rel_x(env, side)
        foot_z = _humanoid_foot_rel_z(env, side)
        target_rel = float(np.clip(-0.08 + 0.22 * min(overspeed, 0.8), -0.08, 0.10))
        behind = max(target_rel - rel_x, 0.0)
        far_behind = max(-0.34 - rel_x, 0.0)
        catch_gate = swing + 0.22 * stance + float(far_behind > 0.0) * 0.25
        catch = float(np.clip((1.80 * behind + 0.95 * far_behind) * catch_gate, 0.0, 0.48))
        catch += float(np.clip(1.35 * far_behind * (0.35 + overspeed + tilt_need), 0.0, 0.24))
        if catch > 0.0:
            base[hip_i] -= catch
            base[knee_i] -= float(np.clip(0.30 * catch, 0.0, 0.14))
            base[ankle_i] -= float(np.clip(0.10 * catch, 0.0, 0.055))
        contacts = np.asarray(env._get_foot_contacts(), dtype=np.float32)[:2]
        contact = float(contacts[0]) if side == "left" and contacts.shape[0] > 0 else (
            float(contacts[1]) if side == "right" and contacts.shape[0] > 1 else 0.0
        )
        landing = float(contact < 0.5) * float(np.clip((foot_z - 0.065) / 0.08, 0.0, 1.0)) * float(
            np.clip(2.2 * overspeed + height_drop + tilt_need, 0.0, 1.0)
        )
        if landing > 0.0:
            base[knee_i] += float(np.clip(0.10 * landing, 0.0, 0.10))
            base[ankle_i] += float(np.clip(0.045 * landing, 0.0, 0.045))

    if overspeed > 0.0:
        brake = float(
            np.clip(
                0.70 * overspeed
                + 0.22 * early_tilt_need
                + 0.18 * yaw_need
                + 0.45 * speed_emergency
                + 0.12 * height_drop,
                0.0,
                0.62,
            )
        )
        left_brake = brake * (1.0 - 0.4 * left_swing)
        right_brake = brake * (1.0 - 0.4 * right_swing)
        base[0] += left_brake
        base[6] += right_brake
        base[3] += 0.70 * left_brake
        base[9] += 0.70 * right_brake
        base[4] += 0.36 * left_brake
        base[10] += 0.36 * right_brake
        base[14] += float(np.clip(-0.30 * brake - 0.06 * pitch_like, -0.22, 0.04))
    try:
        base_y = float(env.data.xpos[env._trunk_body_id][1])
    except Exception:
        base_y = 0.0
    lateral_err = float(np.clip(base_y, -0.22, 0.22))
    lateral_corr = float(np.clip(0.20 * lateral_err - 0.035 * roll_like, -0.07, 0.07))
    late_lateral_gate = float(np.clip((abs(base_y) - 0.035) / 0.18, 0.0, 1.0))
    if late_lateral_gate > 0.0:
        left_support = 1.0 - 0.65 * left_swing
        right_support = 1.0 - 0.65 * right_swing
        base[1] += lateral_corr * late_lateral_gate * left_support
        base[7] += lateral_corr * late_lateral_gate * right_support
        base[5] -= 0.18 * lateral_corr * late_lateral_gate
        base[11] -= 0.18 * lateral_corr * late_lateral_gate

    recovery = float(
        np.clip(
            (gravity_z + 0.88) / 0.22
            + 0.8 * overspeed
            + 0.55 * yaw_need
            + 0.85 * speed_emergency
            + 1.4 * height_drop,
            0.0,
            1.0,
        )
    )
    if recovery > 0.0:
        # Once the trunk is close to the fall-tilt boundary, stop reaching and
        # absorb the forward pitch with the measured backward-driving signs.
        left_support = 1.0 - 0.55 * left_swing
        right_support = 1.0 - 0.55 * right_swing
        base[0] += 0.30 * recovery * left_support
        base[6] += 0.30 * recovery * right_support
        base[3] += 0.22 * recovery * left_support
        base[9] += 0.22 * recovery * right_support
        base[4] += 0.11 * recovery * left_support
        base[10] += 0.11 * recovery * right_support
        base[14] += float(np.clip(-0.24 * recovery - 0.07 * pitch_like, -0.24, 0.03))
    base[14] += float(np.clip(-0.07 * pitch_like - 0.22 * overspeed + 0.04 * backward, -0.18, 0.06))
    yaw_gain = float(getattr(env, "gait_yaw_hip_amp", 0.0))
    if yaw_gain <= 0.0:
        yaw_gain = 1.0
    base = _apply_g1_yaw_lateral_stabilizer(env, base, gain=yaw_gain)
    return np.clip(base, env._ctrl_low, env._ctrl_high).astype(np.float32)


def footstep_ik_walkform_action(env: Any) -> np.ndarray:
    """G1 nominal gait that prioritizes walking form over forward distance.

    The faster forward prior can create reward-hacking rollouts: both feet leave
    contact, trunk pitch grows, and the robot earns x-progress through a lunge.
    This variant keeps the useful forward gait but adds an event-triggered form
    guard only during flight, overspeed, or trunk-pitch risk.
    """
    base = footstep_ik_forward_action(env)
    if env.model.nu < 29 or env.stage != "walk":
        return base

    phase = env.get_gait_phase()
    raw = float(np.sin(phase))
    left_phase = float(np.clip((raw - 0.04) / 0.78, 0.0, 1.0))
    right_phase = float(np.clip((-raw - 0.04) / 0.78, 0.0, 1.0))
    left_swing = float(0.5 - 0.5 * np.cos(np.pi * left_phase))
    right_swing = float(0.5 - 0.5 * np.cos(np.pi * right_phase))
    left_stance = 1.0 - left_swing
    right_stance = 1.0 - right_swing

    cmd_vx = float(env.command[0])
    vx = float(env.data.qvel[0]) if getattr(env, "data", None) is not None else 0.0
    projected_gravity = env._get_projected_gravity()
    pitch_like = float(np.clip(projected_gravity[0], -0.6, 0.6))
    roll_like = float(np.clip(projected_gravity[1], -0.6, 0.6))
    gravity_z = float(projected_gravity[2])
    yaw_abs = abs(float(_base_yaw_wxyz(env)))
    height = float(env.data.xpos[env._trunk_body_id][2])
    contacts = np.asarray(env._get_foot_contacts(), dtype=np.float32)[:2]
    left_contact = float(contacts[0]) if contacts.shape[0] > 0 else 0.0
    right_contact = float(contacts[1]) if contacts.shape[0] > 1 else 0.0
    both_air = float(left_contact < 0.5 and right_contact < 0.5)

    overspeed = max(vx - cmd_vx - 0.03, 0.0)
    speed_emergency = max(vx - 0.42, 0.0)
    early_tilt_need = max(gravity_z + 0.95, 0.0)
    tilt_need = max(gravity_z + 0.86, 0.0)
    yaw_need = max(yaw_abs - 0.24, 0.0)
    height_drop = max(0.67 - height, 0.0)
    form_need = float(
        np.clip(
            1.2 * overspeed
            + 2.2 * speed_emergency
            + 1.5 * early_tilt_need
            + 1.1 * yaw_need
            + 1.0 * height_drop
            + 0.70 * both_air,
            0.0,
            1.0,
        )
    )

    # Stance guard: if the nominal stance foot is airborne, pull that leg back
    # toward a near-home support shape instead of allowing both legs to fly. The
    # correction is weak unless the rollout is already hacking progress.
    for side, hip_i, knee_i, ankle_i, swing, stance, contact in (
        ("left", 0, 3, 4, left_swing, left_stance, left_contact),
        ("right", 6, 9, 10, right_swing, right_stance, right_contact),
    ):
        foot_z = _humanoid_foot_rel_z(env, side)
        stance_air = float(stance > 0.58 and contact < 0.5)
        landing_need = float(
            np.clip(
                stance_air * (0.25 + 1.6 * max(foot_z - 0.070, 0.0))
                + 0.45 * both_air
                + 0.20 * form_need,
                0.0,
                1.0,
            )
        )
        if landing_need > 0.0:
            base[hip_i] += float(np.clip(0.075 * landing_need, 0.0, 0.075)) * stance
            base[knee_i] += float(np.clip(0.055 * landing_need, 0.0, 0.055)) * stance
            base[ankle_i] += float(np.clip(0.035 * landing_need, 0.0, 0.035)) * stance

        rel_x = _humanoid_foot_rel_x(env, side)
        behind = max(-0.08 - rel_x, 0.0)
        catch = float(np.clip(0.45 * behind * swing * (1.0 - 0.65 * form_need), 0.0, 0.08))
        if catch > 0.0:
            base[hip_i] -= catch
            base[knee_i] -= 0.18 * catch
            base[ankle_i] -= 0.08 * catch

    brake = float(np.clip(0.28 * form_need + 0.24 * speed_emergency + 0.18 * tilt_need, 0.0, 0.38))
    if brake > 0.0:
        left_support = 1.0 - 0.45 * left_swing
        right_support = 1.0 - 0.45 * right_swing
        base[0] += 0.11 * brake * left_support
        base[6] += 0.11 * brake * right_support
        base[3] += 0.08 * brake * left_support
        base[9] += 0.08 * brake * right_support
        base[4] += 0.04 * brake * left_support
        base[10] += 0.04 * brake * right_support
        base[14] += float(np.clip(-0.22 * brake - 0.06 * pitch_like, -0.18, 0.02))

    base[1] += float(np.clip(-0.025 * roll_like, -0.025, 0.025))
    base[7] += float(np.clip(-0.025 * roll_like, -0.025, 0.025))
    base[14] += float(np.clip(-0.08 * pitch_like - 0.16 * overspeed - 0.10 * both_air, -0.20, 0.025))
    return np.clip(base, env._ctrl_low, env._ctrl_high).astype(np.float32)


def footstep_ik_lipm_capture_action(env: Any) -> np.ndarray:
    """Footstep IK with a capture-point style sagittal foot target.

    The previous governed variants moved forward, but terminal logs showed the
    pelvis outrunning both feet. This layer estimates a simple LIPM capture
    offset, then asks the airborne or trailing foot to land farther ahead of
    the pelvis when velocity exceeds command.
    """
    base = footstep_ik_landing_governed_action(env)
    if env.model.nu < 29 or env.stage != "walk":
        return base

    phase = env.get_gait_phase()
    raw = float(np.sin(phase))
    left_phase = float(np.clip((raw - 0.02) / 0.80, 0.0, 1.0))
    right_phase = float(np.clip((-raw - 0.02) / 0.80, 0.0, 1.0))
    left_swing = float(0.5 - 0.5 * np.cos(np.pi * left_phase))
    right_swing = float(0.5 - 0.5 * np.cos(np.pi * right_phase))
    left_stance = 1.0 - left_swing
    right_stance = 1.0 - right_swing

    vx = float(env.data.qvel[0]) if getattr(env, "data", None) is not None else 0.0
    cmd_vx = float(env.command[0])
    height = float(env.data.xpos[env._trunk_body_id][2])
    projected_gravity = env._get_projected_gravity()
    gravity_z = float(projected_gravity[2])
    roll_like = float(np.clip(projected_gravity[1], -0.7, 0.7))
    pitch_like = float(np.clip(projected_gravity[0], -0.7, 0.7))
    capture_time = float(np.sqrt(max(height, 0.35) / 9.81))
    overspeed = max(vx - cmd_vx, 0.0)
    tilt_need = max(gravity_z + 0.82, 0.0)
    capture_rel = float(np.clip(-0.08 + 0.75 * capture_time * overspeed, -0.08, 0.16))
    emergency = float(np.clip(1.2 * overspeed + 1.0 * tilt_need, 0.0, 1.4))
    contacts = np.asarray(env._get_foot_contacts(), dtype=np.float32)[:2]

    for side, hip_i, roll_i, knee_i, ankle_i, ankle_roll_i, swing, stance, contact in (
        (
            "left",
            0,
            1,
            3,
            4,
            5,
            left_swing,
            left_stance,
            float(contacts[0]) if contacts.shape[0] > 0 else 0.0,
        ),
        (
            "right",
            6,
            7,
            9,
            10,
            11,
            right_swing,
            right_stance,
            float(contacts[1]) if contacts.shape[0] > 1 else 0.0,
        ),
    ):
        rel_x = _humanoid_foot_rel_x(env, side)
        foot_z = _humanoid_foot_rel_z(env, side)
        err = max(capture_rel - rel_x, 0.0)
        far = max(-0.30 - rel_x, 0.0)
        gate = max(swing, 0.35 * stance * float(emergency > 0.35), 0.45 * float(far > 0.0))
        catch = float(np.clip((1.05 * err + 0.75 * far) * gate * (0.45 + 0.25 * emergency), 0.0, 0.30))
        if catch > 1e-6:
            base[hip_i] -= catch
            base[knee_i] += float(np.clip(0.24 * catch + 0.012 * swing, 0.0, 0.10))
            base[ankle_i] += float(np.clip(0.08 * catch + 0.005 * swing, 0.0, 0.040))

        landing = float(contact < 0.5) * float(np.clip((foot_z - 0.080) / 0.10, 0.0, 1.0)) * min(emergency, 1.0)
        if landing > 1e-6:
            base[knee_i] -= float(np.clip(0.10 * landing, 0.0, 0.10))
            base[ankle_i] -= float(np.clip(0.045 * landing, 0.0, 0.045))

        lateral = float(np.clip(-0.075 * roll_like * (0.35 + stance), -0.055, 0.055))
        base[roll_i] += lateral
        base[ankle_roll_i] -= 0.45 * lateral

    brake = float(np.clip(0.16 * overspeed + 0.08 * tilt_need, 0.0, 0.16))
    if brake > 0.0:
        base[0] += 0.30 * brake
        base[6] += 0.30 * brake
        base[14] += float(np.clip(-0.16 * brake - 0.06 * pitch_like, -0.18, 0.05))

    base = _apply_g1_yaw_lateral_stabilizer(env, base, gain=0.90)
    return np.clip(base, env._ctrl_low, env._ctrl_high).astype(np.float32)


def footstep_ik_landing_strong_action(env: Any) -> np.ndarray:
    """Stronger late-stance landing guard on top of footstep_ik_landing."""
    base = footstep_ik_landing_action(env)
    if env.model.nu < 29 or env.stage != "walk":
        return base

    phase = env.get_gait_phase()
    raw = float(np.sin(phase))
    left_swing = float(np.clip((raw - 0.02) / 0.78, 0.0, 1.0))
    right_swing = float(np.clip((-raw - 0.02) / 0.78, 0.0, 1.0))
    left_swing = float(0.5 - 0.5 * np.cos(np.pi * left_swing))
    right_swing = float(0.5 - 0.5 * np.cos(np.pi * right_swing))
    left_stance = 1.0 - left_swing
    right_stance = 1.0 - right_swing

    vx = float(env.data.qvel[0]) if getattr(env, "data", None) is not None else 0.0
    projected_gravity = env._get_projected_gravity()
    gravity_z = float(projected_gravity[2])
    pitch_like = float(np.clip(projected_gravity[0], -0.6, 0.6))
    late_need = float(np.clip(1.2 * max(vx - 0.45, 0.0) + 1.4 * max(gravity_z + 0.74, 0.0), 0.0, 1.0))
    if late_need <= 1e-6:
        return base

    left_rel = _humanoid_foot_rel_x(env, "left")
    right_rel = _humanoid_foot_rel_x(env, "right")
    left_err = max(-0.22 - left_rel, 0.0)
    right_err = max(-0.22 - right_rel, 0.0)
    left_corr = float(np.clip((0.90 * left_err * left_stance + 0.55 * left_err * left_swing) * late_need, 0.0, 0.16))
    right_corr = float(np.clip((0.90 * right_err * right_stance + 0.55 * right_err * right_swing) * late_need, 0.0, 0.16))

    base[0] -= left_corr
    base[6] -= right_corr
    base[3] += float(np.clip(0.35 * left_corr, 0.0, 0.07))
    base[9] += float(np.clip(0.35 * right_corr, 0.0, 0.07))
    base[14] += float(np.clip(-0.12 * late_need - 0.04 * pitch_like, -0.16, 0.04))

    return np.clip(base, env._ctrl_low, env._ctrl_high).astype(np.float32)


def footstep_ik_landing_upright_action(env: Any) -> np.ndarray:
    """Landing controller with opposite late waist-pitch test."""
    base = footstep_ik_landing_action(env)
    if env.model.nu < 29 or env.stage != "walk":
        return base

    vx = float(env.data.qvel[0]) if getattr(env, "data", None) is not None else 0.0
    gravity_z = float(env._get_projected_gravity()[2])
    late_need = float(np.clip(1.2 * max(vx - 0.45, 0.0) + 1.4 * max(gravity_z + 0.74, 0.0), 0.0, 1.0))
    if late_need > 1e-6:
        base[14] += float(np.clip(0.12 * late_need, -0.04, 0.16))
    return np.clip(base, env._ctrl_low, env._ctrl_high).astype(np.float32)


def footstep_ik_landing_lateral_action(env: Any, *, gain: float = 0.34, ankle_gain: float = 0.25) -> np.ndarray:
    """Landing controller with lateral pelvis-over-stance correction."""
    base = footstep_ik_landing_action(env)
    if env.model.nu < 29 or env.stage != "walk":
        return base

    phase = env.get_gait_phase()
    raw = float(np.sin(phase))
    left_swing = float(np.clip((raw - 0.03) / 0.82, 0.0, 1.0))
    right_swing = float(np.clip((-raw - 0.03) / 0.82, 0.0, 1.0))
    left_swing = float(0.5 - 0.5 * np.cos(np.pi * left_swing))
    right_swing = float(0.5 - 0.5 * np.cos(np.pi * right_swing))

    try:
        base_y = float(env.data.xpos[env._trunk_body_id][1])
    except Exception:
        base_y = 0.0
    projected_gravity = env._get_projected_gravity()
    roll_like = float(np.clip(projected_gravity[1], -0.6, 0.6))
    vx = float(env.data.qvel[0]) if getattr(env, "data", None) is not None else 0.0

    left_contact = 0.0
    right_contact = 0.0
    contacts = np.asarray(env._get_foot_contacts(), dtype=np.float32)[:2]
    if contacts.shape[0] >= 2:
        left_contact = float(contacts[0])
        right_contact = float(contacts[1])

    # In the failing runs the pelvis drifts negative-y while the feet remain at
    # positive rel-y. Bias both hip rolls to push the pelvis back toward center,
    # with extra support on the current stance side.
    lateral_err = float(np.clip(-base_y, -0.25, 0.25))
    late_gate = float(np.clip((vx - 0.25) / 0.55, 0.0, 1.0))
    support_gate = 0.5 + 0.5 * late_gate
    corr = float(np.clip(float(gain) * lateral_err - 0.05 * roll_like, -0.12, 0.12))

    left_support = max(left_contact, 1.0 - left_swing)
    right_support = max(right_contact, 1.0 - right_swing)
    base[1] += corr * support_gate * (0.65 + 0.35 * left_support)
    base[7] += corr * support_gate * (0.65 + 0.35 * right_support)
    base[5] += -float(ankle_gain) * corr * support_gate
    base[11] += -float(ankle_gain) * corr * support_gate

    return np.clip(base, env._ctrl_low, env._ctrl_high).astype(np.float32)


def footstep_cartesian_action(env: Any) -> np.ndarray:
    return _footstep_cartesian_action(env, capture=False)


def footstep_capture_action(env: Any) -> np.ndarray:
    return _footstep_cartesian_action(env, capture=True, aggressive=False)


def footstep_capture_aggressive_action(env: Any) -> np.ndarray:
    return _footstep_cartesian_action(env, capture=True, aggressive=True)


def footstep_capture_earlycatch_action(env: Any) -> np.ndarray:
    return _footstep_cartesian_action(env, capture=True, earlycatch=True)


def footstep_capture_swingcatch_action(env: Any) -> np.ndarray:
    return _footstep_cartesian_action(env, capture=True, swingcatch=True)


def footstep_capture_speedbrake_action(env: Any) -> np.ndarray:
    return _footstep_cartesian_action(env, capture=True, speedbrake=True)


def footstep_capture_placement_action(env: Any) -> np.ndarray:
    return _footstep_cartesian_action(env, capture=True, placement_feedback=True)


def footstep_capture_pitchcatch_action(env: Any) -> np.ndarray:
    return _footstep_cartesian_action(env, capture=True, pitchcatch=True)


def footstep_capture_toelift_action(env: Any) -> np.ndarray:
    base = _footstep_cartesian_action(env, capture=True)
    if env.model.nu < 29 or env.stage != "walk":
        return base

    phase = env.get_gait_phase()
    raw = float(np.sin(phase))
    left_swing = float(np.clip((raw - 0.18) / 0.62, 0.0, 1.0))
    right_swing = float(np.clip((-raw - 0.18) / 0.62, 0.0, 1.0))
    left_swing = float(0.5 - 0.5 * np.cos(np.pi * left_swing))
    right_swing = float(0.5 - 0.5 * np.cos(np.pi * right_swing))

    contacts = np.asarray(env._get_foot_contacts(), dtype=np.float32)[:2]
    left_contact = float(contacts[0]) if contacts.shape[0] > 0 else 1.0
    right_contact = float(contacts[1]) if contacts.shape[0] > 1 else 1.0
    left_z = _humanoid_foot_rel_z(env, "left")
    right_z = _humanoid_foot_rel_z(env, "right")
    target = 0.075

    # Only break contact in the middle of swing; do not add a large clearance
    # pose during stance/double support, which made earlier high-clearance
    # variants fall backward immediately.
    left_need = left_swing * max(target - left_z, 0.0) * (0.35 + 0.65 * left_contact)
    right_need = right_swing * max(target - right_z, 0.0) * (0.35 + 0.65 * right_contact)
    base[3] += float(np.clip(0.65 * left_need + 0.008 * left_swing, 0.0, 0.035))
    base[9] += float(np.clip(0.65 * right_need + 0.008 * right_swing, 0.0, 0.035))
    base[4] += float(np.clip(0.25 * left_need + 0.003 * left_swing, 0.0, 0.014))
    base[10] += float(np.clip(0.25 * right_need + 0.003 * right_swing, 0.0, 0.014))

    return np.clip(base, env._ctrl_low, env._ctrl_high).astype(np.float32)


def footstep_capture_highclearance_action(env: Any) -> np.ndarray:
    return _footstep_cartesian_action(env, capture=True, highclearance=True)


def footstep_march_action(env: Any) -> np.ndarray:
    base = home_action(env)
    if env.model.nu < 29 or env.stage != "walk":
        return base

    phase = env.get_gait_phase()
    raw = np.sin(phase)
    left_swing = float(np.clip((raw - 0.10) / 0.55, 0.0, 1.0))
    right_swing = float(np.clip((-raw - 0.10) / 0.55, 0.0, 1.0))
    left_swing = float(0.5 - 0.5 * np.cos(np.pi * left_swing))
    right_swing = float(0.5 - 0.5 * np.cos(np.pi * right_swing))
    lateral_shift = float(np.sin(phase + np.pi / 2.0))

    # Start from the stable home pose. Earlier crouched march targets caused a
    # large backward drift before any useful stepping behavior emerged.

    # Shift pelvis toward stance foot before lifting the swing leg.
    lateral_amp = 0.020
    base[1] += lateral_amp * lateral_shift
    base[7] += lateral_amp * lateral_shift
    base[5] += -0.35 * lateral_amp * lateral_shift
    base[11] += -0.35 * lateral_amp * lateral_shift

    # Step-in-place swing pose from kinematic search: coordinated leg
    # shortening raises the foot while keeping pelvis-relative x near neutral.
    swing_knee = 0.10
    swing_hip = -0.02
    swing_ankle = 0.02
    base[0] += swing_hip * left_swing
    base[3] += swing_knee * left_swing
    base[4] += swing_ankle * left_swing
    base[6] += swing_hip * right_swing
    base[9] += swing_knee * right_swing
    base[10] += swing_ankle * right_swing

    # Light arm counter-swing for balance.
    base[15] += -0.08 * raw
    base[22] += 0.08 * raw

    return np.clip(base, env._ctrl_low, env._ctrl_high).astype(np.float32)


def footstep_capture_lipm_action(env: Any) -> np.ndarray:
    return _footstep_cartesian_action(env, capture=True, aggressive=False, lipm_capture=True)


def footstep_capture_lipm_swingcatch_action(env: Any) -> np.ndarray:
    return _footstep_cartesian_action(
        env,
        capture=True,
        aggressive=False,
        swingcatch=True,
        lipm_capture=True,
    )


def footstep_capture_lipm_aggressive_action(env: Any) -> np.ndarray:
    return _footstep_cartesian_action(
        env,
        capture=True,
        aggressive=True,
        lipm_capture=True,
    )


def footstep_bounded_action(env: Any) -> np.ndarray:
    """Bounded stepping prior with explicit swing-foot clearance.

    This is a deliberately simple pre-RL controller for the current CPU
    workflow. The reference-motion branch can move forward but drags the feet;
    the LIPM footstep branch stays upright but does not progress. This prior
    chooses the missing middle: keep commanded speed bounded, lift exactly one
    swing foot, and make a short catch step rather than asking PPO to discover
    the whole biped gait.
    """
    base = home_action(env)
    if env.model.nu < 29 or env.stage != "walk":
        return base

    phase = env.get_gait_phase()
    raw = float(np.sin(phase))
    left_phase = float(np.clip((raw - 0.08) / 0.72, 0.0, 1.0))
    right_phase = float(np.clip((-raw - 0.08) / 0.72, 0.0, 1.0))
    left_swing = float(0.5 - 0.5 * np.cos(np.pi * left_phase))
    right_swing = float(0.5 - 0.5 * np.cos(np.pi * right_phase))
    left_stance = 1.0 - left_swing
    right_stance = 1.0 - right_swing

    cmd_vx = float(env.command[0])
    reference_speed = float(getattr(env, "gait_command_reference_speed", 0.23))
    max_scale = float(getattr(env, "gait_command_max_scale", 1.0))
    speed_scale = float(np.clip(abs(cmd_vx) / max(reference_speed, 1e-6), 0.20, max_scale))
    amp_scale = float(getattr(env, "gait_command_scale", 1.0)) * speed_scale

    vx = float(env.data.qvel[0]) if getattr(env, "data", None) is not None else 0.0
    projected_gravity = env._get_projected_gravity()
    pitch_like = float(np.clip(projected_gravity[0], -0.55, 0.55))
    roll_like = float(np.clip(projected_gravity[1], -0.55, 0.55))
    gravity_z = float(projected_gravity[2])
    overspeed = max(vx - cmd_vx - 0.06, 0.0)
    underspeed = max(cmd_vx - vx, 0.0)
    fall_tilt = max(gravity_z + 0.72, 0.0)

    drive_gate = float(np.clip(1.0 - 3.5 * overspeed - 1.8 * fall_tilt, 0.12, 1.0))
    reach_gate = float(np.clip(1.0 - 1.4 * max(vx - 0.55, 0.0), 0.35, 1.0))

    # Slight crouch keeps the knee away from a straight-leg singularity, but it
    # is shallower than old footstep priors so the foot can lift instead of
    # scraping forward.
    base[0] += -0.075
    base[3] += 0.155
    base[4] += -0.070
    base[6] += -0.075
    base[9] += 0.155
    base[10] += -0.070
    base[14] += 0.055

    step_len = float(getattr(env, "gait_thigh_amp", 0.18)) * amp_scale
    lift = float(getattr(env, "gait_calf_amp", 0.34)) * amp_scale
    forward_comp = float(getattr(env, "gait_yaw_forward_compensation", 0.10)) * amp_scale

    # Kinematic finite-difference on this XML: negative hip pitch moves the
    # foot forward. Keep stance almost neutral for now; the immediate blocker
    # is toe drag, not lack of push-off.
    swing_reach = (0.11 + 0.32 * step_len + 0.06 * min(underspeed, 0.35)) * reach_gate
    stance_trail = (0.004 + 0.04 * forward_comp) * drive_gate
    brake = 0.08 * overspeed + 0.12 * max(vx - 0.75, 0.0) + 0.10 * fall_tilt

    left_hip = -swing_reach * left_swing + stance_trail * left_stance - brake - 0.025 * pitch_like
    right_hip = -swing_reach * right_swing + stance_trail * right_stance - brake - 0.025 * pitch_like

    # The main failure in rendered videos is toe drag. Use a strong but bounded
    # shortening pattern during swing: knee flexion plus ankle dorsiflexion.
    left_lift = lift * (1.05 + 0.25 * reach_gate) * left_swing
    right_lift = lift * (1.05 + 0.25 * reach_gate) * right_swing
    left_knee = left_lift - 0.035 * left_stance
    right_knee = right_lift - 0.035 * right_stance
    left_ankle = 0.50 * left_lift - 0.020 * left_stance
    right_ankle = 0.50 * right_lift - 0.020 * right_stance

    # If the measured swing foot is still near the floor, add direct clearance
    # feedback. This makes the nominal test reveal whether the G1 model can
    # actually execute the swing, before residual PPO enters the loop.
    clearance_target = 0.105
    left_clearance_err = max(clearance_target - _humanoid_foot_rel_z(env, "left"), 0.0)
    right_clearance_err = max(clearance_target - _humanoid_foot_rel_z(env, "right"), 0.0)
    left_knee += float(np.clip(3.0 * left_clearance_err * left_swing, 0.0, 0.24))
    right_knee += float(np.clip(3.0 * right_clearance_err * right_swing, 0.0, 0.24))
    left_ankle += float(np.clip(1.4 * left_clearance_err * left_swing, 0.0, 0.09))
    right_ankle += float(np.clip(1.4 * right_clearance_err * right_swing, 0.0, 0.09))

    lateral_shift = float(np.sin(phase + np.pi / 2.0))
    lateral_amp = float(getattr(env, "gait_yaw_turn_scale", 0.0))
    if abs(lateral_amp) <= 1e-9:
        lateral_amp = 0.030
    lateral_amp *= amp_scale
    double_support = 0.25 + 0.75 * abs(np.cos(phase))

    base[0] += left_hip
    base[1] += lateral_amp * lateral_shift - 0.040 * roll_like
    base[3] += left_knee
    base[4] += left_ankle
    base[5] += -0.30 * lateral_amp * lateral_shift

    base[6] += right_hip
    base[7] += lateral_amp * lateral_shift - 0.040 * roll_like
    base[9] += right_knee
    base[10] += right_ankle
    base[11] += -0.30 * lateral_amp * lateral_shift

    base[14] += -0.08 * pitch_like - 0.18 * overspeed - 0.16 * fall_tilt
    base[15] += -0.09 * raw * double_support
    base[22] += 0.09 * raw * double_support

    return np.clip(base, env._ctrl_low, env._ctrl_high).astype(np.float32)


def _footstep_cartesian_action(
    env: Any,
    *,
    capture: bool,
    aggressive: bool = False,
    earlycatch: bool = False,
    swingcatch: bool = False,
    speedbrake: bool = False,
    placement_feedback: bool = False,
    pitchcatch: bool = False,
    highclearance: bool = False,
    lipm_capture: bool = False,
    reference_phase: bool = False,
) -> np.ndarray:
    """Foot-placement-oriented G1 walking prior.

    This is still a joint-space controller, but its gains are chosen from the
    measured local foot-position sensitivities:

    - hip pitch: d(foot_rel_x)/dq ~= -0.65 m/rad
    - knee: d(foot_rel_x)/dq ~= -0.32 m/rad

    The controller therefore specifies a desired swing/stance foot x relative
    to the pelvis, then maps that target mostly through hip pitch while using
    the knee for clearance. It is meant to be a better nominal before PPO, not
    a learned policy replacement.
    """
    base = home_action(env)
    if env.model.nu < 29 or env.stage != "walk":
        return base

    phase = env.get_gait_phase()
    reference_speed = float(getattr(env, "gait_command_reference_speed", 0.18))
    max_scale = float(getattr(env, "gait_command_max_scale", 1.0))
    cmd_vx = float(env.command[0])
    vx_scale = float(np.clip(abs(cmd_vx) / max(reference_speed, 1e-6), 0.0, max_scale))
    amp_scale = float(getattr(env, "gait_command_scale", 1.0)) * vx_scale
    if amp_scale <= 1e-6:
        return base

    # Conservative crouch. Slightly less knee bend than footstep_ik so the
    # swing knee has room to add visible clearance.
    base_hip = -0.10
    base_knee = 0.22
    base_ankle = -0.11
    base_waist_pitch = 0.08
    base[0] += base_hip
    base[3] += base_knee
    base[4] += base_ankle
    base[6] += base_hip
    base[9] += base_knee
    base[10] += base_ankle
    base[14] += base_waist_pitch

    raw = np.sin(phase)
    left_swing = max(0.0, raw)
    right_swing = max(0.0, -raw)
    left_swing_gate = left_swing
    right_swing_gate = right_swing
    if reference_phase:
        get_reference_contact = getattr(env, "get_reference_contact", None)
        ref_contact = (
            np.asarray(get_reference_contact(), dtype=np.float32)
            if callable(get_reference_contact)
            else np.zeros(2, dtype=np.float32)
        )
        if ref_contact.shape[0] >= 2 and float(np.max(ref_contact)) > 0.0:
            left_swing = float(np.clip(1.0 - ref_contact[0], 0.0, 1.0))
            right_swing = float(np.clip(1.0 - ref_contact[1], 0.0, 1.0))
            if left_swing > 0.5 and right_swing > 0.5:
                # Retarget contact extraction can briefly mark both feet as
                # airborne. Keep one support foot using the analytic phase.
                if raw >= 0.0:
                    right_swing = 0.0
                else:
                    left_swing = 0.0
            elif left_swing < 0.5 and right_swing < 0.5:
                # Double support is useful, but if it persists the feet drag.
                # Open a short commanded swing window from the reference clock.
                if raw >= 0.0:
                    left_swing = float(np.clip(raw, 0.0, 1.0))
                    right_swing = 0.0
                else:
                    left_swing = 0.0
                    right_swing = float(np.clip(-raw, 0.0, 1.0))
            left_swing_gate = left_swing
            right_swing_gate = right_swing
    if highclearance:
        # Keep a more binary swing window. The old sine profile leaves a long,
        # shallow transition where the foot still touches the floor and drags.
        if reference_phase:
            left_swing_gate = float(np.clip((left_swing - 0.12) / 0.55, 0.0, 1.0))
            right_swing_gate = float(np.clip((right_swing - 0.12) / 0.55, 0.0, 1.0))
        else:
            left_swing_gate = float(np.clip((raw - 0.12) / 0.55, 0.0, 1.0))
            right_swing_gate = float(np.clip((-raw - 0.12) / 0.55, 0.0, 1.0))
        left_swing_gate = float(0.5 - 0.5 * np.cos(np.pi * left_swing_gate))
        right_swing_gate = float(0.5 - 0.5 * np.cos(np.pi * right_swing_gate))
    left_stance = 1.0 - left_swing
    right_stance = 1.0 - right_swing
    double_support = 0.20 + 0.80 * abs(np.cos(phase))

    vx = float(env.data.qvel[0]) if getattr(env, "data", None) is not None else 0.0
    governor_vx = min(cmd_vx, reference_speed) if cmd_vx >= 0.0 else max(cmd_vx, -reference_speed)
    vx_err = float(np.clip(governor_vx - vx, -0.20, 0.20))
    projected_gravity = env._get_projected_gravity()
    pitch_like = float(np.clip(projected_gravity[0], -0.5, 0.5))
    roll_like = float(np.clip(projected_gravity[1], -0.5, 0.5))
    gravity_z = float(projected_gravity[2])

    overspeed = max(vx - governor_vx - 0.03, 0.0)
    emergency_speed = max(vx - 0.90, 0.0)
    speed_gate = float(np.clip(1.0 - 4.0 * overspeed, 0.04, 1.0))
    pitch_gate = float(np.clip((-0.55 - gravity_z) / 0.30, 0.04, 1.0))
    drive_gate = min(speed_gate, pitch_gate)
    capture_need = 0.0
    if capture:
        speed_start = 0.34 if aggressive else 0.45
        if earlycatch or swingcatch:
            speed_start = 0.39
        if placement_feedback:
            speed_start = 0.18
        capture_need = max(vx - speed_start, 0.0) / 0.60
        tilt_start = 0.84 if aggressive else 0.78
        if earlycatch or swingcatch:
            tilt_start = 0.80
        if placement_feedback:
            tilt_start = 0.88
        capture_need += max(gravity_z + tilt_start, 0.0) / 0.35
        capture_need = float(np.clip(capture_need, 0.0, 1.0))

    step_len_base = float(getattr(env, "gait_thigh_amp", 0.12)) * amp_scale
    # Overspeed should reduce propulsive push, but not the capture step. Earlier
    # versions gated swing placement down during overspeed and both feet stayed
    # behind the pelvis during the late forward fall.
    swing_step_len = step_len_base * (0.65 + 0.35 * pitch_gate)
    if capture:
        swing_gain = 1.55 if aggressive else 1.10
        swing_step_len *= 1.0 + swing_gain * capture_need
    step_len = step_len_base * drive_gate
    lift = float(getattr(env, "gait_calf_amp", 0.10)) * amp_scale
    push = float(getattr(env, "gait_yaw_forward_compensation", 0.10)) * amp_scale * drive_gate
    lateral_base = float(getattr(env, "gait_yaw_turn_scale", 0.0))
    if abs(lateral_base) <= 1e-9:
        lateral_base = 0.045
    lateral_amp = lateral_base * amp_scale
    early_speed = max(vx - (0.24 if aggressive else 0.30), 0.0) if capture else 0.0

    # Desired pelvis-relative foot positions. Positive x means foot ahead of
    # pelvis. Swing foot is placed ahead, stance foot moves only mildly behind.
    capture_extension = 0.0
    if capture:
        capture_extension = 0.06 * capture_need + 0.10 * min(overspeed, 1.0) + 0.08 * min(emergency_speed, 1.0)
        if earlycatch:
            capture_extension += 0.04 * capture_need + 0.04 * min(early_speed, 1.0)
        if aggressive:
            capture_extension += 0.08 * capture_need + 0.10 * min(early_speed, 1.0)
    swing_x = 0.04 + 0.80 * swing_step_len + 0.20 * max(vx_err, 0.0) + capture_extension
    if lipm_capture:
        trunk_height = 0.75
        try:
            trunk_height = max(float(env.data.xpos[env._trunk_body_id][2]), 0.25)
        except Exception:
            pass
        capture_time = float(np.sqrt(trunk_height / 9.81))
        capture_x = vx * capture_time
        excess_capture = max(capture_x - 0.03, 0.0)
        swing_x += float(np.clip(0.45 * excess_capture, 0.0, 0.18))
    stance_x = -0.012 - 0.25 * push
    # If moving too fast, stop asking the stance foot to trail farther behind.
    stance_x += 0.22 * min(overspeed, 1.0)
    stance_x += 0.22 * min(emergency_speed, 1.0)
    stance_x += 0.26 * min(early_speed, 1.0)
    if aggressive:
        stance_x += 0.12 * min(early_speed, 1.0)
    if lipm_capture:
        stance_x += 0.18 * min(early_speed, 1.0)

    placement_sign = float(getattr(env, "gait_yaw_turn_direction", 1.0))
    placement_sign = 1.0 if placement_sign >= 0.0 else -1.0
    left_x_target = placement_sign * (swing_x * left_swing + stance_x * left_stance)
    right_x_target = placement_sign * (swing_x * right_swing + stance_x * right_stance)

    # Map desired foot x to hip pitch. d(rel_x)/d(hip_pitch) is negative.
    hip_gain = -1.0 / 0.65
    left_hip_pitch = hip_gain * left_x_target
    right_hip_pitch = hip_gain * right_x_target
    if highclearance:
        # Add explicit forward reach during the true swing window. This is
        # intentionally separate from lift so the foot does not only rise in
        # place while still trailing behind the pelvis.
        left_hip_pitch += -0.11 * left_swing_gate
        right_hip_pitch += -0.11 * right_swing_gate

    if capture and capture_need > 0.0:
        # During late forward falls both feet are far behind the pelvis. Bias
        # the swing leg toward a forward catch step, while keeping stance
        # corrections small to avoid tripping the support leg.
        catch_x = 0.02 if aggressive else -0.22
        if earlycatch:
            catch_x = -0.08
        if swingcatch:
            catch_x = -0.02
        left_err = max(catch_x - _humanoid_foot_rel_x(env, "left"), 0.0)
        right_err = max(catch_x - _humanoid_foot_rel_x(env, "right"), 0.0)
        stance_weight = 0.14 if aggressive else 0.06
        if earlycatch:
            stance_weight = 0.08
        if swingcatch:
            stance_weight = 0.0
        left_weight = left_swing + stance_weight * left_stance
        right_weight = right_swing + stance_weight * right_stance
        catch_gain = 0.42 if aggressive else 0.26
        min_catch = -0.34 if aggressive else -0.20
        if earlycatch:
            catch_gain = 0.32
            min_catch = -0.26
        if swingcatch:
            catch_gain = 0.48
            min_catch = -0.42
        left_hip_pitch += float(np.clip(catch_gain * hip_gain * left_err * capture_need * left_weight, min_catch, 0.02))
        right_hip_pitch += float(np.clip(catch_gain * hip_gain * right_err * capture_need * right_weight, min_catch, 0.02))

    placement_left_err = 0.0
    placement_right_err = 0.0
    if placement_feedback:
        # Normal successful segments keep both feet close to -0.02 m relative to
        # the pelvis. The failure mode starts when feet drift past about -0.15 m,
        # so feed back measured foot placement before the velocity runaway.
        placement_target = -0.055
        left_rel_x = _humanoid_foot_rel_x(env, "left")
        right_rel_x = _humanoid_foot_rel_x(env, "right")
        placement_left_err = max(placement_target - left_rel_x, 0.0)
        placement_right_err = max(placement_target - right_rel_x, 0.0)
        both_behind = float(left_rel_x < -0.14 and right_rel_x < -0.14)
        left_weight = left_swing + 0.02 * left_stance + 0.05 * both_behind * left_stance
        right_weight = right_swing + 0.02 * right_stance + 0.05 * both_behind * right_stance
        left_hip_pitch += float(np.clip(0.42 * hip_gain * placement_left_err * left_weight, -0.24, 0.0))
        right_hip_pitch += float(np.clip(0.42 * hip_gain * placement_right_err * right_weight, -0.24, 0.0))

    # Knee adds clearance, ankle roughly counter-rotates to avoid toe drag.
    left_knee = lift * (0.30 + 1.30 * left_swing) - 0.03 * left_stance
    right_knee = lift * (0.30 + 1.30 * right_swing) - 0.03 * right_stance
    if capture:
        left_knee += 0.070 * capture_need * left_swing
        right_knee += 0.070 * capture_need * right_swing
        if aggressive:
            left_knee += 0.050 * capture_need * left_swing
            right_knee += 0.050 * capture_need * right_swing
    if placement_feedback:
        left_knee += 0.10 * min(placement_left_err / 0.22, 1.0) * left_swing
        right_knee += 0.10 * min(placement_right_err / 0.22, 1.0) * right_swing
    if highclearance:
        clearance_boost = max(0.045, 1.15 * lift)
        # Kinematic grid checks on G1 showed that visible clearance needs a
        # coordinated shortening pattern, not a tiny knee offset. Approximate
        # useful swing pose relative to crouch: hip pitch -0.18, knee +0.45,
        # ankle pitch +0.18. Keep it gated so stance support is not destroyed.
        left_knee += (0.30 + 0.75 * clearance_boost) * left_swing_gate
        right_knee += (0.30 + 0.75 * clearance_boost) * right_swing_gate
        left_hip_pitch += -0.30 * left_swing_gate
        right_hip_pitch += -0.30 * right_swing_gate
    left_ankle = -0.45 * lift * left_swing + 0.08 * lift * left_stance
    right_ankle = -0.45 * lift * right_swing + 0.08 * lift * right_stance
    if highclearance:
        left_ankle += 0.22 * left_swing_gate
        right_ankle += 0.22 * right_swing_gate

    lateral_shift = np.sin(phase + np.pi / 2.0)
    yaw_norm = float(np.clip(env.command[2] / 0.5, -1.0, 1.0))
    yaw_amp = float(getattr(env, "gait_yaw_hip_amp", 0.0))

    brake_bias = 0.30 * overspeed + 0.45 * emergency_speed + 0.10 * max(gravity_z + 0.75, 0.0)
    if capture:
        brake_bias += 0.34 * min(early_speed, 1.0)
    if speedbrake:
        brake_bias += 0.26 * max(vx - 0.65, 0.0) + 0.18 * max(vx - 0.95, 0.0)
    base[0] += left_hip_pitch - brake_bias - 0.04 * pitch_like
    base[1] += lateral_amp * lateral_shift - 0.04 * roll_like
    base[2] += yaw_amp * yaw_norm
    base[3] += left_knee
    base[4] += left_ankle
    base[5] += -0.35 * lateral_amp * lateral_shift

    base[6] += right_hip_pitch - brake_bias - 0.04 * pitch_like
    base[7] += lateral_amp * lateral_shift - 0.04 * roll_like
    base[8] += -yaw_amp * yaw_norm
    base[9] += right_knee
    base[10] += right_ankle
    base[11] += -0.35 * lateral_amp * lateral_shift

    base[14] += (
        0.01 * amp_scale
        - 0.06 * pitch_like
        - 0.24 * overspeed
        - 0.36 * emergency_speed
        - 0.28 * min(early_speed, 1.0)
    )
    if speedbrake:
        base[14] += -0.22 * max(vx - 0.65, 0.0) - 0.28 * max(gravity_z + 0.72, 0.0)
    if pitchcatch:
        base[14] += 0.34 * max(pitch_like - 0.35, 0.0) + 0.12 * max(vx - 0.75, 0.0)
    base[15] += -0.20 * step_len * raw * double_support
    base[22] += 0.20 * step_len * raw * double_support

    return np.clip(base, env._ctrl_low, env._ctrl_high).astype(np.float32)


def humanoid_walk_status() -> dict[str, str]:
    return {
        "status": "implemented",
        "reason": "Phase-based humanoid walk prior for early residual PPO and imitation/RL experiments.",
    }
