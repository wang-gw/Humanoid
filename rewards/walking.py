from __future__ import annotations

import numpy as np


class WalkingReward:
    """Commanded locomotion reward for early residual walking experiments."""

    def __init__(
        self,
        alive_bonus: float = 1.0,
        w_lin_vel: float = 1.5,
        w_lateral_vel: float = 0.0,
        w_yaw_rate: float = 0.75,
        w_upright: float = 1.0,
        w_ang: float = 0.04,
        w_height: float = 0.5,
        w_dact: float = 0.03,
        w_joint_dev: float = 0.04,
        w_nominal_action: float = 0.0,
        w_contact: float = 0.05,
        w_phase_contact: float = 0.0,
        w_swing_foot_clearance: float = 0.0,
        swing_foot_clearance_target: float = 0.12,
        clearance_requires_air: bool = False,
        w_swing_contact_penalty: float = 0.0,
        w_double_support_penalty: float = 0.0,
        w_foot_slip: float = 0.0,
        w_foot_behind: float = 0.0,
        foot_behind_threshold: float = -0.42,
        w_double_foot_behind: float = 0.0,
        double_foot_behind_threshold: float = -0.36,
        w_footstep_forward: float = 0.0,
        w_footstep_backstep: float = 0.0,
        w_footstep_lateral: float = 0.0,
        w_landing_relx: float = 0.0,
        landing_relx_target: float = -0.04,
        landing_relx_scale: float = 0.16,
        footstep_lateral_target: float = 0.24,
        footstep_forward_clip: float = 0.20,
        footstep_backstep_scale: float = 1.0,
        w_foot_width: float = 0.0,
        foot_width_target: float = 0.24,
        foot_width_tolerance: float = 0.08,
        w_foot_crossing: float = 0.0,
        foot_crossing_margin: float = 0.02,
        w_no_flight: float = 0.0,
        w_body_forward_z: float = 0.0,
        body_forward_z_threshold: float = 0.20,
        w_lin_overspeed: float = 0.0,
        w_abs_overspeed: float = 0.0,
        abs_overspeed_threshold: float = 0.85,
        w_backward_vel: float = 0.0,
        w_lateral_position: float = 0.0,
        lateral_position_grace: float = 0.06,
        w_lateral_capture: float = 0.0,
        lateral_capture_margin: float = 0.08,
        lateral_capture_clip: float = 0.20,
        lateral_capture_vy_threshold: float = 0.0,
        w_roll_bias: float = 0.0,
        roll_bias_grace: float = 0.04,
        roll_bias_decay: float = 0.995,
        w_yaw_position: float = 0.0,
        yaw_position_grace: float = 0.20,
        w_forward_progress: float = 0.0,
        forward_progress_clip: float = 1.0,
        w_waypoint_progress: float = 0.0,
        waypoint_progress_clip: float = 0.35,
        w_waypoint_distance: float = 0.0,
        waypoint_distance_sigma: float = 0.45,
        w_heading_alignment: float = 0.0,
        w_path_lateral: float = 0.0,
        path_lateral_grace: float = 0.08,
        path_lateral_clip: float = 0.40,
        w_cumulative_backward: float = 0.0,
        w_forward_debt: float = 0.0,
        forward_debt_grace: float = 0.03,
        w_recovery_upright: float = 0.0,
        recovery_upright_threshold: float = 0.10,
        recovery_upright_sigma: float = 0.04,
        w_recovery_upright_progress: float = 0.0,
        recovery_upright_progress_clip: float = 1.0,
        w_recovery_foot_support: float = 0.0,
        lin_overspeed_margin: float = 0.02,
        terminal_penalty: float = 2.0,
        lin_vel_sigma: float = 0.25,
        yaw_rate_sigma: float = 0.25,
    ) -> None:
        self.alive_bonus = float(alive_bonus)
        self.w_lin_vel = float(w_lin_vel)
        self.w_lateral_vel = float(w_lateral_vel)
        self.w_yaw_rate = float(w_yaw_rate)
        self.w_upright = float(w_upright)
        self.w_ang = float(w_ang)
        self.w_height = float(w_height)
        self.w_dact = float(w_dact)
        self.w_joint_dev = float(w_joint_dev)
        self.w_nominal_action = float(w_nominal_action)
        self.w_contact = float(w_contact)
        self.w_phase_contact = float(w_phase_contact)
        self.w_swing_foot_clearance = float(w_swing_foot_clearance)
        self.swing_foot_clearance_target = max(float(swing_foot_clearance_target), 0.0)
        self.clearance_requires_air = bool(clearance_requires_air)
        self.w_swing_contact_penalty = float(w_swing_contact_penalty)
        self.w_double_support_penalty = float(w_double_support_penalty)
        self.w_foot_slip = float(w_foot_slip)
        self.w_foot_behind = float(w_foot_behind)
        self.foot_behind_threshold = float(foot_behind_threshold)
        self.w_double_foot_behind = float(w_double_foot_behind)
        self.double_foot_behind_threshold = float(double_foot_behind_threshold)
        self.w_footstep_forward = float(w_footstep_forward)
        self.w_footstep_backstep = float(w_footstep_backstep)
        self.w_footstep_lateral = float(w_footstep_lateral)
        self.w_landing_relx = float(w_landing_relx)
        self.landing_relx_target = float(landing_relx_target)
        self.landing_relx_scale = max(float(landing_relx_scale), 1e-6)
        self.footstep_lateral_target = max(float(footstep_lateral_target), 0.02)
        self.footstep_forward_clip = max(float(footstep_forward_clip), 1e-6)
        self.footstep_backstep_scale = max(float(footstep_backstep_scale), 1e-6)
        self.w_foot_width = float(w_foot_width)
        self.foot_width_target = max(float(foot_width_target), 0.02)
        self.foot_width_tolerance = max(float(foot_width_tolerance), 0.0)
        self.w_foot_crossing = float(w_foot_crossing)
        self.foot_crossing_margin = max(float(foot_crossing_margin), 0.0)
        self.w_no_flight = float(w_no_flight)
        self.w_body_forward_z = float(w_body_forward_z)
        self.body_forward_z_threshold = max(float(body_forward_z_threshold), 0.0)
        self.w_lin_overspeed = float(w_lin_overspeed)
        self.w_abs_overspeed = float(w_abs_overspeed)
        self.abs_overspeed_threshold = max(float(abs_overspeed_threshold), 0.0)
        self.w_backward_vel = float(w_backward_vel)
        self.w_lateral_position = float(w_lateral_position)
        self.lateral_position_grace = max(float(lateral_position_grace), 0.0)
        self.w_lateral_capture = float(w_lateral_capture)
        self.lateral_capture_margin = max(float(lateral_capture_margin), 0.0)
        self.lateral_capture_clip = max(float(lateral_capture_clip), 1e-6)
        self.lateral_capture_vy_threshold = max(float(lateral_capture_vy_threshold), 0.0)
        self.w_roll_bias = float(w_roll_bias)
        self.roll_bias_grace = max(float(roll_bias_grace), 0.0)
        self.roll_bias_decay = float(np.clip(roll_bias_decay, 0.0, 0.9999))
        self.w_yaw_position = float(w_yaw_position)
        self.yaw_position_grace = max(float(yaw_position_grace), 0.0)
        self.w_forward_progress = float(w_forward_progress)
        self.forward_progress_clip = max(float(forward_progress_clip), 1e-6)
        self.w_waypoint_progress = float(w_waypoint_progress)
        self.w_waypoint_distance = float(w_waypoint_distance)
        self.w_heading_alignment = float(w_heading_alignment)
        self.w_path_lateral = float(w_path_lateral)
        self.waypoint_progress_clip = max(float(waypoint_progress_clip), 1e-6)
        self.waypoint_distance_sigma = max(float(waypoint_distance_sigma), 1e-6)
        self.path_lateral_grace = max(float(path_lateral_grace), 0.0)
        self.path_lateral_clip = max(float(path_lateral_clip), 1e-6)
        self.w_cumulative_backward = float(w_cumulative_backward)
        self.w_forward_debt = float(w_forward_debt)
        self.forward_debt_grace = max(float(forward_debt_grace), 0.0)
        self.w_recovery_upright = float(w_recovery_upright)
        self.recovery_upright_threshold = max(float(recovery_upright_threshold), 0.0)
        self.recovery_upright_sigma = max(float(recovery_upright_sigma), 1e-6)
        self.w_recovery_upright_progress = float(w_recovery_upright_progress)
        self.recovery_upright_progress_clip = max(float(recovery_upright_progress_clip), 1e-6)
        self.w_recovery_foot_support = float(w_recovery_foot_support)
        self.lin_overspeed_margin = max(float(lin_overspeed_margin), 0.0)
        self.terminal_penalty = float(terminal_penalty)
        self.lin_vel_sigma = float(lin_vel_sigma)
        self.yaw_rate_sigma = float(yaw_rate_sigma)

        self._z0: float | None = None
        self._prev_action: np.ndarray | None = None
        self._prev_foot_xy: dict[str, np.ndarray] = {}
        self._prev_contact: dict[str, float] = {}
        self._last_landing_x: dict[str, float] = {}
        self._prev_base_x: float | None = None
        self._start_base_x: float | None = None
        self._start_base_y: float | None = None
        self._start_yaw: float | None = None
        self._roll_bias: float = 0.0
        self._prev_waypoint_dist: float | None = None
        self._prev_gravity_xy: float | None = None

    def reset(self, env: object) -> None:
        base_x = float(env.data.xpos[env._trunk_body_id][0])
        base_y = float(env.data.xpos[env._trunk_body_id][1])
        self._z0 = float(getattr(env, "_home_z", env.data.xpos[env._trunk_body_id][2]))
        self._prev_action = np.asarray(env.prev_action, dtype=np.float32).copy()
        self._prev_foot_xy = self._read_foot_xy(env)
        self._prev_contact = self._read_humanoid_contacts(env)
        self._last_landing_x = {
            side: float(pos[0]) for side, pos in self._prev_foot_xy.items()
        }
        self._prev_base_x = base_x
        self._start_base_x = base_x
        self._start_base_y = base_y
        self._start_yaw = self._read_base_yaw(env)
        self._roll_bias = 0.0
        projected_gravity = np.asarray(env._get_projected_gravity(), dtype=np.float32)
        self._prev_gravity_xy = (
            float(np.linalg.norm(projected_gravity[:2])) if projected_gravity.shape[0] >= 2 else 0.0
        )
        waypoint = self._waypoint_terms(env, dt=1.0, update_prev=False)
        self._prev_waypoint_dist = waypoint["distance"] if waypoint["enabled"] else None

    def compute(self, env: object, action: np.ndarray) -> tuple[float, dict[str, float]]:
        if self._z0 is None or self._prev_action is None:
            self.reset(env)

        cmd = np.asarray(env.command, dtype=np.float32)
        base_lin_vel = np.asarray(env.data.qvel[0:3], dtype=np.float32)
        base_ang_vel = np.asarray(env.data.qvel[3:6], dtype=np.float32)
        base_x = float(env.data.xpos[env._trunk_body_id][0])
        base_y = float(env.data.xpos[env._trunk_body_id][1])
        dt = max(float(getattr(env, "frame_skip", 1)) * float(env.model.opt.timestep), 1e-6)
        if self._prev_base_x is None:
            self._prev_base_x = base_x
        if self._start_base_x is None:
            self._start_base_x = base_x
        if self._start_base_y is None:
            self._start_base_y = base_y
        progress_vx = float((base_x - self._prev_base_x) / dt)
        r_forward_progress = float(
            np.clip(progress_vx, -self.forward_progress_clip, self.forward_progress_clip)
            / self.forward_progress_clip
        )
        elapsed = float(getattr(env, "_step_count", getattr(env, "step_count", 0))) * dt
        cumulative_progress = float(base_x - self._start_base_x)
        p_cumulative_backward = float(max(-cumulative_progress, 0.0) ** 2)
        expected_progress = max(float(cmd[0]), 0.0) * elapsed
        forward_debt = max(expected_progress - cumulative_progress - self.forward_debt_grace, 0.0)
        p_forward_debt = float(forward_debt**2)
        lateral_offset = abs(float(base_y - self._start_base_y))
        p_lateral_position = float(max(lateral_offset - self.lateral_position_grace, 0.0) ** 2)

        lin_err = float((base_lin_vel[0] - cmd[0]) ** 2 + (base_lin_vel[1] - cmd[1]) ** 2)
        lateral_err = float((base_lin_vel[1] - cmd[1]) ** 2)
        yaw_err = float((base_ang_vel[2] - cmd[2]) ** 2)
        overspeed = max(float(base_lin_vel[0] - cmd[0] - self.lin_overspeed_margin), 0.0)
        p_lin_overspeed = float(overspeed**2)
        abs_overspeed = max(float(base_lin_vel[0] - self.abs_overspeed_threshold), 0.0)
        p_abs_overspeed = float(abs_overspeed**2)
        backward_vel = max(float(-base_lin_vel[0]), 0.0)
        p_backward_vel = float(backward_vel**2)
        r_lin_vel = float(np.exp(-lin_err / max(self.lin_vel_sigma, 1e-6)))
        r_yaw_rate = float(np.exp(-yaw_err / max(self.yaw_rate_sigma, 1e-6)))
        if self._start_yaw is None:
            self._start_yaw = self._read_base_yaw(env)
        yaw_position_error = self._wrap_angle(self._read_base_yaw(env) - self._start_yaw)
        p_yaw_position = float(max(abs(yaw_position_error) - self.yaw_position_grace, 0.0) ** 2)
        waypoint = self._waypoint_terms(env, dt=dt, update_prev=True)
        r_waypoint_progress = waypoint["progress"]
        r_waypoint_distance = waypoint["distance_reward"]
        r_heading_alignment = waypoint["heading_alignment"]
        p_path_lateral = waypoint["path_lateral_penalty"]

        projected_gravity = env._get_projected_gravity()
        p_upright = float(projected_gravity[0] ** 2 + projected_gravity[1] ** 2)
        gravity_xy = float(np.sqrt(max(p_upright, 0.0)))
        if self._prev_gravity_xy is None:
            self._prev_gravity_xy = gravity_xy
        recovery_active = gravity_xy > self.recovery_upright_threshold
        r_recovery_upright = float(np.exp(-p_upright / self.recovery_upright_sigma)) if recovery_active else 0.0
        recovery_progress_speed = float((self._prev_gravity_xy - gravity_xy) / dt)
        r_recovery_upright_progress = (
            float(
                np.clip(
                    recovery_progress_speed,
                    -self.recovery_upright_progress_clip,
                    self.recovery_upright_progress_clip,
                )
                / self.recovery_upright_progress_clip
            )
            if recovery_active
            else 0.0
        )
        roll_like = float(projected_gravity[1])
        self._roll_bias = (
            self.roll_bias_decay * self._roll_bias
            + (1.0 - self.roll_bias_decay) * roll_like
        )
        p_roll_bias = float(max(abs(self._roll_bias) - self.roll_bias_grace, 0.0) ** 2)
        p_ang = float(np.sum(base_ang_vel ** 2))

        z = float(env.data.xpos[env._trunk_body_id][2])
        p_height = float((z - self._z0) ** 2)

        action = np.asarray(action, dtype=np.float32)
        dact = action - self._prev_action
        p_dact = float(np.mean(dact ** 2)) if dact.size > 0 else 0.0

        joint_pos = np.asarray(env.data.qpos[7:], dtype=np.float32)
        nominal = np.asarray(getattr(env, "nominal_action", env.home_joint_pos), dtype=np.float32)
        ctrl_high = np.asarray(getattr(env, "_ctrl_high", env.action_space.high), dtype=np.float32)
        ctrl_low = np.asarray(getattr(env, "_ctrl_low", env.action_space.low), dtype=np.float32)
        joint_range = np.maximum(ctrl_high - ctrl_low, 1e-6)
        n = min(joint_pos.shape[0], nominal.shape[0], joint_range.shape[0])
        joint_err = (joint_pos[:n] - nominal[:n]) / joint_range[:n]
        p_joint_dev = float(np.mean(joint_err ** 2)) if n > 0 else 0.0
        action_nominal_err = (action[:n] - nominal[:n]) / joint_range[:n]
        p_nominal_action = float(np.mean(action_nominal_err**2)) if n > 0 else 0.0

        contacts = env._get_foot_contacts()
        # Early walking setup: reward having at least one contact and avoid all-feet flight.
        contact_count = float(np.sum(contacts))
        r_contact = 1.0 if 1.0 <= contact_count <= 4.0 else 0.0
        p_no_flight = 1.0 if contact_count < 0.5 else 0.0
        (
            r_phase_contact,
            r_swing_clearance,
            p_swing_contact,
            p_double_support,
        ) = self._phase_foot_rewards(env, contacts)
        p_foot_slip = self._foot_slip_penalty(env, contacts)
        p_foot_behind = self._foot_behind_penalty(env)
        p_double_foot_behind = self._double_foot_behind_penalty(env)
        p_lateral_capture = self._lateral_capture_penalty(env, contacts)
        p_foot_width = self._foot_width_penalty(env)
        p_foot_crossing = self._foot_crossing_penalty(env)
        (
            r_footstep_forward,
            p_footstep_backstep,
            p_footstep_lateral,
            p_landing_relx,
        ) = self._footstep_progress_terms(
            env, contacts
        )
        p_body_forward_z = self._body_forward_z_penalty(env)

        is_fallen, _, _ = env._check_fall()
        p_terminal = 1.0 if is_fallen else 0.0

        reward = (
            self.alive_bonus
            + self.w_lin_vel * r_lin_vel
            + self.w_yaw_rate * r_yaw_rate
            + self.w_contact * r_contact
            + self.w_phase_contact * r_phase_contact
            + self.w_swing_foot_clearance * r_swing_clearance
            + self.w_forward_progress * r_forward_progress
            + self.w_waypoint_progress * r_waypoint_progress
            + self.w_waypoint_distance * r_waypoint_distance
            + self.w_heading_alignment * r_heading_alignment
            - self.w_lateral_vel * lateral_err
            - self.w_lateral_position * p_lateral_position
            - self.w_lateral_capture * p_lateral_capture
            - self.w_path_lateral * p_path_lateral
            - self.w_roll_bias * p_roll_bias
            - self.w_yaw_position * p_yaw_position
            - self.w_lin_overspeed * p_lin_overspeed
            - self.w_abs_overspeed * p_abs_overspeed
            - self.w_backward_vel * p_backward_vel
            - self.w_cumulative_backward * p_cumulative_backward
            - self.w_forward_debt * p_forward_debt
            - self.w_swing_contact_penalty * p_swing_contact
            - self.w_double_support_penalty * p_double_support
            - self.w_foot_slip * p_foot_slip
            - self.w_foot_behind * p_foot_behind
            - self.w_double_foot_behind * p_double_foot_behind
            + self.w_footstep_forward * r_footstep_forward
            - self.w_footstep_backstep * p_footstep_backstep
            - self.w_footstep_lateral * p_footstep_lateral
            - self.w_landing_relx * p_landing_relx
            - self.w_foot_width * p_foot_width
            - self.w_foot_crossing * p_foot_crossing
            + self.w_recovery_upright * r_recovery_upright
            + self.w_recovery_upright_progress * r_recovery_upright_progress
            - self.w_recovery_foot_support * (p_foot_width + p_foot_crossing)
            - self.w_no_flight * p_no_flight
            - self.w_body_forward_z * p_body_forward_z
            - self.w_upright * p_upright
            - self.w_ang * p_ang
            - self.w_height * p_height
            - self.w_dact * p_dact
            - self.w_joint_dev * p_joint_dev
            - self.w_nominal_action * p_nominal_action
            - self.terminal_penalty * p_terminal
        )

        self._prev_action = action.copy()
        self._prev_base_x = base_x
        self._prev_gravity_xy = gravity_xy
        penalty = (
            self.w_upright * p_upright
            + self.w_ang * p_ang
            + self.w_height * p_height
            + self.w_dact * p_dact
            + self.w_joint_dev * p_joint_dev
            + self.w_nominal_action * p_nominal_action
            + self.w_lateral_vel * lateral_err
            + self.w_lateral_position * p_lateral_position
            + self.w_lateral_capture * p_lateral_capture
            + self.w_path_lateral * p_path_lateral
            + self.w_roll_bias * p_roll_bias
            + self.w_yaw_position * p_yaw_position
            + self.w_lin_overspeed * p_lin_overspeed
            + self.w_abs_overspeed * p_abs_overspeed
            + self.w_backward_vel * p_backward_vel
            + self.w_cumulative_backward * p_cumulative_backward
            + self.w_forward_debt * p_forward_debt
            + self.w_swing_contact_penalty * p_swing_contact
            + self.w_double_support_penalty * p_double_support
            + self.w_foot_slip * p_foot_slip
            + self.w_foot_behind * p_foot_behind
            + self.w_double_foot_behind * p_double_foot_behind
            + self.w_footstep_backstep * p_footstep_backstep
            + self.w_footstep_lateral * p_footstep_lateral
            + self.w_landing_relx * p_landing_relx
            + self.w_foot_width * p_foot_width
            + self.w_foot_crossing * p_foot_crossing
            + self.w_recovery_foot_support * (p_foot_width + p_foot_crossing)
            + self.w_no_flight * p_no_flight
            + self.w_body_forward_z * p_body_forward_z
            + self.terminal_penalty * p_terminal
        )

        return reward, {
            "reward_total": reward,
            "reward_alive": self.alive_bonus,
            "reward_lin_vel": r_lin_vel,
            "reward_yaw_rate": r_yaw_rate,
            "reward_lateral_vel": -lateral_err,
            "reward_lateral_position": -p_lateral_position,
            "reward_lateral_capture": -p_lateral_capture,
            "reward_roll_bias": -p_roll_bias,
            "reward_yaw_position": -p_yaw_position,
            "reward_lin_overspeed": -p_lin_overspeed,
            "reward_abs_overspeed": -p_abs_overspeed,
            "reward_backward_vel": -p_backward_vel,
            "reward_cumulative_backward": -p_cumulative_backward,
            "reward_forward_debt": -p_forward_debt,
            "reward_swing_contact_penalty": -p_swing_contact,
            "reward_double_support_penalty": -p_double_support,
            "reward_foot_slip": -p_foot_slip,
            "reward_foot_behind": -p_foot_behind,
            "reward_double_foot_behind": -p_double_foot_behind,
            "reward_footstep_forward": r_footstep_forward,
            "reward_footstep_backstep": -p_footstep_backstep,
            "reward_footstep_lateral": -p_footstep_lateral,
            "reward_landing_relx": -p_landing_relx,
            "reward_foot_width": -p_foot_width,
            "reward_foot_crossing": -p_foot_crossing,
            "reward_recovery_upright": r_recovery_upright,
            "reward_recovery_upright_progress": r_recovery_upright_progress,
            "reward_recovery_foot_support": -(p_foot_width + p_foot_crossing),
            "reward_no_flight": -p_no_flight,
            "reward_body_forward_z": -p_body_forward_z,
            "reward_contact": r_contact,
            "reward_phase_contact": r_phase_contact,
            "reward_swing_foot_clearance": r_swing_clearance,
            "reward_forward_progress": r_forward_progress,
            "reward_waypoint_progress": r_waypoint_progress,
            "reward_waypoint_distance": r_waypoint_distance,
            "reward_heading_alignment": r_heading_alignment,
            "reward_path_lateral": -p_path_lateral,
            "reward_penalty_total": penalty,
            "reward_upright": -p_upright,
            "reward_ang": -p_ang,
            "reward_height": -p_height,
            "reward_dact": -p_dact,
            "reward_act": -p_dact,
            "reward_joint_dev": -p_joint_dev,
            "reward_nominal_action": -p_nominal_action,
            "reward_terminal": -self.terminal_penalty * p_terminal,
            "lin_vel_error": lin_err,
            "lin_overspeed_error": p_lin_overspeed,
            "abs_overspeed_error": p_abs_overspeed,
            "backward_vel_error": p_backward_vel,
            "cumulative_backward_error": p_cumulative_backward,
            "forward_debt_error": p_forward_debt,
            "cumulative_progress": cumulative_progress,
            "expected_progress": expected_progress,
            "swing_contact_error": p_swing_contact,
            "double_support_error": p_double_support,
            "foot_slip_error": p_foot_slip,
            "foot_behind_error": p_foot_behind,
            "double_foot_behind_error": p_double_foot_behind,
            "footstep_backstep_error": p_footstep_backstep,
            "footstep_lateral_error": p_footstep_lateral,
            "landing_relx_error": p_landing_relx,
            "foot_width_error": p_foot_width,
            "foot_crossing_error": p_foot_crossing,
            "recovery_active": float(recovery_active),
            "recovery_gravity_xy": gravity_xy,
            "recovery_gravity_xy_progress_speed": recovery_progress_speed,
            "footstep_forward_progress": r_footstep_forward,
            "no_flight_error": p_no_flight,
            "body_forward_z_error": p_body_forward_z,
            "lateral_vel_error": lateral_err,
            "lateral_position_error": p_lateral_position,
            "lateral_capture_error": p_lateral_capture,
            "roll_bias_error": p_roll_bias,
            "roll_bias": self._roll_bias,
            "yaw_position_error": p_yaw_position,
            "yaw_position": yaw_position_error,
            "lateral_offset": lateral_offset,
            "yaw_rate_error": yaw_err,
            "contact_count": contact_count,
            "progress_vx": progress_vx,
            "waypoint_enabled": float(waypoint["enabled"]),
            "waypoint_dist": waypoint["distance"],
            "waypoint_dist_delta": waypoint["distance_delta"],
            "waypoint_progress_speed": waypoint["progress_speed"],
            "heading_alignment": r_heading_alignment,
            "path_lateral_error": p_path_lateral,
        }

    def _read_base_yaw(self, env: object) -> float:
        try:
            quat = np.asarray(env.data.qpos[3:7], dtype=np.float64)
            if quat.shape[0] != 4:
                return 0.0
            w, x, y, z = quat
            siny_cosp = 2.0 * (w * z + x * y)
            cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
            return float(np.arctan2(siny_cosp, cosy_cosp))
        except Exception:
            return 0.0

    def _wrap_angle(self, angle: float) -> float:
        return float((angle + np.pi) % (2.0 * np.pi) - np.pi)

    def _waypoint_terms(self, env: object, *, dt: float, update_prev: bool) -> dict[str, float | bool]:
        empty = {
            "enabled": False,
            "distance": 0.0,
            "distance_delta": 0.0,
            "progress": 0.0,
            "progress_speed": 0.0,
            "distance_reward": 0.0,
            "heading_alignment": 0.0,
            "path_lateral_penalty": 0.0,
        }
        waypoints = np.asarray(getattr(env, "_waypoints", np.zeros((0, 2))), dtype=np.float32)
        if waypoints.ndim != 2 or waypoints.shape[0] == 0 or waypoints.shape[1] < 2:
            return empty

        try:
            base_xy = np.asarray(env.data.xpos[env._trunk_body_id][:2], dtype=np.float32)
        except Exception:
            return empty
        waypoint_idx = int(np.clip(getattr(env, "_waypoint_index", 0), 0, waypoints.shape[0] - 1))
        target = waypoints[waypoint_idx, :2].astype(np.float32)
        delta = target - base_xy
        dist = float(np.linalg.norm(delta))
        if dist > 1e-6:
            target_dir = delta / dist
        else:
            target_dir = np.zeros(2, dtype=np.float32)

        prev_dist = self._prev_waypoint_dist
        dist_delta = 0.0 if prev_dist is None else float(prev_dist - dist)
        progress_speed = dist_delta / max(float(dt), 1e-6)
        progress = float(
            np.clip(progress_speed, -self.waypoint_progress_clip, self.waypoint_progress_clip)
            / self.waypoint_progress_clip
        )
        distance_reward = float(np.exp(-(dist * dist) / self.waypoint_distance_sigma))

        yaw = self._read_base_yaw(env)
        target_yaw = float(np.arctan2(float(delta[1]), float(delta[0]))) if dist > 1e-6 else yaw
        heading_error = self._wrap_angle(target_yaw - yaw)
        heading_alignment = float(0.5 * (np.cos(heading_error) + 1.0))

        if waypoint_idx <= 0:
            segment_start = np.asarray(
                getattr(env, "_waypoint_start_xy", base_xy), dtype=np.float32
            )[:2]
        else:
            segment_start = waypoints[waypoint_idx - 1, :2].astype(np.float32)
        segment = target - segment_start
        seg_len = float(np.linalg.norm(segment))
        path_lateral_penalty = 0.0
        if seg_len > 1e-6:
            seg_dir = segment / seg_len
            rel = base_xy - segment_start
            cross_track = float(seg_dir[0] * rel[1] - seg_dir[1] * rel[0])
            outside = max(abs(cross_track) - self.path_lateral_grace, 0.0)
            clipped = min(outside, self.path_lateral_clip)
            path_lateral_penalty = float((clipped / self.path_lateral_clip) ** 2)

        if update_prev:
            self._prev_waypoint_dist = dist
        return {
            "enabled": True,
            "distance": dist,
            "distance_delta": dist_delta,
            "progress": progress,
            "progress_speed": progress_speed,
            "distance_reward": distance_reward,
            "heading_alignment": heading_alignment,
            "path_lateral_penalty": path_lateral_penalty,
        }

    def _read_foot_xy(self, env: object) -> dict[str, np.ndarray]:
        out: dict[str, np.ndarray] = {}
        if str(getattr(env, "robot_kind", "")) != "humanoid":
            return out
        for side in ("left", "right"):
            try:
                site_id = int(env.model.site(f"{side}_foot").id)
                out[side] = np.asarray(env.data.site_xpos[site_id][:2], dtype=np.float32).copy()
            except Exception:
                continue
        return out

    def _read_humanoid_contacts(self, env: object) -> dict[str, float]:
        contacts = np.asarray(env._get_foot_contacts(), dtype=np.float32)
        return {
            "left": float(contacts[0]) if contacts.shape[0] > 0 else 0.0,
            "right": float(contacts[1]) if contacts.shape[0] > 1 else 0.0,
        }

    def _footstep_progress_terms(self, env: object, contacts: np.ndarray) -> tuple[float, float, float, float]:
        if str(getattr(env, "robot_kind", "")) != "humanoid":
            return 0.0, 0.0, 0.0, 0.0
        current = self._read_foot_xy(env)
        if not current:
            return 0.0, 0.0, 0.0, 0.0
        try:
            trunk_x = float(env.data.xpos[env._trunk_body_id][0])
            trunk_y = float(env.data.xpos[env._trunk_body_id][1])
        except Exception:
            trunk_x = 0.0
            trunk_y = 0.0
        forward_terms: list[float] = []
        backstep_terms: list[float] = []
        lateral_terms: list[float] = []
        landing_relx_terms: list[float] = []
        for idx, side in enumerate(("left", "right")):
            contact = float(contacts[idx]) if contacts.shape[0] > idx else 0.0
            prev_contact = float(self._prev_contact.get(side, 0.0))
            if contact > 0.5 and prev_contact <= 0.5 and side in current:
                x = float(current[side][0])
                y = float(current[side][1])
                prev_x = self._last_landing_x.get(side)
                if prev_x is not None:
                    dx = x - float(prev_x)
                    forward_terms.append(
                        float(np.clip(dx, 0.0, self.footstep_forward_clip) / self.footstep_forward_clip)
                    )
                    backstep = max(-dx, 0.0) / self.footstep_backstep_scale
                    backstep_terms.append(float(backstep**2))
                rel_y = y - trunk_y
                rel_x = x - trunk_x
                relx_shortfall = max(self.landing_relx_target - rel_x, 0.0)
                landing_relx_terms.append(float((relx_shortfall / self.landing_relx_scale) ** 2))
                expected_sign = 1.0 if side == "left" else -1.0
                side_rel = expected_sign * rel_y
                too_wide = max(side_rel - self.footstep_lateral_target, 0.0)
                crossed = max(-side_rel, 0.0)
                lateral_terms.append(float(too_wide**2 + crossed**2))
                self._last_landing_x[side] = x
            self._prev_contact[side] = contact
        return (
            float(np.mean(forward_terms)) if forward_terms else 0.0,
            float(np.mean(backstep_terms)) if backstep_terms else 0.0,
            float(np.mean(lateral_terms)) if lateral_terms else 0.0,
            float(np.mean(landing_relx_terms)) if landing_relx_terms else 0.0,
        )

    def _foot_width_penalty(self, env: object) -> float:
        if str(getattr(env, "robot_kind", "")) != "humanoid":
            return 0.0
        try:
            left_id = int(env.model.site("left_foot").id)
            right_id = int(env.model.site("right_foot").id)
            width = abs(float(env.data.site_xpos[left_id][1] - env.data.site_xpos[right_id][1]))
        except Exception:
            return 0.0
        low = max(self.foot_width_target - self.foot_width_tolerance, 0.0)
        high = self.foot_width_target + self.foot_width_tolerance
        outside = max(low - width, width - high, 0.0)
        return float(outside**2)

    def _foot_crossing_penalty(self, env: object) -> float:
        if str(getattr(env, "robot_kind", "")) != "humanoid":
            return 0.0
        try:
            trunk_y = float(env.data.xpos[env._trunk_body_id][1])
            left_id = int(env.model.site("left_foot").id)
            right_id = int(env.model.site("right_foot").id)
            left_rel_y = float(env.data.site_xpos[left_id][1] - trunk_y)
            right_rel_y = float(env.data.site_xpos[right_id][1] - trunk_y)
        except Exception:
            return 0.0
        left_cross = max(self.foot_crossing_margin - left_rel_y, 0.0)
        right_cross = max(self.foot_crossing_margin + right_rel_y, 0.0)
        return float(0.5 * (left_cross * left_cross + right_cross * right_cross))

    def _lateral_capture_penalty(self, env: object, contacts: np.ndarray) -> float:
        if str(getattr(env, "robot_kind", "")) != "humanoid":
            return 0.0
        try:
            base_y = float(env.data.xpos[env._trunk_body_id][1])
            base_z = float(env.data.xpos[env._trunk_body_id][2])
            base_vy = float(env.data.qvel[1])
        except Exception:
            return 0.0

        stance_ys: list[float] = []
        for idx, side in enumerate(("left", "right")):
            contact = float(contacts[idx]) if contacts.shape[0] > idx else 0.0
            if contact <= 0.5:
                continue
            try:
                site_id = int(env.model.site(f"{side}_foot").id)
                stance_ys.append(float(env.data.site_xpos[site_id][1]))
            except Exception:
                continue
        if not stance_ys:
            return 0.0

        omega = float(np.sqrt(9.81 / max(base_z, 0.35)))
        capture_y = base_y + base_vy / max(omega, 1e-6)
        low = min(stance_ys) - self.lateral_capture_margin
        high = max(stance_ys) + self.lateral_capture_margin
        outside = max(low - capture_y, capture_y - high, 0.0)
        if abs(base_vy) <= self.lateral_capture_vy_threshold:
            return 0.0
        clipped = min(outside, self.lateral_capture_clip)
        return float((clipped / self.lateral_capture_clip) ** 2)

    def _foot_slip_penalty(self, env: object, contacts: np.ndarray) -> float:
        current = self._read_foot_xy(env)
        if not current:
            return 0.0
        terms: list[float] = []
        for idx, side in enumerate(("left", "right")):
            if contacts.shape[0] <= idx or float(contacts[idx]) <= 0.5:
                continue
            prev = self._prev_foot_xy.get(side)
            now = current.get(side)
            if prev is None or now is None:
                continue
            slip = float(np.linalg.norm(now - prev))
            terms.append(slip * slip)
        self._prev_foot_xy = current
        return float(np.mean(terms)) if terms else 0.0

    def _foot_behind_penalty(self, env: object) -> float:
        if str(getattr(env, "robot_kind", "")) != "humanoid":
            return 0.0
        try:
            trunk_x = float(env.data.xpos[env._trunk_body_id][0])
        except Exception:
            return 0.0
        terms: list[float] = []
        for side in ("left", "right"):
            try:
                site_id = int(env.model.site(f"{side}_foot").id)
                rel_x = float(env.data.site_xpos[site_id][0] - trunk_x)
            except Exception:
                continue
            behind = max(self.foot_behind_threshold - rel_x, 0.0)
            terms.append(behind * behind)
        return float(np.mean(terms)) if terms else 0.0

    def _double_foot_behind_penalty(self, env: object) -> float:
        if str(getattr(env, "robot_kind", "")) != "humanoid":
            return 0.0
        try:
            trunk_x = float(env.data.xpos[env._trunk_body_id][0])
        except Exception:
            return 0.0
        rel_xs: list[float] = []
        for side in ("left", "right"):
            try:
                site_id = int(env.model.site(f"{side}_foot").id)
                rel_xs.append(float(env.data.site_xpos[site_id][0] - trunk_x))
            except Exception:
                continue
        if len(rel_xs) < 2:
            return 0.0
        left_behind = max(self.double_foot_behind_threshold - rel_xs[0], 0.0)
        right_behind = max(self.double_foot_behind_threshold - rel_xs[1], 0.0)
        return float(left_behind * right_behind)

    def _body_forward_z_penalty(self, env: object) -> float:
        if str(getattr(env, "robot_kind", "")) != "humanoid":
            return 0.0
        try:
            body_xmat = np.asarray(env.data.xmat[env._trunk_body_id], dtype=np.float64).reshape(3, 3)
            body_forward_z = abs(float(body_xmat[:, 0][2]))
        except Exception:
            return 0.0
        excess = max(body_forward_z - self.body_forward_z_threshold, 0.0)
        return float(excess * excess)

    def _phase_foot_rewards(
        self, env: object, contacts: np.ndarray
    ) -> tuple[float, float, float, float]:
        if str(getattr(env, "robot_kind", "")) != "humanoid":
            return 0.0, 0.0, 0.0, 0.0
        if contacts.shape[0] < 2:
            return 0.0, 0.0, 0.0, 0.0

        phase = float(env.get_gait_phase()) if hasattr(env, "get_gait_phase") else 0.0
        phase_sin = float(np.sin(phase))
        # Match footstep_ik convention: sin(phase)>0 => left swing / right stance.
        left_swing = phase_sin > 0.0
        swing_side = "left" if left_swing else "right"
        stance_idx = 1 if left_swing else 0
        swing_idx = 0 if left_swing else 1

        swing_air = 1.0 - float(contacts[swing_idx])
        stance_contact = float(contacts[stance_idx])
        r_phase_contact = 0.5 * (swing_air + stance_contact)
        p_swing_contact = float(contacts[swing_idx])
        p_double_support = 1.0 if float(contacts[0]) > 0.5 and float(contacts[1]) > 0.5 else 0.0

        try:
            site_id = int(env.model.site(f"{swing_side}_foot").id)
            foot_z = float(env.data.site_xpos[site_id][2])
        except Exception:
            return r_phase_contact, 0.0, p_swing_contact, p_double_support
        target = max(self.swing_foot_clearance_target, 1e-6)
        # Saturating reward: no incentive to lift far above target.
        r_clearance = float(np.clip(foot_z / target, 0.0, 1.0))
        if self.clearance_requires_air:
            r_clearance *= swing_air
        return r_phase_contact, r_clearance, p_swing_contact, p_double_support
