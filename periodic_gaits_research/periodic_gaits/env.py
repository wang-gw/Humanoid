"""Reference-free periodic-reward walking for the CHIRO Humanoid v2."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import gymnasium as gym
import mujoco
import numpy as np
from gymnasium import spaces


JOINT_NAMES = (
    "left_hip_roll",
    "left_hip_pitch",
    "left_knee_pitch",
    "left_ankle_pitch",
    "left_ankle_roll",
    "right_hip_roll",
    "right_hip_pitch",
    "right_knee_pitch",
    "right_ankle_pitch",
    "right_ankle_roll",
)


@dataclass(frozen=True)
class PeriodicGaitConfig:
    sim_dt: float = 0.002
    control_dt: float = 0.010
    episode_seconds: float = 8.0
    gait_frequency_hz: float = 0.8
    swing_ratio: float = 0.40
    left_phase_offset: float = 0.0
    right_phase_offset: float = 0.5
    phase_sharpness: float = 18.0
    forward_command_m_s: float = 0.08
    lateral_command_m_s: float = 0.0
    yaw_command_rad_s: float = 0.0
    action_filter: float = 0.35
    kp: float = 80.0
    kd: float = 0.32
    reset_joint_noise_rad: float = 0.01
    reset_joint_velocity_noise_rad_s: float = 0.02
    swing_clearance_target_m: float = 0.0
    swing_forward_speed_target_m_s: float = 0.0
    swing_knee_flexion_target_rad: float = 0.0
    swing_knee_forward_margin_m: float = 0.0
    natural_swing_reward_weight: float = 0.0
    action_saturation_reward_weight: float = 0.0
    backward_motion_reward_weight: float = 0.0


@dataclass(frozen=True)
class NaturalGaitConfig(PeriodicGaitConfig):
    """V2: task-space swing guidance without a joint or foot trajectory."""

    swing_clearance_target_m: float = 0.025
    swing_forward_speed_target_m_s: float = 0.0
    swing_knee_flexion_target_rad: float = 0.0
    # A swing knee should be ahead of the contralateral stance knee. This
    # describes the requested task-space shape without prescribing joint angles.
    swing_knee_forward_margin_m: float = 0.03
    natural_swing_reward_weight: float = 0.02
    action_saturation_reward_weight: float = 0.001
    backward_motion_reward_weight: float = 0.03


class PeriodicGaitEnv(gym.Env[np.ndarray, np.ndarray]):
    """100 Hz absolute-joint-target policy with deployable observations only."""

    metadata = {"render_modes": ["rgb_array"], "render_fps": 50}

    def __init__(
        self,
        model_path: str | Path | None = None,
        config: PeriodicGaitConfig | None = None,
        render_mode: str | None = None,
    ) -> None:
        super().__init__()
        root = Path(__file__).resolve().parents[2]
        self.model_path = Path(
            model_path
            or root
            / "humanoidv2/models/urdf_f_v2/URDF_F_v2_footprint_contact_symmetric_inertia.xml"
        ).resolve()
        self.config = config or PeriodicGaitConfig()
        self.render_mode = render_mode
        self.model = mujoco.MjModel.from_xml_path(str(self.model_path))
        self.data = mujoco.MjData(self.model)
        self.model.opt.timestep = self.config.sim_dt
        self.model.opt.integrator = mujoco.mjtIntegrator.mjINT_IMPLICITFAST
        self.frame_skip = int(round(self.config.control_dt / self.config.sim_dt))
        if self.frame_skip < 1 or not np.isclose(
            self.frame_skip * self.config.sim_dt, self.config.control_dt
        ):
            raise ValueError("control_dt must be an integer multiple of sim_dt")
        self.max_episode_steps = int(round(self.config.episode_seconds / self.config.control_dt))

        self._disable_mesh_contacts()
        self.joint_ids = np.array([self.model.joint(name).id for name in JOINT_NAMES])
        self.qpos_ids = self.model.jnt_qposadr[self.joint_ids].copy()
        self.dof_ids = self.model.jnt_dofadr[self.joint_ids].copy()
        self.joint_limits = self.model.jnt_range[self.joint_ids].copy()
        self.base_id = self.model.body("base_link").id
        self.left_foot_id = self.model.body("foot_L_1").id
        self.right_foot_id = self.model.body("foot_R_1").id
        self.left_knee_id = self.model.body("calf_L_1").id
        self.right_knee_id = self.model.body("calf_R_1").id

        self.neutral_q = self._neutral_joint_angles()
        # Safe learning envelope around the neutral crouch, not the full URDF range.
        self.action_half_range = np.array(
            [0.22, 0.38, 0.50, 0.38, 0.18] * 2, dtype=np.float64
        )
        self.action_low = np.maximum(
            self.neutral_q - self.action_half_range, self.joint_limits[:, 0]
        )
        self.action_high = np.minimum(
            self.neutral_q + self.action_half_range, self.joint_limits[:, 1]
        )
        self.action_center = 0.5 * (self.action_low + self.action_high)
        self.action_scale = 0.5 * (self.action_high - self.action_low)
        self.torque_limit = np.array([24.0, 24.0, 24.0, 24.0, 7.0] * 2)
        self.velocity_scale = np.array([4.2, 4.2, 4.2, 4.2, 15.7] * 2)

        self.action_space = spaces.Box(-1.0, 1.0, shape=(10,), dtype=np.float32)
        self.observation_space = spaces.Box(-np.inf, np.inf, shape=(44,), dtype=np.float32)
        self.previous_action = np.zeros(10, dtype=np.float64)
        self.filtered_target = self.neutral_q.copy()
        self.phase = 0.0
        self.step_count = 0
        self.initial_forward = 0.0
        self.standing_height = 0.0
        self.previous_foot_positions = np.zeros((2, 3), dtype=np.float64)
        self.phase_metrics: dict[str, list[float]] = {}
        self.episode_peak_torque = 0.0
        self._renderer: mujoco.Renderer | None = None

    def _disable_mesh_contacts(self) -> None:
        mesh_type = int(mujoco.mjtGeom.mjGEOM_MESH)
        for geom_id in range(self.model.ngeom):
            if int(self.model.geom_type[geom_id]) == mesh_type:
                self.model.geom_contype[geom_id] = 0
                self.model.geom_conaffinity[geom_id] = 0

    @staticmethod
    def _neutral_joint_angles() -> np.ndarray:
        # A standing pose is an action center, not a time-varying reference motion.
        alpha = 0.30
        beta = np.arcsin(-(0.13 / 0.20) * np.sin(alpha))
        left = np.array([0.0, -alpha, -(beta - alpha), -beta, 0.0])
        right = np.array([0.0, alpha, beta - alpha, -beta, 0.0])
        return np.concatenate((left, right))

    def _sole_position(self, body_id: int) -> np.ndarray:
        rotation = self.data.xmat[body_id].reshape(3, 3)
        return self.data.xpos[body_id] + rotation @ np.array([0.0, 0.0, -0.06025269])

    def _place_on_ground(self) -> None:
        self.data.qpos[:] = 0.0
        self.data.qpos[3] = 1.0
        self.data.qpos[self.qpos_ids] = self.neutral_q
        mujoco.mj_forward(self.model, self.data)
        ground = min(
            self._sole_position(self.left_foot_id)[2],
            self._sole_position(self.right_foot_id)[2],
        )
        self.data.qpos[2] = -ground + 0.001
        mujoco.mj_forward(self.model, self.data)

    def _projected_gravity(self) -> np.ndarray:
        rotation = self.data.xmat[self.base_id].reshape(3, 3)
        return rotation.T @ np.array([0.0, 0.0, -1.0])

    def _body_gyro(self) -> np.ndarray:
        rotation = self.data.xmat[self.base_id].reshape(3, 3)
        return rotation.T @ self.data.qvel[3:6]

    def _clock(self, offset: float) -> float:
        return float(np.sin(2.0 * np.pi * ((self.phase + offset) % 1.0)))

    def _observation(self) -> np.ndarray:
        q = self.data.qpos[self.qpos_ids]
        qd = self.data.qvel[self.dof_ids]
        command = np.array(
            [
                self.config.forward_command_m_s / 0.3,
                self.config.lateral_command_m_s / 0.2,
                self.config.yaw_command_rad_s / 1.0,
            ]
        )
        obs = np.concatenate(
            (
                self._projected_gravity(),
                0.25 * self._body_gyro(),
                (q - self.action_center) / np.maximum(self.action_scale, 1e-6),
                qd / self.velocity_scale,
                self.previous_action,
                command,
                [self._clock(self.config.left_phase_offset), self._clock(self.config.right_phase_offset)],
                [self.config.swing_ratio, 1.0 - self.config.swing_ratio],
                [self.config.gait_frequency_hz / 2.0],
            )
        )
        assert obs.shape == (44,)
        return np.clip(obs, -10.0, 10.0).astype(np.float32)

    def _swing_weight(self, offset: float) -> float:
        """Von-Mises-inspired smooth periodic interval expectation.

        The paper smooths random phase boundaries with von Mises CDFs. For this
        dependency-light V1, a circular logistic window provides the same useful
        boundary smoothing and stays in [0, 1].
        """
        local = (self.phase + offset) % 1.0
        center = 0.5 * self.config.swing_ratio
        cosine = np.cos(2.0 * np.pi * (local - center))
        boundary = np.cos(np.pi * self.config.swing_ratio)
        x = np.clip(self.config.phase_sharpness * (cosine - boundary), -60.0, 60.0)
        return float(1.0 / (1.0 + np.exp(-x)))

    def _foot_forces(self) -> tuple[float, float]:
        forces = [0.0, 0.0]
        contact_force = np.zeros(6, dtype=np.float64)
        for index in range(self.data.ncon):
            contact = self.data.contact[index]
            names = (
                self.model.geom(contact.geom1).name or "",
                self.model.geom(contact.geom2).name or "",
            )
            mujoco.mj_contactForce(self.model, self.data, index, contact_force)
            normal = max(float(contact_force[0]), 0.0)
            if any(name.startswith("foot_L_1_sole_pad") for name in names):
                forces[0] += normal
            if any(name.startswith("foot_R_1_sole_pad") for name in names):
                forces[1] += normal
        return float(forces[0]), float(forces[1])

    def _reward(
        self, action: np.ndarray, foot_velocities: np.ndarray, torque: np.ndarray
    ) -> tuple[float, dict[str, float]]:
        left_force, right_force = self._foot_forces()
        swing = np.array(
            [
                self._swing_weight(self.config.left_phase_offset),
                self._swing_weight(self.config.right_phase_offset),
            ]
        )
        stance = 1.0 - swing
        # Appendix Table II bounded kernels, adapted to this robot's force scale.
        force_cost = 1.0 - np.exp(-np.square(np.array([left_force, right_force])) / 100.0)
        foot_speeds = np.linalg.norm(foot_velocities, axis=1)
        speed_cost = 1.0 - np.exp(-2.0 * np.square(foot_speeds))
        periodic_cost = 0.5 * float(np.sum(swing * force_cost + stance * speed_cost))

        rotation = self.data.xmat[self.base_id].reshape(3, 3)
        local_velocity = rotation.T @ self.data.qvel[:3]
        # Anatomical forward for this model is world/local -Y.
        forward_velocity = -float(local_velocity[1])
        lateral_velocity = float(local_velocity[0])
        yaw_rate = float(self._body_gyro()[2])
        velocity_error = (
            abs(self.config.forward_command_m_s - forward_velocity)
            + abs(self.config.lateral_command_m_s - lateral_velocity)
            + 0.25 * abs(self.config.yaw_command_rad_s - yaw_rate)
        )
        command_cost = 1.0 - np.exp(-2.0 * velocity_error)
        upright_cost = 1.0 - np.exp(-3.0 * (1.0 - float(self.data.qpos[3]) ** 2))
        action_diff_cost = 1.0 - np.exp(-5.0 * float(np.linalg.norm(action - self.previous_action)))
        torque_cost = 1.0 - np.exp(-0.05 * float(np.linalg.norm(torque)))
        angular_cost = 1.0 - np.exp(-0.10 * float(np.linalg.norm(self._body_gyro())))

        foot_positions = np.array(
            [self._sole_position(self.left_foot_id), self._sole_position(self.right_foot_id)]
        )
        clearance_target = self.config.swing_clearance_target_m * swing
        clearance_error = np.maximum(clearance_target - foot_positions[:, 2], 0.0)
        clearance_cost = float(
            np.mean(np.square(clearance_error / max(self.config.swing_clearance_target_m, 1e-6)))
        ) if self.config.swing_clearance_target_m > 0.0 else 0.0

        # Anatomical forward is -Y. Only insufficient or backward swing motion is penalized.
        foot_forward_velocity = -foot_velocities[:, 1]
        foot_forward_velocity_relative_base = foot_forward_velocity - forward_velocity
        forward_deficit = np.maximum(
            self.config.swing_forward_speed_target_m_s - foot_forward_velocity_relative_base, 0.0
        )
        swing_forward_cost = float(
            np.sum(swing * (1.0 - np.exp(-4.0 * forward_deficit)))
            / max(np.sum(swing), 1e-6)
        ) if self.config.swing_forward_speed_target_m_s > 0.0 else 0.0

        q = self.data.qpos[self.qpos_ids]
        # Left/right numeric knee signs are mirrored in this MJCF.
        knee_flexion = np.array([q[2], -q[7]])
        knee_deficit = np.maximum(
            self.config.swing_knee_flexion_target_rad - knee_flexion, 0.0
        )
        knee_flexion_cost = float(
            np.sum(swing * np.square(knee_deficit / max(self.config.swing_knee_flexion_target_rad, 1e-6)))
            / max(np.sum(swing), 1e-6)
        ) if self.config.swing_knee_flexion_target_rad > 0.0 else 0.0

        knee_positions = np.array(
            [self.data.xpos[self.left_knee_id], self.data.xpos[self.right_knee_id]]
        )
        knee_local = (rotation.T @ (knee_positions - self.data.xpos[self.base_id]).T).T
        knee_forward = -knee_local[:, 1]
        # For each leg, compare its knee with the other leg. Only the active
        # swing interval contributes, so left/right roles exchange every half-cycle.
        knee_forward_advantage = np.array(
            [knee_forward[0] - knee_forward[1], knee_forward[1] - knee_forward[0]]
        )
        knee_forward_deficit = np.maximum(
            self.config.swing_knee_forward_margin_m - knee_forward_advantage, 0.0
        )
        knee_forward_cost = float(
            np.sum(swing * np.square(knee_forward_deficit / self.config.swing_knee_forward_margin_m))
            / max(np.sum(swing), 1e-6)
        ) if self.config.swing_knee_forward_margin_m > 0.0 else 0.0

        expected_clearance = self.config.swing_clearance_target_m * swing
        clearance_symmetry_cost = float(
            np.mean(np.square((foot_positions[:, 2] - expected_clearance) / max(self.config.swing_clearance_target_m, 1e-6)))
        ) if self.config.swing_clearance_target_m > 0.0 else 0.0
        clearance_symmetry_cost = min(clearance_symmetry_cost, 4.0) / 4.0
        natural_swing_cost = 0.50 * clearance_cost + 0.50 * knee_forward_cost
        saturation_cost = float(np.mean(np.square(np.maximum(np.abs(action) - 0.90, 0.0) / 0.10)))
        backward_motion_cost = float(np.clip(max(-forward_velocity, 0.0) / 0.08, 0.0, 1.0) ** 2)

        reward = 1.0 - (
            0.40 * periodic_cost
            + 0.30 * (0.65 * command_cost + 0.35 * upright_cost)
            + 0.10 * (0.45 * action_diff_cost + 0.35 * torque_cost + 0.20 * angular_cost)
            + self.config.natural_swing_reward_weight * natural_swing_cost
            + self.config.action_saturation_reward_weight * saturation_cost
            + self.config.backward_motion_reward_weight * backward_motion_cost
        )
        metrics = {
            "periodic_cost": periodic_cost,
            "command_cost": command_cost,
            "upright_cost": upright_cost,
            "action_diff_cost": action_diff_cost,
            "torque_cost": torque_cost,
            "angular_cost": angular_cost,
            "left_force_n": left_force,
            "right_force_n": right_force,
            "left_swing_weight": float(swing[0]),
            "right_swing_weight": float(swing[1]),
            "left_foot_speed_m_s": float(foot_speeds[0]),
            "right_foot_speed_m_s": float(foot_speeds[1]),
            "forward_velocity_m_s": forward_velocity,
            "clearance_cost": clearance_cost,
            "swing_forward_cost": swing_forward_cost,
            "knee_flexion_cost": knee_flexion_cost,
            "knee_forward_cost": knee_forward_cost,
            "clearance_symmetry_cost": clearance_symmetry_cost,
            "natural_swing_cost": natural_swing_cost,
            "action_saturation_cost": saturation_cost,
            "backward_motion_cost": backward_motion_cost,
            "left_knee_flexion_rad": float(knee_flexion[0]),
            "right_knee_flexion_rad": float(knee_flexion[1]),
            "left_knee_forward_m": float(knee_forward[0]),
            "right_knee_forward_m": float(knee_forward[1]),
            "left_foot_clearance_m": float(foot_positions[0, 2]),
            "right_foot_clearance_m": float(foot_positions[1, 2]),
            "left_foot_forward_velocity_m_s": float(foot_forward_velocity[0]),
            "right_foot_forward_velocity_m_s": float(foot_forward_velocity[1]),
            "left_foot_forward_velocity_relative_base_m_s": float(foot_forward_velocity_relative_base[0]),
            "right_foot_forward_velocity_relative_base_m_s": float(foot_forward_velocity_relative_base[1]),
        }
        return float(reward), metrics

    def reset(
        self, *, seed: int | None = None, options: dict[str, Any] | None = None
    ) -> tuple[np.ndarray, dict[str, Any]]:
        super().reset(seed=seed)
        mujoco.mj_resetData(self.model, self.data)
        self._place_on_ground()
        self.data.qpos[self.qpos_ids] = np.clip(
            self.neutral_q
            + self.np_random.normal(0.0, self.config.reset_joint_noise_rad, size=10),
            self.joint_limits[:, 0],
            self.joint_limits[:, 1],
        )
        self.data.qvel[self.dof_ids] = self.np_random.normal(
            0.0, self.config.reset_joint_velocity_noise_rad_s, size=10
        )
        mujoco.mj_forward(self.model, self.data)
        self.phase = float(self.np_random.uniform(0.0, 1.0))
        self.step_count = 0
        self.previous_action.fill(0.0)
        self.filtered_target = self.neutral_q.copy()
        self.initial_forward = -float(self.data.qpos[1])
        self.standing_height = float(self.data.xipos[self.base_id, 2])
        self.previous_foot_positions[:] = [
            self._sole_position(self.left_foot_id),
            self._sole_position(self.right_foot_id),
        ]
        self.phase_metrics = {
            "swing_force": [],
            "stance_force": [],
            "swing_speed": [],
            "stance_speed": [],
        }
        self.episode_peak_torque = 0.0
        return self._observation(), {"phase": self.phase}

    def step(self, action: np.ndarray):
        action = np.clip(np.asarray(action, dtype=np.float64), -1.0, 1.0)
        raw_target = self.action_center + self.action_scale * action
        b = self.config.action_filter
        self.filtered_target = (1.0 - b) * self.filtered_target + b * raw_target
        torque = np.zeros(10, dtype=np.float64)
        for _ in range(self.frame_skip):
            error = self.filtered_target - self.data.qpos[self.qpos_ids]
            torque = self.config.kp * error - self.config.kd * self.data.qvel[self.dof_ids]
            self.data.ctrl[:] = np.clip(torque, -self.torque_limit, self.torque_limit)
            mujoco.mj_step(self.model, self.data)

        foot_positions = np.array(
            [self._sole_position(self.left_foot_id), self._sole_position(self.right_foot_id)]
        )
        foot_velocities = (foot_positions - self.previous_foot_positions) / self.config.control_dt
        self.previous_foot_positions = foot_positions
        reward, metrics = self._reward(action, foot_velocities, torque)
        self.episode_peak_torque = max(
            self.episode_peak_torque, float(np.max(np.abs(self.data.actuator_force)))
        )
        for side in ("left", "right"):
            weight = metrics[f"{side}_swing_weight"]
            if weight > 0.8:
                self.phase_metrics["swing_force"].append(metrics[f"{side}_force_n"])
                self.phase_metrics["swing_speed"].append(metrics[f"{side}_foot_speed_m_s"])
            elif weight < 0.2:
                self.phase_metrics["stance_force"].append(metrics[f"{side}_force_n"])
                self.phase_metrics["stance_speed"].append(metrics[f"{side}_foot_speed_m_s"])
        self.previous_action = action.copy()
        self.phase = (self.phase + self.config.gait_frequency_hz * self.config.control_dt) % 1.0
        self.step_count += 1

        height = float(self.data.xipos[self.base_id, 2])
        projected_gravity = self._projected_gravity()
        finite = bool(np.isfinite(self.data.qpos).all() and np.isfinite(self.data.qvel).all())
        terminated = bool(
            not finite or height < 0.60 * self.standing_height or projected_gravity[2] > -0.45
        )
        if terminated:
            reward -= 5.0
        truncated = bool(self.step_count >= self.max_episode_steps)
        forward_distance = -float(self.data.qpos[1]) - self.initial_forward
        phase_means = {
            key: float(np.mean(values)) if values else 0.0
            for key, values in self.phase_metrics.items()
        }
        gait_pattern = bool(
            phase_means["swing_force"] < 0.5 * max(phase_means["stance_force"], 1e-6)
            and phase_means["swing_speed"] > 1.5 * max(phase_means["stance_speed"], 1e-6)
        )
        info: dict[str, Any] = {
            **metrics,
            **{f"mean_{key}": value for key, value in phase_means.items()},
            "phase": self.phase,
            "torso_height_m": height,
            "forward_distance_m": forward_distance,
            "max_abs_torque_nm": self.episode_peak_torque,
            "gait_pattern_satisfied": gait_pattern,
            "is_success": bool(
                truncated and not terminated and forward_distance > 0.20 and gait_pattern
            ),
        }
        return self._observation(), reward, terminated, truncated, info

    def render(self) -> np.ndarray:
        if self.render_mode != "rgb_array":
            raise RuntimeError("render_mode must be 'rgb_array'")
        if self._renderer is None:
            self._renderer = mujoco.Renderer(self.model, height=480, width=640)
        camera = mujoco.MjvCamera()
        camera.type = mujoco.mjtCamera.mjCAMERA_FREE
        camera.lookat[:] = [0.0, -0.15, 0.22]
        camera.distance = 1.6
        camera.azimuth = 160.0
        camera.elevation = -12.0
        self._renderer.update_scene(self.data, camera=camera)
        return self._renderer.render()

    def close(self) -> None:
        if self._renderer is not None:
            self._renderer.close()
            self._renderer = None
