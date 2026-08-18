"""KHR-3HV-paper-style residual walking task for the CHIRO Humanoid v2.

This is a clean-room implementation of the method described in
"Reinforcement Learning of Bipedal Walking Using a Simple Reference Motion".
It deliberately does not import Open Duck or Berkeley Humanoid code.
"""

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


# Actuator speeds of the physical robot, in JOINT_NAMES order (2026-08-18
# hardware feedback). Hip roll is a different unit from the other joints:
# 82 rpm / 34 N*m against 40 rpm / 24 N*m, so the "AK45-36 everywhere but the
# ankle roll" reading taken from the CAD mesh names does not hold for it.
# KHRConfig.max_speed_rpm deliberately does not default to this; see there.
HARDWARE_MAX_SPEED_RPM = (82.0, 40.0, 40.0, 40.0, 150.0) * 2


@dataclass(frozen=True)
class KHRConfig:
    sim_dt: float = 0.002
    control_dt: float = 0.04
    gait_period_steps: int = 50
    max_episode_steps: int = 500
    forward_speed: float = 0.15
    # Fixed-reference baseline reported in the KHR-3HV paper (SI units).
    sway_width: float = 0.020
    sway_offset: float = 0.005
    foot_height: float = 0.025
    foot_height_offset: float = 0.005
    action_filter: float = 0.2
    kp: float = 18.5
    kd: float = 0.009
    strict_velocity_reward: bool = False
    enhanced_collisions: bool = False
    velocity_window_steps: int = 1
    mirrored_roll_symmetry: bool = False
    alive_weight: float = 0.05
    fall_penalty: float = 0.0
    stride_half_length: float = 0.0
    torque_penalty_scale: float = 0.01
    swap_swing_legs: bool = False
    phased_stride: bool = False
    reset_to_reference: bool = False
    lateral_sign: float = 1.0
    swap_forward_legs: bool = False
    contact_schedule_reward_weight: float = 0.0
    # Actuator limits per joint, in JOINT_NAMES order.
    #
    # max_speed_rpm is NOT a limit on simulated joint speed. It only scales the
    # residual action, so it is part of the interface every trained policy was
    # fitted to: raising it makes an unchanged policy output move a joint
    # target further. It therefore stays at the values V1-V22 were trained
    # with, and HARDWARE_MAX_SPEED_RPM records what the robot actually has.
    # Start new training from the hardware values, not from these.
    max_speed_rpm: tuple[float, ...] = (40.0, 40.0, 40.0, 40.0, 150.0) * 2
    # torque_limit_n_m is a hard clip on every control sample, so it can track
    # the hardware directly: the V1-V22 rollouts peak at 9.4 N*m and never
    # reach a clip, which is why hip roll could be corrected to its measured
    # 34 N*m without changing any archived result.
    torque_limit_n_m: tuple[float, ...] = (34.0, 24.0, 24.0, 24.0, 7.0) * 2


class KHR3HVEnv(gym.Env[np.ndarray, np.ndarray]):
    """Ten-joint residual-policy environment matching the KHR paper."""

    metadata = {"render_modes": ["rgb_array"], "render_fps": 25}
    _reference_cache: dict[
        tuple[str, float, float, float, float, float, bool, bool, bool, float, bool],
        tuple[np.ndarray, np.ndarray, float],
    ] = {}

    def __init__(
        self,
        model_path: str | Path | None = None,
        render_mode: str | None = None,
        config: KHRConfig | None = None,
        reference_only: bool = False,
    ) -> None:
        super().__init__()
        root = Path(__file__).resolve().parents[1]
        self.model_path = Path(model_path or root / "models/urdf_f_v2/URDF_F_v2_footprint_contact.xml").resolve()
        self.config = config or KHRConfig()
        self.render_mode = render_mode
        self.reference_only = reference_only

        self.model = self._load_model()
        self.data = mujoco.MjData(self.model)
        self.model.opt.timestep = self.config.sim_dt
        self.model.opt.integrator = mujoco.mjtIntegrator.mjINT_IMPLICITFAST
        self.model.opt.iterations = 50
        self._disable_mesh_contacts()

        self.joint_ids = np.array([self.model.joint(name).id for name in JOINT_NAMES])
        self.qpos_ids = np.array([self.model.jnt_qposadr[j] for j in self.joint_ids])
        self.dof_ids = np.array([self.model.jnt_dofadr[j] for j in self.joint_ids])
        self.joint_ranges = self.model.jnt_range[self.joint_ids].copy()
        self.base_id = self.model.body("base_link").id
        self.left_foot_id = self.model.body("foot_L_1").id
        self.right_foot_id = self.model.body("foot_R_1").id
        self.frame_skip = int(round(self.config.control_dt / self.config.sim_dt))
        if self.frame_skip < 1:
            raise ValueError("control_dt must be at least sim_dt")

        cache_key = (
            str(self.model_path),
            self.config.sway_width,
            self.config.sway_offset,
            self.config.foot_height,
            self.config.foot_height_offset,
            self.config.stride_half_length,
            self.config.swap_swing_legs,
            self.config.phased_stride,
            self.config.reset_to_reference,
            self.config.lateral_sign,
            self.config.swap_forward_legs,
        )
        if cache_key not in self._reference_cache:
            self._reference_cache[cache_key] = self._build_reference_trajectory()
        reference, home_qpos, standing_height = self._reference_cache[cache_key]
        self.reference = reference.copy()
        self.home_qpos = home_qpos.copy()
        self.standing_height = float(standing_height)
        self.fall_height = 0.535 * self.standing_height

        # Actuator limits come from the config; see KHRConfig for the defaults.
        # max_speed only sets how far one control step may move a joint target,
        # it is not enforced on the simulated joint speed. torque_limit is a
        # hard clip applied to every control sample.
        self.max_speed = self._actuator_limit("max_speed_rpm") * 2.0 * np.pi / 60.0
        self.torque_limit = self._actuator_limit("torque_limit_n_m")
        self.action_scale = 0.5 * self.max_speed * self.config.control_dt

        self.action_space = spaces.Box(-1.0, 1.0, shape=(10,), dtype=np.float32)
        self.observation_space = spaces.Box(-np.inf, np.inf, shape=(82,), dtype=np.float32)
        self._history = np.zeros((5, 16), dtype=np.float64)
        self._filtered_action = np.zeros(10, dtype=np.float64)
        self._step_count = 0
        self._previous_linear_velocity = np.zeros(3, dtype=np.float64)
        self._velocity_history = np.zeros((self.config.velocity_window_steps, 5), dtype=np.float64)
        self._renderer: mujoco.Renderer | None = None
        self.render_camera_mode = "tracking"
        self.render_camera_lookat = np.array([0.0, 0.10, 0.18], dtype=np.float64)
        self.render_camera_distance = 1.25
        self.render_camera_azimuth = 135.0
        self.render_camera_elevation = -12.0

    @classmethod
    def base_config(cls) -> KHRConfig:
        """Config the task environments build on.

        The V4-V22 environments compose their own config from this one, so
        overriding it in a subclass is the single place to change actuator
        limits for a whole task family without touching the task settings.
        """
        return KHRConfig()

    def _actuator_limit(self, field: str) -> np.ndarray:
        values = np.array(getattr(self.config, field), dtype=np.float64)
        if values.shape != (len(JOINT_NAMES),):
            raise ValueError(f"{field} must have {len(JOINT_NAMES)} entries")
        if not np.all(values > 0.0):
            raise ValueError(f"{field} entries must be positive")
        return values

    def _write_actuator_limits(self, model: mujoco.MjModel) -> mujoco.MjModel:
        """Put the configured torque limits into the model itself.

        The control loop already clips every sample, but the MJCF ships generic
        +-100 N*m ranges that never bind and therefore misreport the hardware.
        Writing the real limits here keeps the model honest for anything that
        drives it outside this environment, and makes a configured limit above
        100 N*m actually take effect.
        """
        torque_limit = self._actuator_limit("torque_limit_n_m")
        for actuator_id in range(model.nu):
            if int(model.actuator_trntype[actuator_id]) != int(mujoco.mjtTrn.mjTRN_JOINT):
                continue
            joint_id = int(model.actuator_trnid[actuator_id, 0])
            limit = torque_limit[JOINT_NAMES.index(model.joint(joint_id).name)]
            gear = float(model.actuator_gear[actuator_id, 0])
            model.actuator_ctrlrange[actuator_id] = [-limit / gear, limit / gear]
            model.actuator_forcerange[actuator_id] = [-limit, limit]
            model.jnt_actfrcrange[joint_id] = [-limit, limit]
        return model

    def _load_model(self) -> mujoco.MjModel:
        if not self.config.enhanced_collisions:
            return self._write_actuator_limits(
                mujoco.MjModel.from_xml_path(str(self.model_path))
            )
        spec = mujoco.MjSpec.from_file(str(self.model_path))
        # Collision proxies use bit 2 and only the floor accepts bit 2. This
        # prevents overlapping coarse proxies from exerting internal forces.
        spec.geom("floor").conaffinity = 3
        collision_rgba = [0.25, 0.55, 0.95, 0.12]
        geom_type = mujoco.mjtGeom.mjGEOM_CAPSULE
        # Conservative primitive proxies. They preserve the explicit inertials
        # in the source MJCF and only make a fallen torso/leg hit the floor.
        spec.body("base_link").add_geom(
            name="v2_torso_collision",
            type=mujoco.mjtGeom.mjGEOM_BOX,
            pos=[0.0, -0.08, 0.30],
            size=[0.105, 0.085, 0.075],
            mass=0.0,
            friction=[0.8, 0.02, 0.001],
            contype=2,
            conaffinity=0,
            rgba=collision_rgba,
        )
        capsule_specs = (
            ("hipjoint1_L_1", "v2_left_hip_collision", [0.0, 0.0, 0.0, 0.057, -0.054, 0.0], 0.035),
            ("thigh_L_1", "v2_left_thigh_collision", [0.0, 0.0, 0.0, -0.014, 0.0, -0.13], 0.034),
            ("calf_L_1", "v2_left_calf_collision", [0.0, 0.0, 0.0, 0.0, 0.0, -0.20], 0.031),
            ("footJ_L_1", "v2_left_ankle_collision", [0.0, 0.0, 0.0, -0.043, -0.022, 0.0], 0.028),
            ("hipjoint1_R_1", "v2_right_hip_collision", [0.0, 0.0, 0.0, -0.057, -0.054, 0.0], 0.035),
            ("thigh_R_1", "v2_right_thigh_collision", [0.0, 0.0, 0.0, 0.014, 0.0, -0.13], 0.034),
            ("calf_R_1", "v2_right_calf_collision", [0.0, 0.0, 0.0, 0.0, 0.0, -0.20], 0.031),
            ("footJ_R_1", "v2_right_ankle_collision", [0.0, 0.0, 0.0, 0.043, -0.022, 0.0], 0.028),
        )
        for body_name, geom_name, fromto, radius in capsule_specs:
            spec.body(body_name).add_geom(
                name=geom_name,
                type=geom_type,
                fromto=fromto,
                size=[radius],
                mass=0.0,
                friction=[0.8, 0.02, 0.001],
                contype=2,
                conaffinity=0,
                rgba=collision_rgba,
            )
        return self._write_actuator_limits(spec.compile())

    def _disable_mesh_contacts(self) -> None:
        """Use CAD meshes for visuals and the eight sole boxes for contact.

        The source foot meshes extend about 46 mm below the intended sole and
        otherwise start deeply embedded in the floor.
        """
        mesh_type = int(mujoco.mjtGeom.mjGEOM_MESH)
        for geom_id in range(self.model.ngeom):
            if int(self.model.geom_type[geom_id]) == mesh_type:
                self.model.geom_contype[geom_id] = 0
                self.model.geom_conaffinity[geom_id] = 0

    @staticmethod
    def _home_joint_angles() -> np.ndarray:
        # Symmetric shallow crouch. The original KHR lowers its COM by 20 mm;
        # this robot's asymmetric hip limits leave lift clearance with 10 mm.
        alpha = 0.30
        beta = np.arcsin(-(0.13 / 0.20) * np.sin(alpha))
        left = np.array([0.0, -alpha, -(beta - alpha), -beta, 0.0])
        right = np.array([0.0, alpha, beta - alpha, -beta, 0.0])
        return np.concatenate((left, right))

    def _sole_point(self, foot_id: int, bottom: bool = False) -> np.ndarray:
        local_z = -0.06025269 if bottom else -0.04525269
        rotation = self.data.xmat[foot_id].reshape(3, 3)
        return self.data.xpos[foot_id] + rotation @ np.array([0.0, 0.0, local_z])

    def _foot_output(self, foot_id: int) -> np.ndarray:
        rotation = self.data.xmat[foot_id].reshape(3, 3)
        normal = rotation[:, 2]
        return np.concatenate((self._sole_point(foot_id), 0.05 * normal[:2]))

    def _solve_leg_ik(
        self,
        base_qpos: np.ndarray,
        seed: np.ndarray,
        leg_slice: slice,
        foot_id: int,
        target: np.ndarray,
    ) -> np.ndarray:
        """Thirty-step damped Gauss-Newton IK, as in the paper."""
        q = seed.copy()
        eps = 1.0e-5
        for _ in range(30):
            self.data.qpos[:] = base_qpos
            self.data.qpos[self.qpos_ids] = q
            mujoco.mj_forward(self.model, self.data)
            output = self._foot_output(foot_id)
            error = target - output
            if np.linalg.norm(error) < 1.0e-6:
                break
            jac = np.zeros((5, 5), dtype=np.float64)
            for column, joint_index in enumerate(range(leg_slice.start, leg_slice.stop)):
                perturbed = q.copy()
                perturbed[joint_index] += eps
                self.data.qpos[:] = base_qpos
                self.data.qpos[self.qpos_ids] = perturbed
                mujoco.mj_forward(self.model, self.data)
                jac[:, column] = (self._foot_output(foot_id) - output) / eps
            damping = 1.0e-5 * np.eye(5)
            update = jac.T @ np.linalg.solve(jac @ jac.T + damping, error)
            q[leg_slice] += 0.5 * update
            q[leg_slice] = np.clip(
                q[leg_slice], self.joint_ranges[leg_slice, 0], self.joint_ranges[leg_slice, 1]
            )
        return q

    def _build_reference_trajectory(self) -> tuple[np.ndarray, np.ndarray, float]:
        home = self._home_joint_angles()
        base_qpos = np.zeros(self.model.nq, dtype=np.float64)
        base_qpos[3] = 1.0
        base_qpos[self.qpos_ids] = home
        self.data.qpos[:] = base_qpos
        mujoco.mj_forward(self.model, self.data)
        ground = min(self._sole_point(self.left_foot_id, True)[2], self._sole_point(self.right_foot_id, True)[2])
        base_qpos[2] = -ground + 0.001
        self.data.qpos[:] = base_qpos
        mujoco.mj_forward(self.model, self.data)

        left_start = self._foot_output(self.left_foot_id)
        right_start = self._foot_output(self.right_foot_id)
        trajectory = np.zeros((self.config.gait_period_steps, 10), dtype=np.float64)
        seed = home.copy()
        for phase in range(self.config.gait_period_steps):
            angle = 2.0 * np.pi * phase / self.config.gait_period_steps
            sine = np.sin(angle)
            lateral = (
                self.config.lateral_sign
                * np.sign(sine)
                * min(abs(self.config.sway_width * sine) + self.config.sway_offset, self.config.sway_width)
            )
            opposite_sine = np.sin(angle + np.pi)
            if self.config.swap_swing_legs:
                left_wave, right_wave = opposite_sine, sine
            else:
                left_wave, right_wave = sine, opposite_sine
            if not self.config.swap_forward_legs:
                left_angle, right_angle = angle, angle + np.pi
                left_forward_wave, right_forward_wave = sine, opposite_sine
            else:
                left_angle, right_angle = angle + np.pi, angle
                left_forward_wave, right_forward_wave = opposite_sine, sine
            left_lift = max(0.0, self.config.foot_height * left_wave - self.config.foot_height_offset)
            right_lift = max(0.0, self.config.foot_height * right_wave - self.config.foot_height_offset)
            if self.config.phased_stride:
                left_forward = -self.config.stride_half_length * np.cos(left_angle)
                right_forward = -self.config.stride_half_length * np.cos(right_angle)
            else:
                left_forward = self.config.stride_half_length * left_forward_wave
                right_forward = self.config.stride_half_length * right_forward_wave
            left_target = left_start.copy()
            right_target = right_start.copy()
            left_target[:3] += np.array([lateral, left_forward, left_lift])
            right_target[:3] += np.array([lateral, right_forward, right_lift])
            seed = self._solve_leg_ik(base_qpos, seed, slice(0, 5), self.left_foot_id, left_target)
            seed = self._solve_leg_ik(base_qpos, seed, slice(5, 10), self.right_foot_id, right_target)
            trajectory[phase] = seed

        if self.config.reset_to_reference:
            base_qpos[self.qpos_ids] = trajectory[0]
        self.data.qpos[:] = base_qpos
        mujoco.mj_forward(self.model, self.data)
        standing_height = float(self.data.xipos[self.base_id, 2])
        return trajectory, base_qpos, standing_height

    @staticmethod
    def _roll_pitch(quaternion: np.ndarray) -> tuple[float, float]:
        w, x, y, z = quaternion
        roll = np.arctan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
        pitch = np.arcsin(np.clip(2.0 * (w * y - z * x), -1.0, 1.0))
        return float(roll), float(pitch)

    def _sensor_frame(self) -> np.ndarray:
        joint_angles = self.data.qpos[self.qpos_ids]
        world_velocity = self.data.qvel[:3].copy()
        world_acceleration = (world_velocity - self._previous_linear_velocity) / self.config.control_dt
        self._previous_linear_velocity = world_velocity
        rotation = self.data.xmat[self.base_id].reshape(3, 3)
        local_acceleration = rotation.T @ (world_acceleration - np.array([0.0, 0.0, -9.81]))
        local_gyro = rotation.T @ self.data.qvel[3:6]
        return np.clip(np.concatenate((joint_angles, local_acceleration, local_gyro)), -100.0, 100.0)

    def _observation(self, advance_history: bool = True) -> np.ndarray:
        if advance_history:
            self._history[:-1] = self._history[1:]
            self._history[-1] = self._sensor_frame()
        angle = 2.0 * np.pi * (self._step_count % self.config.gait_period_steps) / self.config.gait_period_steps
        phase = np.array([np.sin(angle), np.cos(angle)])
        return np.concatenate((self._history.ravel(), phase)).astype(np.float32)

    def _foot_contacts(self) -> tuple[bool, bool, float, float]:
        left_contact = right_contact = False
        left_force = right_force = 0.0
        contact_force = np.zeros(6, dtype=np.float64)
        for contact_index in range(self.data.ncon):
            contact = self.data.contact[contact_index]
            names = (self.model.geom(contact.geom1).name or "", self.model.geom(contact.geom2).name or "")
            mujoco.mj_contactForce(self.model, self.data, contact_index, contact_force)
            normal_force = max(float(contact_force[0]), 0.0)
            if any(name.startswith("foot_L_1_sole_pad") for name in names):
                left_contact = True
                left_force += normal_force
            if any(name.startswith("foot_R_1_sole_pad") for name in names):
                right_contact = True
                right_force += normal_force
        return left_contact, right_contact, left_force, right_force

    def reset(
        self,
        *,
        seed: int | None = None,
        options: dict[str, Any] | None = None,
    ) -> tuple[np.ndarray, dict[str, Any]]:
        super().reset(seed=seed)
        mujoco.mj_resetData(self.model, self.data)
        self.data.qpos[:] = self.home_qpos
        self.data.qvel[:] = 0.0
        self._filtered_action[:] = 0.0
        self._step_count = 0
        self._velocity_history[:] = 0.0
        mujoco.mj_forward(self.model, self.data)
        self._previous_linear_velocity = self.data.qvel[:3].copy()
        first = self._sensor_frame()
        self._history[:] = first
        return self._observation(advance_history=False), {
            "torso_height": float(self.data.xipos[self.base_id, 2]),
            "reference_phase": 0,
        }

    def step(self, action: np.ndarray) -> tuple[np.ndarray, float, bool, bool, dict[str, Any]]:
        action = np.asarray(action, dtype=np.float64)
        if self.reference_only:
            action = np.zeros(10, dtype=np.float64)
        action = np.clip(action, -1.0, 1.0)
        b = self.config.action_filter
        self._filtered_action = b * action + (1.0 - b) * self._filtered_action
        phase = self._step_count % self.config.gait_period_steps
        target = self.reference[phase] + self.action_scale * self._filtered_action
        target = np.clip(target, self.joint_ranges[:, 0], self.joint_ranges[:, 1])

        torque = np.zeros(10, dtype=np.float64)
        for _ in range(self.frame_skip):
            error = target - self.data.qpos[self.qpos_ids]
            torque = self.config.kp * error - self.config.kd * self.data.qvel[self.dof_ids]
            self.data.ctrl[:] = np.clip(torque, -self.torque_limit, self.torque_limit)
            mujoco.mj_step(self.model, self.data)

        self._step_count += 1
        roll, pitch = self._roll_pitch(self.data.qpos[3:7])
        orientation_reward = float(np.exp(-75.0 * (roll * roll + pitch * pitch)))
        rotation = self.data.xmat[self.base_id].reshape(3, 3)
        world_velocity = self.data.qvel[:3]
        local_velocity = rotation.T @ world_velocity
        forward_world, lateral_world = world_velocity[1], world_velocity[0]
        forward_local, lateral_local = local_velocity[1], local_velocity[0]
        yaw_rate = self.data.qvel[5]
        self._velocity_history[:-1] = self._velocity_history[1:]
        self._velocity_history[-1] = [forward_world, forward_local, lateral_world, lateral_local, yaw_rate]
        smoothed_velocity = self._velocity_history.mean(axis=0)
        reward_forward_world, reward_forward_local = smoothed_velocity[:2]
        reward_lateral_world, reward_lateral_local, reward_yaw_rate = smoothed_velocity[2:]
        velocity_loss = (
            15.0
            * (
                (self.config.forward_speed - reward_forward_world) ** 2
                + (self.config.forward_speed - reward_forward_local) ** 2
            )
            + 50.0 * (reward_lateral_world * reward_lateral_world + reward_lateral_local * reward_lateral_local)
            + 25.0 * reward_yaw_rate * reward_yaw_rate
        )
        velocity_tracking = float(np.exp(-velocity_loss))
        if self.config.strict_velocity_reward:
            mean_forward = 0.5 * (reward_forward_world + reward_forward_local)
            progress = float(np.clip(mean_forward / self.config.forward_speed, -1.0, 1.0))
            # No reward at rest, full tracking reward near commanded speed, and
            # an explicit penalty for moving backward.
            velocity_reward = velocity_tracking * max(progress, 0.0) + 0.25 * min(progress, 0.0)
        else:
            velocity_reward = velocity_tracking if forward_world > 0.0 and orientation_reward > 0.8 else 0.0
        q = self.data.qpos[self.qpos_ids]
        if self.config.mirrored_roll_symmetry:
            symmetry_error = (q[0] + q[5]) ** 2 + (q[4] + q[9]) ** 2
        else:
            symmetry_error = (q[0] - q[5]) ** 2 + (q[4] - q[9]) ** 2
        symmetry_reward = float(np.exp(-symmetry_error / 0.01))
        torso_height = float(self.data.xipos[self.base_id, 2])
        terminated = bool(torso_height < self.fall_height or not np.isfinite(self.data.qpos).all())
        alive_reward = 0.0 if terminated else 1.0
        torque_penalty = -self.config.torque_penalty_scale * float(np.abs(self.data.actuator_force).sum())
        left_contact, right_contact, left_force, right_force = self._foot_contacts()
        phase_sine = np.sin(2.0 * np.pi * phase / self.config.gait_period_steps)
        lift_threshold = self.config.foot_height_offset / self.config.foot_height
        swing_activation = max(0.0, (abs(phase_sine) - lift_threshold) / (1.0 - lift_threshold))
        positive_swing_is_right = self.config.swap_forward_legs
        if (phase_sine > 0.0) == positive_swing_is_right:
            swing_force = right_force
        else:
            swing_force = left_force
        contact_schedule_reward = swing_activation * float(np.exp(-swing_force / 5.0))
        reward = (
            0.10 * orientation_reward
            + 0.75 * velocity_reward
            + 0.10 * symmetry_reward
            + self.config.alive_weight * alive_reward
            + torque_penalty
            + (self.config.fall_penalty if terminated else 0.0)
            + self.config.contact_schedule_reward_weight * contact_schedule_reward
        )
        truncated = bool(self._step_count >= self.config.max_episode_steps)
        info = {
            "orientation_reward": orientation_reward,
            "velocity_reward": velocity_reward,
            "symmetry_reward": symmetry_reward,
            "alive_reward": alive_reward,
            "torque_penalty": torque_penalty,
            "forward_velocity": float(forward_world),
            "lateral_velocity": float(lateral_world),
            "torso_height": torso_height,
            "distance": float(self.data.qpos[1] - self.home_qpos[1]),
            "mean_episode_forward_velocity": float(
                (self.data.qpos[1] - self.home_qpos[1]) / (self._step_count * self.config.control_dt)
            ),
            "reference_phase": phase,
            "commanded_forward_speed": self.config.forward_speed,
            "left_foot_contact": left_contact,
            "right_foot_contact": right_contact,
            "left_foot_normal_force": left_force,
            "right_foot_normal_force": right_force,
            "contact_schedule_reward": contact_schedule_reward,
        }
        info["is_success"] = bool(
            truncated
            and not terminated
            and info["distance"] >= 2.4
            and 0.12 <= info["mean_episode_forward_velocity"] <= 0.18
        )
        return self._observation(), float(reward), terminated, truncated, info

    def configure_render_camera(
        self,
        *,
        mode: str = "tracking",
        lookat: tuple[float, float, float] | None = None,
        distance: float | None = None,
        azimuth: float | None = None,
        elevation: float | None = None,
    ) -> None:
        if mode not in {"tracking", "world_fixed"}:
            raise ValueError("camera mode must be 'tracking' or 'world_fixed'")
        self.render_camera_mode = mode
        if lookat is not None:
            self.render_camera_lookat[:] = lookat
        if distance is not None:
            self.render_camera_distance = float(distance)
        if azimuth is not None:
            self.render_camera_azimuth = float(azimuth)
        if elevation is not None:
            self.render_camera_elevation = float(elevation)

    def render(self) -> np.ndarray:
        if self._renderer is None:
            self._renderer = mujoco.Renderer(self.model, height=480, width=640)
        camera = mujoco.MjvCamera()
        camera.type = mujoco.mjtCamera.mjCAMERA_FREE
        if self.render_camera_mode == "tracking":
            camera.lookat[:] = self.data.xpos[self.base_id]
            camera.lookat[1] += 0.10
        else:
            camera.lookat[:] = self.render_camera_lookat
        camera.distance = self.render_camera_distance
        camera.azimuth = self.render_camera_azimuth
        camera.elevation = self.render_camera_elevation
        self._renderer.update_scene(self.data, camera=camera)
        return self._renderer.render()

    def close(self) -> None:
        if self._renderer is not None:
            self._renderer.close()
            self._renderer = None
