from __future__ import annotations

import time
import math
from collections import deque
from typing import Any
from pathlib import Path

import gymnasium as gym
import numpy as np
from gymnasium import spaces
from policies import build_nominal_action
from policies.common import MODE_TO_STAGE, STAGE_TO_MODE
from rewards import RewardModule, build_reward


ROBOT_CONFIGS: dict[str, dict[str, Any]] = {
    "go1": {
        "xml_path": "robots/go1/wrapper.xml",
        "root_body": "trunk",
        "home_key": "home",
        "foot_geom_names": ("FR", "FL", "RR", "RL"),
        "kind": "quadruped",
    },
    "g1": {
        "xml_path": "robots/g1/wrapper.xml",
        "root_body": "pelvis",
        "home_key": "stand",
        "foot_geom_names": (
            "left_foot_hind_outer",
            "left_foot_hind_inner",
            "left_foot_toe_outer",
            "left_foot_toe_inner",
            "right_foot_hind_outer",
            "right_foot_hind_inner",
            "right_foot_toe_outer",
            "right_foot_toe_inner",
        ),
        "kind": "humanoid",
    },
}


class UnitreeEnv(gym.Env[np.ndarray, np.ndarray]):
    """MuJoCo-backed Unitree environment."""

    metadata = {"render_modes": ["human", "rgb_array"], "render_fps": 50}

    def __init__(
        self,
        render_mode: str | None = None,
        robot: str = "go1",
        xml_path: str | None = None,
        frame_skip: int = 5,
        sleep_sec: float = 0.0,
        max_episode_steps: int = 1000,
        image_width: int = 640,
        image_height: int = 480,
        render_camera_distance: float = 4.0,
        render_camera_azimuth: float = 135.0,
        render_camera_elevation: float = -18.0,
        render_camera_lookat_z: float = 0.75,
        stage: str = "stand",
        reward_name: str = "forward_velocity",
        reward_module: RewardModule | None = None,
        fall_height_threshold: float = 0.18,
        fall_height_ratio: float | None = None,
        fall_gravity_z_threshold: float = -0.5,
        soft_termination_prob: float = 1.0,
        soft_termination_max_grace_steps: int = 0,
        failure_state_path: str = "",
        failure_state_reset_prob: float = 0.0,
        max_forward_speed: float = 0.0,
        residual_action: bool = False,
        residual_scale: float = 1.0,
        residual_limit_ratio: float = 0.25,
        residual_action_mask: str = "all",
        residual_ramp_time: float = 0.0,
        nominal_ramp_time: float = 0.0,
        base_controller: str = "home",
        gait_frequency: float = 1.5,
        gait_phase_offset: float = 0.0,
        gait_thigh_amp: float = 0.25,
        gait_calf_amp: float = 0.35,
        gait_command_scale: float = 1.0,
        gait_command_reference_speed: float = 0.3,
        gait_command_max_scale: float = 1.0,
        gait_yaw_turn_scale: float = 0.0,
        gait_yaw_turn_direction: float = 1.0,
        gait_yaw_forward_compensation: float = 0.0,
        gait_yaw_hip_amp: float = 0.0,
        obs_noise_std: float = 0.0,
        action_noise_std: float = 0.0,
        action_delay_steps: int = 0,
        randomize_dynamics: bool = False,
        friction_scale_range: tuple[float, float] = (1.0, 1.0),
        trunk_mass_scale_range: tuple[float, float] = (1.0, 1.0),
        command_low: tuple[float, float, float] | None = None,
        command_high: tuple[float, float, float] | None = None,
        command_generator: str = "random",
        waypoint_count: int = 0,
        waypoint_first_distance: float = 1.0,
        waypoint_forward_range: tuple[float, float] = (0.8, 1.4),
        waypoint_lateral_range: tuple[float, float] = (-0.35, 0.35),
        waypoint_radius: float = 0.35,
        waypoint_target_speed: float = 0.20,
        waypoint_yaw_kp: float = 1.2,
        waypoint_yaw_rate_max: float = 0.35,
        waypoint_vx_min: float = 0.05,
        waypoint_vx_max: float = 0.25,
        reference_obs_path: str = "",
        reference_obs: bool = False,
        reference_obs_horizon: int = 2,
        reference_obs_dt: float = 0.10,
        reference_obs_scope: str = "lower_body",
        reference_random_start: bool = False,
        reference_start_time_range: tuple[float, float] | None = None,
        reference_start_time: float | None = None,
        reference_init_pose_on_reset: bool = False,
        reference_init_pose_blend: float | None = None,
        reference_error_obs: bool = False,
        foot_state_obs: bool = False,
        reference_time_scale: float = 1.0,
        reference_action_blend: float = 1.0,
    ) -> None:
        super().__init__()
        try:
            import mujoco
            import mujoco.viewer
        except ImportError as exc:
            raise ImportError(
                "UnitreeEnv requires 'mujoco'. Install with: pip install mujoco"
            ) from exc

        self._mujoco = mujoco
        self._viewer_mod = mujoco.viewer
        if robot not in ROBOT_CONFIGS:
            valid = ", ".join(sorted(ROBOT_CONFIGS))
            raise ValueError(f"Unknown robot '{robot}'. Available: {valid}")
        self.robot = robot
        self.robot_config = ROBOT_CONFIGS[robot]
        self.robot_kind = str(self.robot_config["kind"])
        self.render_mode = render_mode
        self.frame_skip = frame_skip
        self.sleep_sec = sleep_sec
        self.max_episode_steps = max_episode_steps
        self.image_width = image_width
        self.image_height = image_height
        self.render_camera_distance = float(render_camera_distance)
        self.render_camera_azimuth = float(render_camera_azimuth)
        self.render_camera_elevation = float(render_camera_elevation)
        self.render_camera_lookat_z = float(render_camera_lookat_z)
        self.stage = stage
        self.fall_height_threshold = float(fall_height_threshold)
        self.fall_height_ratio = float(fall_height_ratio) if fall_height_ratio is not None else None
        self.fall_gravity_z_threshold = float(fall_gravity_z_threshold)
        self.soft_termination_prob = float(np.clip(soft_termination_prob, 0.0, 1.0))
        self.soft_termination_max_grace_steps = max(int(soft_termination_max_grace_steps), 0)
        self.failure_state_path = str(failure_state_path)
        self.failure_state_reset_prob = float(np.clip(failure_state_reset_prob, 0.0, 1.0))
        self._failure_qpos = np.zeros((0, 0), dtype=np.float32)
        self._failure_qvel = np.zeros((0, 0), dtype=np.float32)
        self._failure_reference_time = np.zeros(0, dtype=np.float32)
        self._failure_command = np.zeros((0, 3), dtype=np.float32)
        self._soft_fall_grace_steps = 0
        self.max_forward_speed = max(float(max_forward_speed), 0.0)
        self.residual_action = bool(residual_action)
        self.residual_scale = float(residual_scale)
        self.residual_limit_ratio = float(residual_limit_ratio)
        self.residual_action_mask = str(residual_action_mask).strip().lower()
        self.residual_ramp_time = max(float(residual_ramp_time), 0.0)
        self.nominal_ramp_time = max(float(nominal_ramp_time), 0.0)
        self.base_controller = str(base_controller)
        self.gait_frequency = float(gait_frequency)
        self.gait_phase_offset = float(gait_phase_offset)
        self.gait_thigh_amp = float(gait_thigh_amp)
        self.gait_calf_amp = float(gait_calf_amp)
        self.gait_command_scale = float(gait_command_scale)
        self.gait_command_reference_speed = max(float(gait_command_reference_speed), 1e-6)
        self.gait_command_max_scale = max(float(gait_command_max_scale), 1e-6)
        self.gait_yaw_turn_scale = float(gait_yaw_turn_scale)
        self.gait_yaw_turn_direction = 1.0 if float(gait_yaw_turn_direction) >= 0.0 else -1.0
        self.gait_yaw_forward_compensation = float(gait_yaw_forward_compensation)
        self.gait_yaw_hip_amp = float(gait_yaw_hip_amp)
        self.obs_noise_std = float(obs_noise_std)
        self.action_noise_std = float(action_noise_std)
        self.action_delay_steps = max(0, int(action_delay_steps))
        self.randomize_dynamics = bool(randomize_dynamics)
        self.friction_scale_range = tuple(float(v) for v in friction_scale_range)
        self.trunk_mass_scale_range = tuple(float(v) for v in trunk_mass_scale_range)
        self.command_generator = str(command_generator).strip().lower()
        self.waypoint_count = max(int(waypoint_count), 0)
        self.waypoint_first_distance = max(float(waypoint_first_distance), 0.05)
        self.waypoint_forward_range = tuple(float(v) for v in waypoint_forward_range)
        self.waypoint_lateral_range = tuple(float(v) for v in waypoint_lateral_range)
        self.waypoint_radius = max(float(waypoint_radius), 0.05)
        self.waypoint_target_speed = max(float(waypoint_target_speed), 0.0)
        self.waypoint_yaw_kp = max(float(waypoint_yaw_kp), 0.0)
        self.waypoint_yaw_rate_max = max(float(waypoint_yaw_rate_max), 0.0)
        self.waypoint_vx_min = max(float(waypoint_vx_min), 0.0)
        self.waypoint_vx_max = max(float(waypoint_vx_max), self.waypoint_vx_min)
        self._waypoints = np.zeros((0, 2), dtype=np.float32)
        self._waypoint_index = 0
        self._waypoint_start_xy = np.zeros(2, dtype=np.float32)
        self._waypoint_start_yaw = 0.0
        self.reference_obs = bool(reference_obs)
        self.reference_obs_path = str(reference_obs_path)
        self.reference_obs_horizon = max(1, int(reference_obs_horizon))
        self.reference_obs_dt = max(float(reference_obs_dt), 0.0)
        self.reference_obs_scope = str(reference_obs_scope).strip().lower()
        self.reference_random_start = bool(reference_random_start)
        self.reference_start_time_range = (
            tuple(float(v) for v in reference_start_time_range)
            if reference_start_time_range is not None
            else None
        )
        self.reference_start_time = float(reference_start_time) if reference_start_time is not None else None
        self.reference_init_pose_on_reset = bool(reference_init_pose_on_reset)
        self.reference_init_pose_blend = (
            None if reference_init_pose_blend is None else float(np.clip(reference_init_pose_blend, 0.0, 1.0))
        )
        self.reference_error_obs = bool(reference_error_obs)
        self.foot_state_obs = bool(foot_state_obs)
        ref_time_scale = float(reference_time_scale)
        if abs(ref_time_scale) < 1e-6:
            ref_time_scale = 1e-6
        self.reference_time_scale = ref_time_scale
        self.reference_action_blend = float(np.clip(reference_action_blend, 0.0, 1.0))
        self._reference_time_offset = 0.0

        resolved_xml_path = xml_path or str(self.robot_config["xml_path"])
        xml_abs_path = Path(resolved_xml_path)
        if not xml_abs_path.is_absolute():
            xml_abs_path = Path(__file__).resolve().parent / xml_abs_path
        xml_abs_path = xml_abs_path.resolve()
        if not xml_abs_path.exists():
            raise FileNotFoundError(f"XML not found: {xml_abs_path}")

        self.model = self._mujoco.MjModel.from_xml_path(str(xml_abs_path))
        self.data = self._mujoco.MjData(self.model)
        self._trunk_body_id = self._resolve_trunk_body_id(str(self.robot_config["root_body"]))
        self._foot_names = tuple(str(v) for v in self.robot_config["foot_geom_names"])
        self._foot_geom_ids = self._resolve_foot_geom_ids(self._foot_names)
        self._default_geom_friction = np.asarray(self.model.geom_friction, dtype=np.float64).copy()
        self._default_body_mass = np.asarray(self.model.body_mass, dtype=np.float64).copy()
        self._home_key_id = self._resolve_key_id(str(self.robot_config["home_key"]))
        self.home_qpos = self._get_home_qpos()
        self._home_z = float(self.home_qpos[2]) if self.home_qpos.size > 2 else 0.0
        self.home_joint_pos = self.home_qpos[7:].copy().astype(np.float32)
        self._reference_fps = 0.0
        self._reference_joint_pos = np.zeros((0, int(self.model.nu)), dtype=np.float32)
        self._reference_contact_labels = np.zeros((0, 2), dtype=np.float32)
        self._reference_root_pos = np.zeros((0, 3), dtype=np.float32)
        self._reference_root_quat = np.zeros((0, 4), dtype=np.float32)
        self._reference_cycle_start_frame = 0
        self._load_failure_states()

        if self.model.nu > 0:
            self._ctrl_low = self.model.actuator_ctrlrange[:, 0].astype(np.float32)
            self._ctrl_high = self.model.actuator_ctrlrange[:, 1].astype(np.float32)
        else:
            self._ctrl_low = np.zeros(0, dtype=np.float32)
            self._ctrl_high = np.zeros(0, dtype=np.float32)
        self._load_reference_motion()

        if self.residual_action and self.model.nu > 0:
            delta_lim = self.residual_limit_ratio * (self._ctrl_high - self._ctrl_low)
            delta_lim = np.maximum(delta_lim, 1e-6).astype(np.float32)
            low = -delta_lim
            high = delta_lim
        else:
            low = self._ctrl_low.copy()
            high = self._ctrl_high.copy()

        self.action_space = spaces.Box(low=low, high=high, dtype=np.float32)
        self._residual_action_mask = self._build_residual_action_mask()

        self.command = np.zeros(3, dtype=np.float32)
        self.prev_action = np.zeros(self.model.nu, dtype=np.float32)
        self.nominal_action = np.zeros(self.model.nu, dtype=np.float32)
        self.mode = self._stage_to_mode(self.stage)
        self._delayed_actions: deque[np.ndarray] = deque(maxlen=max(self.action_delay_steps + 1, 1))
        self._stage_command_ranges: dict[str, tuple[np.ndarray, np.ndarray]] = {
            "stand": (
                np.array([-0.1, -0.1, -0.3], dtype=np.float32),
                np.array([0.1, 0.1, 0.3], dtype=np.float32),
            ),
            "walk": (
                np.array([0.0, -0.3, -0.5], dtype=np.float32),
                np.array([0.5, 0.3, 0.5], dtype=np.float32),
            ),
        }
        if command_low is not None or command_high is not None:
            default_low, default_high = self._stage_command_ranges[self.stage]
            low = default_low if command_low is None else np.asarray(command_low, dtype=np.float32)
            high = default_high if command_high is None else np.asarray(command_high, dtype=np.float32)
            low = low.reshape(3)
            high = high.reshape(3)
            if np.any(low > high):
                raise ValueError(f"command_low must be <= command_high, got {low} > {high}")
            self._stage_command_ranges[self.stage] = (low.copy(), high.copy())

        obs_dim = int(
            3  # base lin vel
            + 3  # base ang vel
            + 3  # projected gravity
            + (self.model.nq - 7)  # joint pos
            + (self.model.nv - 6)  # joint vel
            + 3  # command
            + self.model.nu  # prev action
            + 2  # gait phase sin/cos
            + 4  # foot contacts, padded to fixed size
            + (6 if self.foot_state_obs else 0)  # humanoid left/right foot rel xyz
            + (self.model.nu * self.reference_obs_horizon if self.reference_obs else 0)
            + (self.model.nu if self.reference_error_obs else 0)
        )
        self.observation_space = spaces.Box(
            low=-np.inf, high=np.inf, shape=(obs_dim,), dtype=np.float32
        )

        self._step_count = 0
        self._viewer = None
        self._renderer = None
        self._render_camera = None
        self._render_error: str | None = None
        self._reward_module: RewardModule = (
            reward_module if reward_module is not None else build_reward(reward_name)
        )

        # Initialize command with current stage limits.
        self.set_stage(self.stage, resample_command=True)

    def _resolve_trunk_body_id(self, body_name: str) -> int:
        try:
            return int(self.model.body(body_name).id)
        except Exception:
            pass

        try:
            body_id = int(
                self._mujoco.mj_name2id(
                    self.model, self._mujoco.mjtObj.mjOBJ_BODY, body_name
                )
            )
            if body_id >= 0:
                return body_id
        except Exception:
            pass

        for body_id in range(int(self.model.nbody)):
            try:
                name = self._mujoco.mj_id2name(
                    self.model, self._mujoco.mjtObj.mjOBJ_BODY, body_id
                )
            except Exception:
                name = None
            if name == body_name:
                return body_id

        raise ValueError(f"Body '{body_name}' not found in model.")

    def _resolve_foot_geom_ids(self, geom_names: tuple[str, ...]) -> dict[str, int]:
        out: dict[str, int] = {}
        for geom_name in geom_names:
            geom_id = -1
            try:
                geom_id = int(
                    self._mujoco.mj_name2id(
                        self.model, self._mujoco.mjtObj.mjOBJ_GEOM, geom_name
                    )
                )
            except Exception:
                geom_id = -1
            if geom_id >= 0:
                out[geom_name] = geom_id
        return out

    def _resolve_key_id(self, key_name: str) -> int:
        try:
            key_id = int(
                self._mujoco.mj_name2id(
                    self.model, self._mujoco.mjtObj.mjOBJ_KEY, key_name
                )
            )
            if key_id >= 0:
                return key_id
        except Exception:
            pass
        return -1

    def _get_home_qpos(self) -> np.ndarray:
        if self._home_key_id >= 0 and self.model.nkey > self._home_key_id:
            return np.asarray(self.model.key_qpos[self._home_key_id], dtype=np.float32).copy()

        qpos = np.zeros(self.model.nq, dtype=np.float32)
        qpos[3] = 1.0
        return qpos

    def _include_reference_obs_joint(self, name: str) -> bool:
        if self.reference_obs_scope in {"all", ""}:
            return True
        if self.reference_obs_scope in {"lower", "lower_body", "legs_waist"}:
            return (
                name.startswith("left_hip_")
                or name.startswith("left_knee")
                or name.startswith("left_ankle")
                or name.startswith("right_hip_")
                or name.startswith("right_knee")
                or name.startswith("right_ankle")
                or name.startswith("waist_")
            )
        if self.reference_obs_scope in {"legs_only", "legs"}:
            return (
                name.startswith("left_hip_")
                or name.startswith("left_knee")
                or name.startswith("left_ankle")
                or name.startswith("right_hip_")
                or name.startswith("right_knee")
                or name.startswith("right_ankle")
            )
        raise ValueError(
            f"Unknown reference_obs_scope={self.reference_obs_scope!r}. "
            "Use all, lower_body, or legs_only."
        )

    def _load_reference_motion(self) -> None:
        if (
            not self.reference_obs
            and not self.reference_error_obs
            and not self.base_controller.startswith("reference_")
        ):
            return
        if not self.reference_obs_path.strip():
            raise ValueError("reference motion features require reference_obs_path")

        path = Path(self.reference_obs_path)
        if not path.exists():
            raise FileNotFoundError(f"Reference obs motion not found: {path}")

        data = np.load(path, allow_pickle=False)
        fps = float(np.asarray(data["fps"]).reshape(()))
        joint_pos = np.asarray(data["joint_pos"], dtype=np.float32)
        if "contact_labels" in data:
            contacts = np.asarray(data["contact_labels"], dtype=np.float32)
            if contacts.ndim == 2 and contacts.shape[1] >= 2:
                self._reference_contact_labels = contacts[:, :2].copy()
        if "root_pos" in data:
            root_pos = np.asarray(data["root_pos"], dtype=np.float32)
            if root_pos.ndim == 2 and root_pos.shape[1] >= 3:
                self._reference_root_pos = root_pos[:, :3].copy()
        if "root_quat" in data:
            root_quat = np.asarray(data["root_quat"], dtype=np.float32)
            if root_quat.ndim == 2 and root_quat.shape[1] >= 4:
                self._reference_root_quat = root_quat[:, :4].copy()
        if "cycle_start_frame" in data:
            cycle_start = int(np.asarray(data["cycle_start_frame"]).reshape(-1)[0])
            self._reference_cycle_start_frame = max(cycle_start, 0)
        joint_names = [str(v) for v in np.asarray(data["joint_names"]).tolist()]
        ref_index = {name: i for i, name in enumerate(joint_names)}

        reference = np.tile(self.home_joint_pos.reshape(1, -1), (joint_pos.shape[0], 1))
        for act_i in range(int(self.model.nu)):
            name = str(self.model.actuator(act_i).name)
            ref_i = ref_index.get(name)
            if ref_i is None or not self._include_reference_obs_joint(name):
                continue
            reference[:, act_i] = joint_pos[:, ref_i]

        self._reference_fps = fps
        self._reference_joint_pos = reference.astype(np.float32)

    def _load_failure_states(self) -> None:
        if not self.failure_state_path.strip():
            return
        path = Path(self.failure_state_path)
        if not path.exists():
            raise FileNotFoundError(f"Failure-state dataset not found: {path}")
        data = np.load(path, allow_pickle=False)
        qpos = np.asarray(data["qpos"], dtype=np.float32)
        qvel = np.asarray(data["qvel"], dtype=np.float32)
        if qpos.ndim != 2 or qpos.shape[1] != int(self.model.nq):
            raise ValueError(f"failure qpos shape must be (*, {int(self.model.nq)}), got {qpos.shape}")
        if qvel.ndim != 2 or qvel.shape[1] != int(self.model.nv):
            raise ValueError(f"failure qvel shape must be (*, {int(self.model.nv)}), got {qvel.shape}")
        if qpos.shape[0] != qvel.shape[0]:
            raise ValueError("failure qpos/qvel sample counts differ")
        self._failure_qpos = qpos.copy()
        self._failure_qvel = qvel.copy()
        if "reference_time" in data:
            ref_time = np.asarray(data["reference_time"], dtype=np.float32).reshape(-1)
            self._failure_reference_time = ref_time[: qpos.shape[0]].copy()
        else:
            self._failure_reference_time = np.zeros(qpos.shape[0], dtype=np.float32)
        if "command" in data:
            command = np.asarray(data["command"], dtype=np.float32)
            if command.ndim == 2 and command.shape[1] >= 3:
                self._failure_command = command[: qpos.shape[0], :3].copy()
            else:
                self._failure_command = np.zeros((qpos.shape[0], 3), dtype=np.float32)
        else:
            self._failure_command = np.zeros((qpos.shape[0], 3), dtype=np.float32)

    def _should_reset_to_failure_state(self) -> bool:
        if self.failure_state_reset_prob <= 0.0 or self._failure_qpos.shape[0] == 0:
            return False
        rng = getattr(self, "np_random", None)
        value = float(rng.random()) if rng is not None else float(np.random.random())
        return value < self.failure_state_reset_prob

    def _apply_failure_state_reset(self) -> bool:
        if self._failure_qpos.shape[0] == 0:
            return False
        rng = getattr(self, "np_random", None)
        if rng is not None:
            idx = int(rng.integers(0, self._failure_qpos.shape[0]))
        else:
            idx = int(np.random.randint(0, self._failure_qpos.shape[0]))
        self.data.qpos[:] = self._failure_qpos[idx].astype(np.float64)
        self.data.qvel[:] = self._failure_qvel[idx].astype(np.float64)
        if self._failure_reference_time.shape[0] > idx:
            ref_time = float(self._failure_reference_time[idx])
            self._reference_time_offset = ref_time - self.reference_time_scale * float(self.data.time)
        if self._failure_command.shape[0] > idx:
            self.command = self._failure_command[idx].copy().astype(np.float32)
        self._mujoco.mj_forward(self.model, self.data)
        return True

    def get_reference_frame_index(self, offset: float = 0.0) -> int:
        if self._reference_fps <= 0.0:
            return 0
        t = self.get_reference_time() + float(offset)
        n_frames = int(max(self._reference_joint_pos.shape[0], self._reference_contact_labels.shape[0]))
        if n_frames <= 0:
            return 0
        raw_frame = int(np.floor(t * self._reference_fps))
        cycle_start = int(np.clip(self._reference_cycle_start_frame, 0, max(n_frames - 1, 0)))
        if cycle_start > 0 and n_frames - cycle_start > 1 and raw_frame >= cycle_start:
            return cycle_start + ((raw_frame - cycle_start) % (n_frames - cycle_start))
        return int(raw_frame % n_frames)

    def get_reference_contact(self, offset: float = 0.0) -> np.ndarray:
        if self._reference_contact_labels.shape[0] == 0 or self._reference_fps <= 0.0:
            return np.zeros(2, dtype=np.float32)
        frame = self.get_reference_frame_index(offset) % int(self._reference_contact_labels.shape[0])
        return self._reference_contact_labels[frame].copy().astype(np.float32)

    def get_reference_root_velocity(self, offset: float = 0.0) -> np.ndarray:
        if self._reference_root_pos.shape[0] < 2 or self._reference_fps <= 0.0:
            return np.zeros(3, dtype=np.float32)
        n = int(self._reference_root_pos.shape[0])
        frame = self.get_reference_frame_index(offset) % n
        next_frame = self.get_reference_frame_index(offset + 1.0 / max(self._reference_fps, 1e-6)) % n
        if (
            self._reference_cycle_start_frame > 0
            and frame >= self._reference_cycle_start_frame
            and next_frame == self._reference_cycle_start_frame
            and frame > self._reference_cycle_start_frame
        ):
            next_frame = frame
        return (
            (self._reference_root_pos[next_frame] - self._reference_root_pos[frame])
            * self._reference_fps
            * abs(float(self.reference_time_scale))
        ).astype(np.float32)

    def get_reference_action(self, offset: float = 0.0) -> np.ndarray:
        if self._reference_joint_pos.shape[0] == 0 or self._reference_fps <= 0.0:
            return self.home_joint_pos[: int(self.model.nu)].copy().astype(np.float32)
        n_frames = int(self._reference_joint_pos.shape[0])
        frame = self.get_reference_frame_index(offset) % n_frames
        action = self._reference_joint_pos[frame].copy()
        home = self.home_joint_pos[: int(self.model.nu)].copy().astype(np.float32)
        n = min(action.shape[0], home.shape[0])
        action[:n] = home[:n] + self.reference_action_blend * (action[:n] - home[:n])
        return np.clip(action, self._ctrl_low, self._ctrl_high).astype(np.float32)

    def _reset_data_to_home(self) -> None:
        if self._home_key_id >= 0 and int(self.model.nkey) > self._home_key_id:
            self._mujoco.mj_resetDataKeyframe(self.model, self.data, self._home_key_id)
            return
        self._mujoco.mj_resetData(self.model, self.data)

    def _stage_to_mode(self, stage: str) -> str:
        return STAGE_TO_MODE.get(stage, "STAND")

    def get_gait_phase(self) -> float:
        if self.stage != "walk":
            return 0.0
        phase = 2.0 * np.pi * self.gait_frequency * float(self.data.time) + self.gait_phase_offset
        return float(np.mod(phase, 2.0 * np.pi))

    def get_reference_time(self) -> float:
        return self.reference_time_scale * float(self.data.time) + float(self._reference_time_offset)

    def _get_phase_obs(self) -> np.ndarray:
        phase = self.get_gait_phase()
        return np.array([np.sin(phase), np.cos(phase)], dtype=np.float32)

    def _get_reference_obs(self, nominal: np.ndarray) -> np.ndarray:
        if not self.reference_obs:
            return np.zeros(0, dtype=np.float32)
        if self._reference_joint_pos.shape[0] == 0 or self._reference_fps <= 0.0:
            return np.zeros(int(self.model.nu) * self.reference_obs_horizon, dtype=np.float32)

        ctrl_range = np.maximum(self._ctrl_high - self._ctrl_low, 1e-6)
        frames = []
        for i in range(self.reference_obs_horizon):
            t = self.get_reference_time() + i * self.reference_obs_dt
            n_frames = int(self._reference_joint_pos.shape[0])
            frame = int(np.floor((t * self._reference_fps) % n_frames)) % n_frames
            ref = self._reference_joint_pos[frame]
            n = min(ref.shape[0], nominal.shape[0], ctrl_range.shape[0])
            delta = np.zeros(int(self.model.nu), dtype=np.float32)
            delta[:n] = (ref[:n] - nominal[:n]) / ctrl_range[:n]
            frames.append(delta)
        return np.concatenate(frames).astype(np.float32)

    def _get_reference_error_obs(self, joint_pos: np.ndarray) -> np.ndarray:
        if not self.reference_error_obs:
            return np.zeros(0, dtype=np.float32)
        if self._reference_joint_pos.shape[0] == 0 or self._reference_fps <= 0.0:
            return np.zeros(int(self.model.nu), dtype=np.float32)

        n_frames = int(self._reference_joint_pos.shape[0])
        frame = int(np.floor((self.get_reference_time() * self._reference_fps) % n_frames)) % n_frames
        ref = self._reference_joint_pos[frame]
        ctrl_range = np.maximum(self._ctrl_high - self._ctrl_low, 1e-6)
        err = np.zeros(int(self.model.nu), dtype=np.float32)
        n = min(joint_pos.shape[0], ref.shape[0], ctrl_range.shape[0])
        err[:n] = (joint_pos[:n] - ref[:n]) / ctrl_range[:n]
        return err.astype(np.float32)

    def _get_base_action(self) -> np.ndarray:
        return build_nominal_action(self, self.robot, self.base_controller)

    def _ramp_alpha(self, ramp_time: float) -> float:
        if ramp_time <= 1e-9:
            return 1.0
        return float(np.clip(float(self.data.time) / ramp_time, 0.0, 1.0))

    def _get_ramped_base_action(self) -> np.ndarray:
        base = self._get_base_action()
        if self.nominal_ramp_time <= 0.0 or self.base_controller == "home":
            return base.astype(np.float32)
        home = self.home_joint_pos[: int(self.model.nu)].copy().astype(np.float32)
        n = min(base.shape[0], home.shape[0])
        out = base.copy().astype(np.float32)
        alpha = self._ramp_alpha(self.nominal_ramp_time)
        out[:n] = home[:n] + alpha * (base[:n] - home[:n])
        return np.clip(out, self._ctrl_low, self._ctrl_high).astype(np.float32)

    def _get_residual_alpha(self) -> float:
        return self._ramp_alpha(self.residual_ramp_time)

    def _get_foot_contacts(self) -> np.ndarray:
        contacts = np.zeros(4, dtype=np.float32)
        if not self._foot_geom_ids:
            return contacts
        if self.robot_kind == "humanoid":
            left_ids = {
                geom_id
                for name, geom_id in self._foot_geom_ids.items()
                if name.startswith("left_") and geom_id >= 0
            }
            right_ids = {
                geom_id
                for name, geom_id in self._foot_geom_ids.items()
                if name.startswith("right_") and geom_id >= 0
            }
            for contact_idx in range(int(self.data.ncon)):
                contact = self.data.contact[contact_idx]
                g1 = int(contact.geom1)
                g2 = int(contact.geom2)
                if g1 in left_ids or g2 in left_ids:
                    contacts[0] = 1.0
                if g1 in right_ids or g2 in right_ids:
                    contacts[1] = 1.0
            return contacts
        names = self._foot_names[:4]
        foot_ids = {name: self._foot_geom_ids.get(name, -1) for name in names}
        for contact_idx in range(int(self.data.ncon)):
            contact = self.data.contact[contact_idx]
            g1 = int(contact.geom1)
            g2 = int(contact.geom2)
            for i, name in enumerate(names):
                geom_id = foot_ids[name]
                if geom_id >= 0 and (g1 == geom_id or g2 == geom_id):
                    contacts[i] = 1.0
        return contacts

    def _get_foot_state_obs(self) -> np.ndarray:
        if not self.foot_state_obs:
            return np.zeros(0, dtype=np.float32)
        out = np.zeros(6, dtype=np.float32)
        if self.robot_kind != "humanoid":
            return out
        try:
            trunk_pos = np.asarray(self.data.xpos[self._trunk_body_id], dtype=np.float32)
        except Exception:
            return out
        for foot_i, side in enumerate(("left", "right")):
            try:
                site_id = int(self.model.site(f"{side}_foot").id)
                foot_pos = np.asarray(self.data.site_xpos[site_id], dtype=np.float32)
            except Exception:
                continue
            start = 3 * foot_i
            out[start : start + 3] = foot_pos - trunk_pos
        return out

    def _uniform(self, low: float, high: float) -> float:
        rng = getattr(self, "np_random", None)
        if rng is None:
            rng = np.random.default_rng()
        return float(rng.uniform(low, high))

    @staticmethod
    def _wrap_angle(angle: float) -> float:
        return float((angle + np.pi) % (2.0 * np.pi) - np.pi)

    @staticmethod
    def _yaw_from_wxyz(quat: np.ndarray) -> float:
        w, x, y, z = [float(v) for v in quat[:4]]
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return float(math.atan2(siny_cosp, cosy_cosp))

    def _get_base_xy_yaw(self) -> tuple[np.ndarray, float]:
        xy = np.asarray(self.data.qpos[:2], dtype=np.float32).copy()
        yaw = 0.0
        if self.data.qpos.shape[0] >= 7:
            yaw = self._yaw_from_wxyz(np.asarray(self.data.qpos[3:7], dtype=np.float32))
        return xy, yaw

    def _reset_waypoints(self) -> None:
        self._waypoints = np.zeros((0, 2), dtype=np.float32)
        self._waypoint_index = 0
        self._waypoint_start_xy, self._waypoint_start_yaw = self._get_base_xy_yaw()
        if self.command_generator != "waypoint" or self.waypoint_count <= 0:
            return

        rng = getattr(self, "np_random", None)
        if rng is None:
            rng = np.random.default_rng()
        fwd_low, fwd_high = self.waypoint_forward_range
        lat_low, lat_high = self.waypoint_lateral_range
        if fwd_high < fwd_low:
            fwd_low, fwd_high = fwd_high, fwd_low
        if lat_high < lat_low:
            lat_low, lat_high = lat_high, lat_low

        c = float(np.cos(self._waypoint_start_yaw))
        s = float(np.sin(self._waypoint_start_yaw))
        forward = np.array([c, s], dtype=np.float32)
        lateral = np.array([-s, c], dtype=np.float32)

        points: list[np.ndarray] = []
        current = self._waypoint_start_xy + forward * self.waypoint_first_distance
        points.append(current.copy())
        for _ in range(max(self.waypoint_count - 1, 0)):
            dx = float(rng.uniform(fwd_low, fwd_high))
            dy = float(rng.uniform(lat_low, lat_high))
            current = current + forward * dx + lateral * dy
            points.append(current.copy())
        self._waypoints = np.asarray(points, dtype=np.float32).reshape(-1, 2)

    def _update_waypoint_command(self) -> None:
        if self.command_generator != "waypoint" or self._waypoints.shape[0] == 0:
            return
        xy, yaw = self._get_base_xy_yaw()
        while self._waypoint_index < self._waypoints.shape[0] - 1:
            dist = float(np.linalg.norm(self._waypoints[self._waypoint_index] - xy))
            if dist > self.waypoint_radius:
                break
            self._waypoint_index += 1

        target = self._waypoints[min(self._waypoint_index, self._waypoints.shape[0] - 1)]
        delta = target - xy
        dist = float(np.linalg.norm(delta))
        target_yaw = float(math.atan2(float(delta[1]), float(delta[0]))) if dist > 1e-6 else yaw
        heading_error = self._wrap_angle(target_yaw - yaw)
        facing_scale = max(float(np.cos(heading_error)), 0.0)
        vx = float(np.clip(self.waypoint_target_speed * facing_scale, self.waypoint_vx_min, self.waypoint_vx_max))
        yaw_rate = float(
            np.clip(
                self.waypoint_yaw_kp * heading_error,
                -self.waypoint_yaw_rate_max,
                self.waypoint_yaw_rate_max,
            )
        )
        self.set_command(np.array([vx, 0.0, yaw_rate], dtype=np.float32))

    def _apply_dynamics_randomization(self) -> None:
        self.model.geom_friction[:] = self._default_geom_friction
        self.model.body_mass[:] = self._default_body_mass
        if not self.randomize_dynamics:
            return

        fr_low, fr_high = self.friction_scale_range
        mass_low, mass_high = self.trunk_mass_scale_range
        friction_scale = self._uniform(fr_low, fr_high)
        mass_scale = self._uniform(mass_low, mass_high)
        self.model.geom_friction[:, 0] = self._default_geom_friction[:, 0] * friction_scale
        self.model.body_mass[self._trunk_body_id] = (
            self._default_body_mass[self._trunk_body_id] * mass_scale
        )

    def _apply_observation_noise(self, obs: np.ndarray) -> np.ndarray:
        if self.obs_noise_std <= 0.0:
            return obs
        rng = getattr(self, "np_random", None)
        if rng is None:
            rng = np.random.default_rng()
        noise = rng.normal(0.0, self.obs_noise_std, size=obs.shape).astype(np.float32)
        return (obs + noise).astype(np.float32)

    def _apply_action_delay(self, policy_action: np.ndarray) -> np.ndarray:
        if self.action_delay_steps <= 0:
            return policy_action
        if not self._delayed_actions:
            for _ in range(self.action_delay_steps + 1):
                self._delayed_actions.append(np.zeros_like(policy_action, dtype=np.float32))
        self._delayed_actions.append(policy_action.copy().astype(np.float32))
        return self._delayed_actions[0].copy().astype(np.float32)

    def _apply_action_noise(self, policy_action: np.ndarray) -> np.ndarray:
        if self.action_noise_std <= 0.0:
            return policy_action
        rng = getattr(self, "np_random", None)
        if rng is None:
            rng = np.random.default_rng()
        noise = rng.normal(0.0, self.action_noise_std, size=policy_action.shape).astype(np.float32)
        return (policy_action + noise).astype(np.float32)

    def _get_obs(self) -> np.ndarray:
        qpos = self.data.qpos.ravel().astype(np.float32)
        qvel = self.data.qvel.ravel().astype(np.float32)

        base_lin_vel = qvel[0:3]
        base_ang_vel = qvel[3:6]

        projected_gravity = self._get_projected_gravity()

        nominal = self._get_ramped_base_action()
        self.nominal_action = nominal.copy()
        joint_pos = qpos[7:]
        n = min(joint_pos.shape[0], nominal.shape[0])
        joint_delta = joint_pos.copy()
        joint_delta[:n] = joint_pos[:n] - nominal[:n]
        joint_vel = qvel[6:]

        obs = np.concatenate(
            [
                base_lin_vel,
                base_ang_vel,
                projected_gravity,
                joint_delta,
                joint_vel,
                self.command,
                self.prev_action,
                self._get_phase_obs(),
                self._get_foot_contacts(),
                self._get_foot_state_obs(),
                self._get_reference_obs(nominal),
                self._get_reference_error_obs(joint_pos),
            ]
        ).astype(np.float32)
        return self._apply_observation_noise(obs)

    def _get_projected_gravity(self) -> np.ndarray:
        xmat = np.asarray(self.data.xmat[self._trunk_body_id])
        rot = xmat.reshape(3, 3).astype(np.float32)
        return rot.T @ np.array([0.0, 0.0, -1.0], dtype=np.float32)

    def _check_fall(self) -> tuple[bool, str, dict[str, float]]:
        trunk_height = float(self.data.xpos[self._trunk_body_id][2])
        gravity_z = float(self._get_projected_gravity()[2])
        forward_speed = float(self.data.qvel[0])
        height_threshold = self.fall_height_threshold
        if self.fall_height_ratio is not None and self._home_z > 0.0:
            height_threshold = max(height_threshold, self.fall_height_ratio * self._home_z)

        base_info = {
            "trunk_height": trunk_height,
            "projected_gravity_z": gravity_z,
            "height_threshold": float(height_threshold),
            "forward_speed": forward_speed,
            "max_forward_speed": float(self.max_forward_speed),
        }
        if trunk_height < height_threshold:
            return True, "fall_height", base_info
        if gravity_z > self.fall_gravity_z_threshold:
            return True, "fall_tilt", base_info
        if self.max_forward_speed > 0.0 and forward_speed > self.max_forward_speed:
            return True, "forward_speed_limit", base_info
        return False, "", base_info

    def _get_stage_command_bounds(self, stage: str) -> tuple[np.ndarray, np.ndarray]:
        if stage not in self._stage_command_ranges:
            valid = ", ".join(sorted(self._stage_command_ranges.keys()))
            raise ValueError(f"Unknown stage '{stage}'. Available: {valid}")
        low, high = self._stage_command_ranges[stage]
        return low.copy(), high.copy()

    def sample_command(self) -> np.ndarray:
        if self.command_generator == "waypoint":
            return np.asarray(self.command, dtype=np.float32).copy()
        if self.command_generator not in {"", "random"}:
            raise ValueError(
                f"Unknown command_generator '{self.command_generator}'. "
                "Use random or waypoint."
            )
        low, high = self._get_stage_command_bounds(self.stage)
        rng = getattr(self, "np_random", None)
        if rng is None:
            rng = np.random.default_rng()
        cmd = rng.uniform(low=low, high=high).astype(np.float32)
        return cmd

    def set_command(self, command: np.ndarray) -> None:
        low, high = self._get_stage_command_bounds(self.stage)
        cmd = np.asarray(command, dtype=np.float32).reshape(3)
        self.command = np.clip(cmd, low, high).astype(np.float32)

    def set_stage(self, stage: str, *, resample_command: bool = True) -> None:
        self._get_stage_command_bounds(stage)
        self.stage = stage
        self.mode = self._stage_to_mode(stage)
        if resample_command:
            self.command = self.sample_command()

    def set_mode(self, mode: str, *, resample_command: bool = True) -> None:
        mode_key = mode.strip().upper()
        if mode_key not in MODE_TO_STAGE:
            valid = ", ".join(sorted(MODE_TO_STAGE))
            raise ValueError(f"Unknown mode '{mode}'. Available: {valid}")
        self.set_stage(MODE_TO_STAGE[mode_key], resample_command=resample_command)

    def _reset_reward_module(self) -> None:
        # Always reset reward module state at episode start (e.g., z0, prev_action).
        reset_fn = getattr(self._reward_module, "reset", None)
        if callable(reset_fn):
            reset_fn(self)

    def _get_info(self) -> dict[str, Any]:
        base_lin_vel = np.asarray(self.data.qvel[0:3], dtype=np.float32)
        base_ang_vel = np.asarray(self.data.qvel[3:6], dtype=np.float32)
        base_pos = np.asarray(self.data.qpos[0:3], dtype=np.float32)
        cmd = np.asarray(self.command, dtype=np.float32)
        tracking_vec = np.array(
            [
                float(base_lin_vel[0] - cmd[0]),
                float(base_lin_vel[1] - cmd[1]),
                float(base_ang_vel[2] - cmd[2]),
            ],
            dtype=np.float32,
        )
        waypoint_dist = 0.0
        waypoint_target_x = 0.0
        waypoint_target_y = 0.0
        if self._waypoints.shape[0] > 0:
            waypoint_idx = min(self._waypoint_index, self._waypoints.shape[0] - 1)
            waypoint_target = self._waypoints[waypoint_idx]
            waypoint_dist = float(np.linalg.norm(waypoint_target - base_pos[:2]))
            waypoint_target_x = float(waypoint_target[0])
            waypoint_target_y = float(waypoint_target[1])
        return {
            "step_count": self._step_count,
            "robot": self.robot,
            "robot_kind": self.robot_kind,
            "sim_time": float(self.data.time),
            "nq": int(self.model.nq),
            "nv": int(self.model.nv),
            "nu": int(self.model.nu),
            "base_pos_x": float(base_pos[0]) if base_pos.size > 0 else 0.0,
            "base_pos_y": float(base_pos[1]) if base_pos.size > 1 else 0.0,
            "base_pos_z": float(base_pos[2]) if base_pos.size > 2 else 0.0,
            "base_lin_vel_x": float(base_lin_vel[0]),
            "base_lin_vel_y": float(base_lin_vel[1]),
            "base_lin_vel_z": float(base_lin_vel[2]),
            "base_yaw_rate": float(base_ang_vel[2]),
            "base_lin_vel_l2": float(np.linalg.norm(base_lin_vel)),
            "base_ang_vel_l2": float(np.linalg.norm(base_ang_vel)),
            "command_tracking_l2": float(np.linalg.norm(tracking_vec)),
            "action_l2": float(np.linalg.norm(self.prev_action)),
            "joint_dev_l2": float(
                np.linalg.norm(
                    np.asarray(self.data.qpos[7:], dtype=np.float32)[: self.nominal_action.shape[0]]
                    - self.nominal_action
                )
            ),
            "command_vx": float(cmd[0]),
            "command_vy": float(cmd[1]),
            "command_yaw_rate": float(cmd[2]),
            "command_generator": self.command_generator,
            "waypoint_index": float(self._waypoint_index),
            "waypoint_count": float(self._waypoints.shape[0]),
            "waypoint_dist": waypoint_dist,
            "waypoint_target_x": waypoint_target_x,
            "waypoint_target_y": waypoint_target_y,
        }

    def reset(
        self,
        *,
        seed: int | None = None,
        options: dict[str, Any] | None = None,
    ) -> tuple[np.ndarray, dict[str, Any]]:
        super().reset(seed=seed)
        if options is not None:
            if "mode" in options:
                self.set_mode(str(options["mode"]), resample_command=False)
            if "stage" in options:
                self.set_stage(str(options["stage"]), resample_command=False)
            if "command" in options:
                self.set_command(np.asarray(options["command"], dtype=np.float32))

        self._apply_dynamics_randomization()
        self._reset_data_to_home()
        self._soft_fall_grace_steps = 0
        self._reference_time_offset = 0.0
        if self.reference_start_time is not None and self._reference_fps > 0.0 and self._reference_joint_pos.shape[0] > 0:
            duration = float(self._reference_joint_pos.shape[0]) / float(self._reference_fps)
            self._reference_time_offset = float(np.clip(self.reference_start_time, 0.0, duration))
        elif self.reference_random_start and self._reference_fps > 0.0 and self._reference_joint_pos.shape[0] > 0:
            duration = float(self._reference_joint_pos.shape[0]) / float(self._reference_fps)
            if self.reference_start_time_range is None:
                start_low, start_high = 0.0, duration
            else:
                start_low, start_high = self.reference_start_time_range
                start_low = float(np.clip(start_low, 0.0, duration))
                start_high = float(np.clip(start_high, 0.0, duration))
                if start_high < start_low:
                    start_low, start_high = start_high, start_low
            self._reference_time_offset = self._uniform(start_low, start_high)
        if self.reference_init_pose_on_reset and self._reference_joint_pos.shape[0] > 0:
            frame = self.get_reference_frame_index()
            ref_action = self.get_reference_action()
            if self.reference_init_pose_blend is not None:
                home = self.home_joint_pos[: int(self.model.nu)].copy().astype(np.float32)
                m = min(ref_action.shape[0], home.shape[0])
                ref_action[:m] = home[:m] + self.reference_init_pose_blend * (ref_action[:m] - home[:m])
            n = min(ref_action.shape[0], max(self.data.qpos.shape[0] - 7, 0))
            if n > 0:
                self.data.qpos[7 : 7 + n] = ref_action[:n]
                if self._reference_root_quat.shape[0] > 0 and self.data.qpos.shape[0] >= 7:
                    root_frame = frame % int(self._reference_root_quat.shape[0])
                    quat = self._reference_root_quat[root_frame].astype(np.float64)
                    norm = float(np.linalg.norm(quat))
                    if norm > 1e-6:
                        self.data.qpos[3:7] = quat / norm
                if self._reference_root_pos.shape[0] > 0 and self.data.qpos.shape[0] >= 3:
                    root_frame = frame % int(self._reference_root_pos.shape[0])
                    self.data.qpos[2] = float(self._reference_root_pos[root_frame, 2])
                self.data.qvel[:] = 0.0
                next_frame = (frame + 1) % int(self._reference_joint_pos.shape[0])
                ref_dt = 1.0 / max(self._reference_fps * abs(float(self.reference_time_scale)), 1e-6)
                next_action = self._reference_joint_pos[next_frame].copy()
                home = self.home_joint_pos[: int(self.model.nu)].copy().astype(np.float32)
                m = min(next_action.shape[0], home.shape[0])
                next_action[:m] = home[:m] + self.reference_action_blend * (next_action[:m] - home[:m])
                joint_vel = (next_action[:n] - ref_action[:n]) / ref_dt
                self.data.qvel[6 : 6 + n] = joint_vel[: max(min(n, self.data.qvel.shape[0] - 6), 0)]
                if self._reference_root_pos.shape[0] > 1:
                    root_frame = frame % int(self._reference_root_pos.shape[0])
                    root_next = (root_frame + 1) % int(self._reference_root_pos.shape[0])
                    root_vel = (
                        self._reference_root_pos[root_next] - self._reference_root_pos[root_frame]
                    ) / ref_dt
                    self.data.qvel[:3] = root_vel[:3]
                self._mujoco.mj_forward(self.model, self.data)
        failure_reset = False
        if options is not None and bool(options.get("failure_state_reset", False)):
            failure_reset = self._apply_failure_state_reset()
        elif self._should_reset_to_failure_state():
            failure_reset = self._apply_failure_state_reset()
        if self.model.nu > 0:
            self.nominal_action = self._get_ramped_base_action()
            if self.residual_action:
                self.data.ctrl[:] = self.nominal_action
            else:
                self.data.ctrl[:] = 0.5 * (self._ctrl_low + self._ctrl_high)
            self.prev_action = self.data.ctrl.copy().astype(np.float32)
        else:
            self.prev_action = np.zeros(0, dtype=np.float32)
        self._delayed_actions.clear()
        self._mujoco.mj_forward(self.model, self.data)
        self._reset_reward_module()

        self._step_count = 0
        if options is not None and "command" in options:
            self._waypoints = np.zeros((0, 2), dtype=np.float32)
            self._waypoint_index = 0
        elif self.command_generator == "waypoint":
            self._reset_waypoints()
            self._update_waypoint_command()
        else:
            self.command = self.sample_command()
        obs = self._get_obs()
        info = self._get_info()
        info["stage"] = self.stage
        info["mode"] = self.mode
        info["command"] = self.command.copy()
        fallen, reason, fall_info = self._check_fall()
        info["terminated_reason"] = reason
        info["is_fallen"] = fallen
        info["failure_state_reset"] = failure_reset
        info.update(fall_info)

        if self.render_mode == "human":
            self.render()
        return obs, info

    def step(
        self, action: np.ndarray
    ) -> tuple[np.ndarray, float, bool, bool, dict[str, Any]]:
        self._update_waypoint_command()
        policy_action = np.asarray(action, dtype=np.float32)
        applied_action = np.zeros(self.model.nu, dtype=np.float32)
        if self.model.nu > 0:
            policy_action = np.clip(policy_action, self.action_space.low, self.action_space.high)
            policy_action = self._apply_action_delay(policy_action)
            policy_action = self._apply_action_noise(policy_action)
            policy_action = np.clip(policy_action, self.action_space.low, self.action_space.high)
            if self.residual_action:
                base_action = self._get_ramped_base_action()
                self.nominal_action = base_action.copy()
                policy_action = policy_action * self._residual_action_mask
                applied_action = np.clip(
                    base_action + self.residual_scale * self._get_residual_alpha() * policy_action,
                    self._ctrl_low,
                    self._ctrl_high,
                ).astype(np.float32)
            else:
                applied_action = policy_action.astype(np.float32)
            self.data.ctrl[:] = applied_action
            self.prev_action = self.data.ctrl.copy().astype(np.float32)

        for _ in range(self.frame_skip):
            self._mujoco.mj_step(self.model, self.data)

        self._step_count += 1

        reward, reward_info = self._reward_module.compute(self, applied_action)
        terminated, term_reason, fall_info = self._check_fall()
        soft_termination_suppressed = False
        if terminated and term_reason.startswith("fall_") and self.soft_termination_prob < 1.0:
            can_suppress = (
                self.soft_termination_max_grace_steps <= 0
                or self._soft_fall_grace_steps < self.soft_termination_max_grace_steps
            )
            if can_suppress:
                rng = getattr(self, "np_random", None)
                draw = float(rng.random()) if rng is not None else float(np.random.random())
                if draw >= self.soft_termination_prob:
                    terminated = False
                    soft_termination_suppressed = True
                    self._soft_fall_grace_steps += 1
            if terminated:
                self._soft_fall_grace_steps = 0
        elif not term_reason.startswith("fall_"):
            self._soft_fall_grace_steps = 0
        truncated = self._step_count >= self.max_episode_steps

        if self.render_mode == "human":
            self.render()
            if self.sleep_sec > 0:
                time.sleep(self.sleep_sec)

        info = self._get_info()
        info["stage"] = self.stage
        info["mode"] = self.mode
        info["command"] = self.command.copy()
        info["residual_action"] = self.residual_action
        if self.model.nu > 0:
            info["policy_action_l2"] = float(np.linalg.norm(policy_action))
            info["applied_action_l2"] = float(np.linalg.norm(applied_action))
            info["nominal_action_l2"] = float(np.linalg.norm(self.nominal_action))
        info["gait_phase_sin"] = float(self._get_phase_obs()[0])
        info["gait_phase_cos"] = float(self._get_phase_obs()[1])
        contacts = self._get_foot_contacts()
        info["foot_contact_count"] = float(np.sum(contacts))
        for name, value in zip(("FR", "FL", "RR", "RL"), contacts):
            info[f"foot_contact_{name}"] = float(value)
        info["terminated_reason"] = term_reason
        info["is_fallen"] = term_reason.startswith("fall_")
        info["is_terminated"] = terminated
        info["soft_termination_suppressed"] = soft_termination_suppressed
        info["soft_fall_grace_steps"] = float(self._soft_fall_grace_steps)
        info.update(fall_info)
        info.update(reward_info)
        self._update_waypoint_command()
        return self._get_obs(), float(reward), terminated, truncated, info

    def _build_residual_action_mask(self) -> np.ndarray:
        mask = np.ones(self.model.nu, dtype=np.float32)
        mode = self.residual_action_mask
        if mode in {"", "all"}:
            return mask
        if self.robot == "g1" and mode in {"g1_stabilizer", "stabilizer", "lower_stabilizer"}:
            mask[:] = 0.0
            if self.model.nu >= 12:
                # G1 actuator order: left leg 0:6, right leg 6:12.
                # Keep nominal in charge of hip pitch/yaw and let PPO adjust
                # frontal support, knee clearance, and ankle placement.
                for idx in (1, 3, 4, 5, 7, 9, 10, 11):
                    mask[idx] = 1.0
            return mask
        if self.robot == "g1" and mode in {"g1_turning", "turning", "yaw_stabilizer"}:
            mask[:] = 0.0
            if self.model.nu >= 12:
                # Start from the stabilizer mask and add hip yaw joints so the
                # residual policy can create a small commanded turn.
                for idx in (1, 2, 3, 4, 5, 7, 8, 9, 10, 11):
                    mask[idx] = 1.0
            if self.model.nu >= 15:
                # Waist yaw/roll/pitch help redirect the torso without giving
                # the residual policy full upper-body authority.
                for idx in (12, 13, 14):
                    mask[idx] = 1.0
            return mask
        if self.robot == "g1" and mode in {"g1_capture", "capture", "lower_capture"}:
            mask[:] = 0.0
            if self.model.nu >= 12:
                # Capture failures are sagittal foot-placement failures. This
                # mask lets PPO correct hip pitch in addition to the stabilizer
                # joints, while still leaving hip yaw and upper body mostly to
                # the nominal controller.
                for idx in (0, 1, 3, 4, 5, 6, 7, 9, 10, 11):
                    mask[idx] = 1.0
            if self.model.nu >= 15:
                mask[14] = 1.0
            return mask
        if self.robot == "g1" and mode in {"g1_balance", "balance", "balance_only"}:
            mask[:] = 0.0
            if self.model.nu >= 12:
                # Preserve nominal swing foot placement by freezing hip pitch,
                # knees, and ankle pitch. Let PPO correct only frontal-plane
                # hip balance.
                for idx in (1, 7):
                    mask[idx] = 1.0
            if self.model.nu >= 15:
                # Waist roll/pitch can recover torso tilt without directly
                # changing the next foot landing target.
                for idx in (13, 14):
                    mask[idx] = 1.0
            return mask
        if self.robot == "g1" and mode in {"g1_sagittal", "sagittal", "sagittal_capture"}:
            mask[:] = 0.0
            if self.model.nu >= 12:
                # Keep frontal-plane balance joints under the nominal
                # stabilizer. Let PPO correct only sagittal leg placement.
                for idx in (0, 3, 4, 6, 9, 10):
                    mask[idx] = 1.0
            if self.model.nu >= 15:
                mask[14] = 1.0
            return mask
        raise ValueError(f"Unknown residual_action_mask '{self.residual_action_mask}'.")

    def render(self) -> np.ndarray | None:
        if self._render_error is not None:
            return None

        if self.render_mode == "human":
            if self._viewer is None:
                try:
                    self._viewer = self._viewer_mod.launch_passive(self.model, self.data)
                except Exception as exc:
                    self._render_error = str(exc)
                    raise RuntimeError(
                        "Failed to open human viewer. Run from local GUI terminal with "
                        "`mjpython scripts/test_env.py --render-mode human`."
                    ) from exc
            if hasattr(self._viewer, "is_running") and not self._viewer.is_running():
                return None
            self._viewer.sync()
            return None

        if self.render_mode == "rgb_array":
            if self._renderer is None:
                try:
                    self._renderer = self._mujoco.Renderer(
                        self.model, height=self.image_height, width=self.image_width
                    )
                    self._render_camera = self._mujoco.MjvCamera()
                    self._render_camera.type = self._mujoco.mjtCamera.mjCAMERA_FREE
                    self._render_camera.distance = self.render_camera_distance
                    self._render_camera.azimuth = self.render_camera_azimuth
                    self._render_camera.elevation = self.render_camera_elevation
                except Exception as exc:
                    self._render_error = str(exc)
                    return None
            if self._render_camera is not None:
                trunk_pos = np.asarray(self.data.xpos[self._trunk_body_id], dtype=np.float64)
                self._render_camera.lookat[:] = (
                    float(trunk_pos[0]),
                    float(trunk_pos[1]),
                    self.render_camera_lookat_z,
                )
                self._renderer.update_scene(self.data, camera=self._render_camera)
            else:
                self._renderer.update_scene(self.data)
            return self._renderer.render()
        return None

    def close(self) -> None:
        if self._viewer is not None:
            self._viewer.close()
            self._viewer = None
        if self._renderer is not None:
            self._renderer.close()
            self._renderer = None
        self._render_camera = None
        self._render_error = None
        return None
