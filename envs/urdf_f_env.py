from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Any

import gymnasium as gym
import mujoco
import numpy as np
from gymnasium import spaces


PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_MODEL = PROJECT_ROOT / "envs/robots/urdf_f_link/URDF_F_link_virtual_com_gate_com_y005.xml"
DEFAULT_POSE = PROJECT_ROOT / "configs/dynamic_standing_pose_actuator_dynamics_contact_5s.json"
ROLL_ROLE_ACTUATORS = {
    "motor_left_hip_pitch",
    "motor_right_hip_pitch",
    "motor_left_ankle_pitch",
    "motor_right_ankle_pitch",
}
PITCH_ROLE_ACTUATORS = {
    "motor_left_hip_roll",
    "motor_right_hip_roll",
    "motor_left_ankle_roll",
    "motor_right_ankle_roll",
}


def quat_to_rpy(q: np.ndarray) -> tuple[float, float, float]:
    w, x, y, z = [float(v) for v in q[:4]]
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = 2.0 * (w * y - z * x)
    pitch = math.copysign(math.pi / 2.0, sinp) if abs(sinp) >= 1.0 else math.asin(sinp)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


class UrdfFEnv(gym.Env[np.ndarray, np.ndarray]):
    """Gymnasium wrapper for the URDF_F humanoid validation model.

    The policy action is a normalized joint-target residual. A PD loop converts
    the target to motor torque, matching the hardware-validation probes.
    """

    metadata = {"render_modes": ["rgb_array"], "render_fps": 30}

    def __init__(
        self,
        model_path: str | Path = DEFAULT_MODEL,
        pose_path: str | Path = DEFAULT_POSE,
        nominal_pose_path: str | Path | None = None,
        task: str = "standing",
        max_episode_steps: int = 500,
        frame_skip: int = 10,
        kp: float = 20.0,
        kd: float = 12.0,
        torque_limit: float = 30.0,
        action_scale: float = 0.35,
        upright_penalty_weight: float = 8.0,
        action_penalty_weight: float = 0.01,
        action_delta_penalty_weight: float = 0.0,
        right_contact_penalty_weight: float = 0.0,
        left_contact_penalty_weight: float = 0.0,
        clearance_reward_weight: float = 2.0,
        clearance_target: float = 0.002,
        gated_clearance_reward: bool = False,
        clearance_gate_roll: float = 0.12,
        clearance_gate_pitch: float = 0.12,
        step_target_length: float = 0.0,
        step_reward_weight: float = 1.0,
        flat_foot_penalty_weight: float = 0.0,
        flat_foot_deadzone_rad: float = 0.087,
        flat_foot_scale_rad: float = 0.30,
        flat_foot_contact_clearance: float = 0.003,
        swing_step_reward_weight: float = 0.0,
        swing_step_target_speed: float = 0.05,
        weight_shift_reward_weight: float = 0.0,
        swing_contact_penalty_weight: float = 0.0,
        foot_split_penalty_weight: float = 0.0,
        foot_split_deadzone: float = 0.08,
        foot_split_scale: float = 0.12,
        jitter_qpos_std: float = 0.0,
        jitter_qvel_std: float = 0.0,
        fall_penalty: float = 0.0,
        stability_excess_penalty_weight: float = 0.0,
        pose_tracking_penalty_weight: float = 0.3,
        termination_roll_limit: float = 0.55,
        termination_pitch_limit: float = 0.55,
        termination_base_drop: float = 0.12,
        nominal_kp_att: float = 0.0,
        nominal_kd_att: float = 1.0,
        nominal_kcom: float = 1.0,
        nominal_roll_sign: float = 1.0,
        nominal_pitch_sign: float = -1.0,
        render_mode: str | None = None,
        width: int = 640,
        height: int = 480,
    ) -> None:
        super().__init__()
        self.model_path = Path(model_path).resolve()
        self.pose_path = Path(pose_path).resolve()
        self.nominal_pose_path = Path(nominal_pose_path).resolve() if nominal_pose_path else None
        self.task = task
        self.max_episode_steps = int(max_episode_steps)
        self.frame_skip = int(frame_skip)
        self.kp = float(kp)
        self.kd = float(kd)
        self.torque_limit = float(torque_limit)
        self.action_scale = float(action_scale)
        self.upright_penalty_weight = float(upright_penalty_weight)
        self.action_penalty_weight = float(action_penalty_weight)
        self.action_delta_penalty_weight = float(action_delta_penalty_weight)
        self.right_contact_penalty_weight = float(right_contact_penalty_weight)
        self.left_contact_penalty_weight = float(left_contact_penalty_weight)
        self.clearance_reward_weight = float(clearance_reward_weight)
        self.clearance_target = float(clearance_target)
        self.gated_clearance_reward = bool(gated_clearance_reward)
        self.clearance_gate_roll = float(clearance_gate_roll)
        self.clearance_gate_pitch = float(clearance_gate_pitch)
        self.step_target_length = float(step_target_length)
        self.step_reward_weight = float(step_reward_weight)
        self.flat_foot_penalty_weight = float(flat_foot_penalty_weight)
        self.flat_foot_deadzone_rad = float(flat_foot_deadzone_rad)
        self.flat_foot_scale_rad = float(flat_foot_scale_rad)
        self.flat_foot_contact_clearance = float(flat_foot_contact_clearance)
        self.swing_step_reward_weight = float(swing_step_reward_weight)
        self.swing_step_target_speed = float(swing_step_target_speed)
        self.weight_shift_reward_weight = float(weight_shift_reward_weight)
        self.swing_contact_penalty_weight = float(swing_contact_penalty_weight)
        self.foot_split_penalty_weight = float(foot_split_penalty_weight)
        self.foot_split_deadzone = float(foot_split_deadzone)
        self.foot_split_scale = float(foot_split_scale)
        self.jitter_qpos_std = float(jitter_qpos_std)
        self.jitter_qvel_std = float(jitter_qvel_std)
        self._prev_left_foot_y = 0.0
        self._prev_right_foot_y = 0.0
        self.fall_penalty = float(fall_penalty)
        self.stability_excess_penalty_weight = float(stability_excess_penalty_weight)
        self.pose_tracking_penalty_weight = float(pose_tracking_penalty_weight)
        self.termination_roll_limit = float(termination_roll_limit)
        self.termination_pitch_limit = float(termination_pitch_limit)
        self.termination_base_drop = float(termination_base_drop)
        self.nominal_kp_att = float(nominal_kp_att)
        self.nominal_kd_att = float(nominal_kd_att)
        self.nominal_kcom = float(nominal_kcom)
        self.nominal_roll_sign = float(nominal_roll_sign)
        self.nominal_pitch_sign = float(nominal_pitch_sign)
        self.render_mode = render_mode
        self.width = int(width)
        self.height = int(height)

        self.model = mujoco.MjModel.from_xml_path(str(self.model_path))
        self.data = mujoco.MjData(self.model)
        self.joints = self._hinge_joints()
        if len(self.joints) != int(self.model.nu):
            raise ValueError(f"Expected one actuator per hinge joint, got joints={len(self.joints)} nu={self.model.nu}")

        self.base_z, self.init_q, self.nominal_q = self._load_pose()
        self.actuator_names = [
            mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, idx) or f"actuator_{idx}"
            for idx in range(int(self.model.nu))
        ]
        self.joint_qposadr = np.array([j["qposadr"] for j in self.joints], dtype=np.int32)
        self.joint_dofadr = np.array([j["dofadr"] for j in self.joints], dtype=np.int32)
        self.joint_range = np.asarray(self.model.jnt_range[1 : 1 + len(self.joints)], dtype=np.float64)
        self.foot_geom_to_side = self._foot_geom_ids()
        self.base_body_id = int(mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "base_link"))
        self._left_foot_body_id = int(mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "foot_L_1"))
        self._right_foot_body_id = int(mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "foot_R_v1_1"))

        self.prev_action = np.zeros(int(self.model.nu), dtype=np.float32)
        self._step_count = 0
        self._renderer: mujoco.Renderer | None = None

        self.action_space = spaces.Box(
            low=-np.ones(int(self.model.nu), dtype=np.float32),
            high=np.ones(int(self.model.nu), dtype=np.float32),
            dtype=np.float32,
        )
        obs_dim = 3 + 3 + len(self.joints) + len(self.joints) + 2 + 4 + 3 + int(self.model.nu)
        self.observation_space = spaces.Box(low=-np.inf, high=np.inf, shape=(obs_dim,), dtype=np.float32)

    def _hinge_joints(self) -> list[dict[str, int | str]]:
        joints: list[dict[str, int | str]] = []
        for joint_id in range(int(self.model.njnt)):
            if self.model.jnt_type[joint_id] != mujoco.mjtJoint.mjJNT_HINGE:
                continue
            name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT, joint_id) or f"joint_{joint_id}"
            joints.append(
                {
                    "id": joint_id,
                    "name": name,
                    "qposadr": int(self.model.jnt_qposadr[joint_id]),
                    "dofadr": int(self.model.jnt_dofadr[joint_id]),
                }
            )
        return joints

    def _load_pose(self) -> tuple[float, np.ndarray, np.ndarray]:
        base_z = float(self.model.qpos0[2]) if self.model.nq >= 3 else 0.0
        target_q = np.array([float(self.model.qpos0[int(j["qposadr"])]) for j in self.joints], dtype=np.float64)
        payload = json.loads(self.pose_path.read_text(encoding="utf-8"))
        base_z = float(payload.get("base_z", base_z))
        self._init_base_quat: np.ndarray | None = None
        raw_quat = payload.get("base_quat")
        if raw_quat is not None:
            self._init_base_quat = np.array(raw_quat, dtype=np.float64)
        targets = payload.get("joint_targets", {})
        for idx, joint in enumerate(self.joints):
            name = str(joint["name"])
            if name in targets:
                target_q[idx] = float(targets[name])
        init_q = target_q.copy()
        if self.nominal_pose_path is not None:
            nominal_q = np.array([float(self.model.qpos0[int(j["qposadr"])]) for j in self.joints], dtype=np.float64)
            nom_payload = json.loads(self.nominal_pose_path.read_text(encoding="utf-8"))
            nom_targets = nom_payload.get("joint_targets", {})
            for idx, joint in enumerate(self.joints):
                name = str(joint["name"])
                if name in nom_targets:
                    nominal_q[idx] = float(nom_targets[name])
        else:
            nominal_q = target_q.copy()
        return base_z, init_q, nominal_q

    def _foot_geom_ids(self) -> dict[int, str]:
        out: dict[int, str] = {}
        for geom_id in range(int(self.model.ngeom)):
            name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_GEOM, geom_id) or ""
            if name.startswith("foot_L_1_sole_pad_"):
                out[geom_id] = "left"
            if name.startswith("foot_R_v1_1_sole_pad_"):
                out[geom_id] = "right"
        return out

    def _init_state(self) -> None:
        self.data.qpos[:] = self.model.qpos0
        self.data.qvel[:] = 0.0
        self.data.qpos[0:3] = np.array([0.0, 0.0, self.base_z], dtype=np.float64)
        if self._init_base_quat is not None:
            self.data.qpos[3:7] = self._init_base_quat
        else:
            self.data.qpos[3:7] = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
        self.data.qpos[self.joint_qposadr] = self.init_q
        # Initial-state jitter for robustness: perturb joint angles/velocities so the
        # policy sees a distribution of starts instead of a single trajectory.
        if self.jitter_qpos_std > 0.0:
            self.data.qpos[self.joint_qposadr] += self.np_random.normal(
                0.0, self.jitter_qpos_std, size=self.joint_qposadr.shape
            )
        if self.jitter_qvel_std > 0.0:
            self.data.qvel[self.joint_dofadr] += self.np_random.normal(
                0.0, self.jitter_qvel_std, size=self.joint_dofadr.shape
            )
        self.data.ctrl[:] = 0.0
        mujoco.mj_forward(self.model, self.data)

    def _projected_gravity(self) -> np.ndarray:
        rot = np.asarray(self.data.xmat[self.base_body_id], dtype=np.float64).reshape(3, 3)
        return (rot.T @ np.array([0.0, 0.0, -1.0], dtype=np.float64)).astype(np.float32)

    def _contact_stats(self) -> dict[str, float]:
        stats = {
            "left_force": 0.0,
            "right_force": 0.0,
            "left_contacts": 0.0,
            "right_contacts": 0.0,
        }
        force = np.zeros(6, dtype=np.float64)
        for contact_id in range(int(self.data.ncon)):
            contact = self.data.contact[contact_id]
            side = self.foot_geom_to_side.get(int(contact.geom1)) or self.foot_geom_to_side.get(int(contact.geom2))
            if side is None:
                continue
            mujoco.mj_contactForce(self.model, self.data, contact_id, force)
            normal = max(float(force[0]), 0.0)
            stats[f"{side}_force"] += normal
            stats[f"{side}_contacts"] += 1.0
        total = max(stats["left_force"] + stats["right_force"], 1e-9)
        stats["left_force_ratio"] = stats["left_force"] / total
        stats["right_force_ratio"] = stats["right_force"] / total
        return stats

    def _total_com(self) -> np.ndarray:
        mass = np.asarray(self.model.body_mass, dtype=np.float64)
        return (np.asarray(self.data.xipos, dtype=np.float64) * mass[:, None]).sum(axis=0) / mass.sum()

    def _support_center(self) -> tuple[np.ndarray, np.ndarray]:
        xs: list[float] = []
        ys: list[float] = []
        for geom_id in self.foot_geom_to_side:
            center = np.asarray(self.data.geom_xpos[geom_id], dtype=np.float64)
            xmat = np.asarray(self.data.geom_xmat[geom_id], dtype=np.float64).reshape(3, 3)
            half = np.asarray(self.model.geom_size[geom_id], dtype=np.float64)
            extent = np.abs(xmat) @ half
            xs.extend([float(center[0] - extent[0]), float(center[0] + extent[0])])
            ys.extend([float(center[1] - extent[1]), float(center[1] + extent[1])])
        if not xs or not ys:
            return np.zeros(2, dtype=np.float64), np.ones(2, dtype=np.float64)
        center = np.array([(min(xs) + max(xs)) * 0.5, (min(ys) + max(ys)) * 0.5], dtype=np.float64)
        half = np.array([(max(xs) - min(xs)) * 0.5, (max(ys) - min(ys)) * 0.5], dtype=np.float64)
        return center, np.maximum(half, 1e-6)

    def _nominal_stabilizer_torque(self, roll: float, pitch: float) -> np.ndarray:
        com = self._total_com()
        center, half = self._support_center()
        com_err = (com[:2] - center) / half
        wx = float(self.data.qvel[3]) if self.model.nv >= 6 else 0.0
        wy = float(self.data.qvel[4]) if self.model.nv >= 6 else 0.0
        roll_term = self.nominal_kp_att * roll + self.nominal_kd_att * wx + self.nominal_kcom * float(com_err[1])
        pitch_term = self.nominal_kp_att * pitch + self.nominal_kd_att * wy + self.nominal_kcom * float(com_err[0])
        tau = np.zeros(int(self.model.nu), dtype=np.float64)
        for idx, name in enumerate(self.actuator_names):
            if name in ROLL_ROLE_ACTUATORS:
                tau[idx] += self.nominal_roll_sign * roll_term
            if name in PITCH_ROLE_ACTUATORS:
                tau[idx] += self.nominal_pitch_sign * pitch_term
        return tau

    def _right_foot_clearance(self) -> float:
        min_z = np.inf
        for geom_id, side in self.foot_geom_to_side.items():
            if side != "right":
                continue
            z = float(self.data.geom_xpos[geom_id][2] - self.model.geom_size[geom_id][2])
            min_z = min(min_z, z)
        return 0.0 if not np.isfinite(min_z) else min_z

    def _left_foot_clearance(self) -> float:
        min_z = np.inf
        for geom_id, side in self.foot_geom_to_side.items():
            if side != "left":
                continue
            z = float(self.data.geom_xpos[geom_id][2] - self.model.geom_size[geom_id][2])
            min_z = min(min_z, z)
        return 0.0 if not np.isfinite(min_z) else min_z

    def _foot_sole_tilt(self, body_id: int) -> float:
        """Angle (rad) between the foot's up-axis and world +Z. 0 = sole flat on ground."""
        cos_up = float(self.data.xmat[body_id].reshape(3, 3)[2, 2])
        return float(np.arccos(np.clip(cos_up, -1.0, 1.0)))

    def _flat_foot_penalty(self, left_in_contact: bool, right_in_contact: bool) -> float:
        """Penalize a foot in ground contact whose sole is not flat (targets edge-walking).

        Gated on actual contact rather than clearance: a foot walking on its edge stays
        rolled but its box-center clearance reads as 'airborne', so clearance would miss it.
        """
        if self.flat_foot_penalty_weight <= 0.0:
            return 0.0
        dead = self.flat_foot_deadzone_rad
        scale = max(self.flat_foot_scale_rad, 1e-6)
        pen = 0.0
        if left_in_contact:
            pen += np.clip((self._foot_sole_tilt(self._left_foot_body_id) - dead) / scale, 0.0, 1.0)
        if right_in_contact:
            pen += np.clip((self._foot_sole_tilt(self._right_foot_body_id) - dead) / scale, 0.0, 1.0)
        return self.flat_foot_penalty_weight * float(pen)

    def _left_foot_y(self) -> float:
        return float(self.data.xpos[self._left_foot_body_id, 1])

    def _right_foot_y(self) -> float:
        return float(self.data.xpos[self._right_foot_body_id, 1])

    def _obs(self) -> np.ndarray:
        q = self.data.qpos[self.joint_qposadr].astype(np.float32)
        qd = self.data.qvel[self.joint_dofadr].astype(np.float32)
        roll, pitch, yaw = quat_to_rpy(np.asarray(self.data.qpos[3:7], dtype=np.float64))
        stats = self._contact_stats()
        obs = np.concatenate(
            [
                np.asarray([roll, pitch, yaw], dtype=np.float32),
                np.asarray(self.data.qvel[3:6], dtype=np.float32),
                (q - self.nominal_q.astype(np.float32)),
                qd,
                np.asarray([stats["left_force_ratio"], stats["right_force_ratio"]], dtype=np.float32),
                np.asarray(
                    [
                        stats["left_contacts"],
                        stats["right_contacts"],
                        stats["left_force"] / 100.0,
                        stats["right_force"] / 100.0,
                    ],
                    dtype=np.float32,
                ),
                np.asarray([self._left_foot_clearance(), self._right_foot_clearance(), float(self.data.qpos[2] - self.base_z)], dtype=np.float32),
                self.prev_action,
            ]
        )
        return obs.astype(np.float32)

    def _reward(self, action: np.ndarray) -> tuple[float, dict[str, float]]:
        q = self.data.qpos[self.joint_qposadr]
        qd = self.data.qvel[self.joint_dofadr]
        roll, pitch, _ = quat_to_rpy(np.asarray(self.data.qpos[3:7], dtype=np.float64))
        stats = self._contact_stats()
        clearance = self._right_foot_clearance()

        upright_penalty = roll * roll + pitch * pitch
        stability_excess_penalty = (
            max(abs(roll) - self.clearance_gate_roll, 0.0) ** 2
            + max(abs(pitch) - self.clearance_gate_pitch, 0.0) ** 2
        )
        vel_penalty = float(np.mean(qd * qd))
        action_penalty = float(np.mean(np.square(action)))
        action_delta_penalty = float(np.mean(np.square(action - self.prev_action))) if action.size else 0.0
        posture_penalty = float(np.mean(np.square(q - self.nominal_q)))
        contact_reward = 0.0
        task_reward = 0.0
        flat_foot_penalty = 0.0
        swing_step_reward = 0.0
        weight_shift_reward = 0.0
        swing_contact_penalty = 0.0
        foot_split_penalty = 0.0

        stable_for_clearance = (
            abs(roll) <= self.clearance_gate_roll
            and abs(pitch) <= self.clearance_gate_pitch
        )

        if self.task == "standing":
            contact_reward = 0.5 * (min(stats["left_contacts"], 1.0) + min(stats["right_contacts"], 1.0))
        elif self.task == "weight_shift_left":
            task_reward = -4.0 * abs(stats["left_force_ratio"] - 0.65)
            contact_reward = min(stats["left_contacts"], 1.0) + 0.5 * min(stats["right_contacts"], 1.0)
            if self.step_target_length > 0:
                step_fwd = self._right_foot_y() - self._left_foot_y()
                task_reward += self.step_reward_weight * np.clip(step_fwd / self.step_target_length, -1.0, 1.0)
        elif self.task == "weight_shift_right":
            task_reward = -4.0 * abs(stats["right_force_ratio"] - 0.65)
            contact_reward = np.clip(stats["right_contacts"] / 8.0, 0.0, 1.0) + 0.5 * min(stats["left_contacts"], 1.0)
        elif self.task == "right_unload":
            task_reward = 1.5 * np.clip((30.0 - stats["right_force"]) / 30.0, -1.0, 1.0)
            contact_reward = min(stats["left_contacts"], 1.0)
        elif self.task == "left_unload":
            task_reward = 1.5 * np.clip((30.0 - stats["left_force"]) / 30.0, -1.0, 1.0)
            contact_reward = min(stats["right_contacts"], 1.0)
        elif self.task == "right_clearance":
            clearance_scale = 1.0 if stable_for_clearance or not self.gated_clearance_reward else 0.0
            task_reward = clearance_scale * self.clearance_reward_weight * np.clip(
                clearance / max(self.clearance_target, 1e-6), -1.0, 1.0
            )
            task_reward += np.clip((10.0 - stats["right_force"]) / 10.0, -1.0, 1.0)
            if self.step_target_length > 0:
                step_fwd = self._right_foot_y() - self._left_foot_y()
                task_reward += self.step_reward_weight * np.clip(step_fwd / self.step_target_length, -1.0, 1.0)
            contact_reward = min(stats["left_contacts"], 1.0)
        elif self.task == "left_clearance":
            left_clearance = self._left_foot_clearance()
            clearance_scale = 1.0 if stable_for_clearance or not self.gated_clearance_reward else 0.0
            task_reward = clearance_scale * self.clearance_reward_weight * np.clip(
                left_clearance / max(self.clearance_target, 1e-6), -1.0, 1.0
            )
            task_reward += np.clip((10.0 - stats["left_force"]) / 10.0, -1.0, 1.0)
            if self.step_target_length > 0:
                step_fwd = self._left_foot_y() - self._right_foot_y()
                task_reward += self.step_reward_weight * np.clip(step_fwd / self.step_target_length, -1.0, 1.0)
            contact_reward = min(stats["right_contacts"], 1.0)
        elif self.task == "balance_settle":
            # Drive force ratio toward 0.5 while keeping both feet planted
            task_reward = -3.0 * abs(stats["left_force_ratio"] - 0.50)
            contact_reward = min(stats["left_contacts"], 1.0) + min(stats["right_contacts"], 1.0)
        elif self.task == "right_return":
            target_clearance = 0.0005
            low_clearance_reward = 1.0 - np.clip(abs(clearance - target_clearance) / 0.004, 0.0, 1.0)
            right_contact_reward = np.clip(stats["right_contacts"] / 3.0, 0.0, 1.0)
            right_force_reward = np.clip(stats["right_force"] / 25.0, 0.0, 1.0)
            task_reward = 1.2 * low_clearance_reward + 0.8 * right_contact_reward + 0.6 * right_force_reward
            if self.step_target_length > 0:
                target_y = self._left_foot_y() + self.step_target_length
                landing_err = abs(self._right_foot_y() - target_y)
                task_reward += self.step_reward_weight * np.clip(1.0 - landing_err / self.step_target_length, 0.0, 1.0)
            contact_reward = min(stats["left_contacts"], 1.0)
        elif self.task == "left_return":
            left_clearance = self._left_foot_clearance()
            target_clearance = 0.0005
            low_clearance_reward = 1.0 - np.clip(abs(left_clearance - target_clearance) / 0.004, 0.0, 1.0)
            left_contact_reward = np.clip(stats["left_contacts"] / 3.0, 0.0, 1.0)
            left_force_reward = np.clip(stats["left_force"] / 25.0, 0.0, 1.0)
            task_reward = 1.2 * low_clearance_reward + 0.8 * left_contact_reward + 0.6 * left_force_reward
            if self.step_target_length > 0:
                target_y = self._right_foot_y() + self.step_target_length
                landing_err = abs(self._left_foot_y() - target_y)
                task_reward += self.step_reward_weight * np.clip(1.0 - landing_err / self.step_target_length, 0.0, 1.0)
            contact_reward = min(stats["right_contacts"], 1.0)
        elif self.task == "walking":
            vel_y = float(self.data.qvel[1])
            vel_reward = np.clip(vel_y / 0.05, -0.5, 1.0) * 3.0
            left_clr = self._left_foot_clearance()
            right_clr = self._right_foot_clearance()
            left_foot_y = self._left_foot_y()
            right_foot_y = self._right_foot_y()
            dt = self.frame_skip * float(self.model.opt.timestep)
            phase_fn = getattr(self, "_gait_phase", None)
            swing_step_reward = 0.0
            weight_shift_reward = 0.0
            if phase_fn is not None:
                phase = phase_fn()
                right_swing = np.sin(phase) > 0
                swing_clr = right_clr if right_swing else left_clr
                stance_contacts = stats["left_contacts"] if right_swing else stats["right_contacts"]
                gait_reward = np.clip(swing_clr / 0.004, 0.0, 1.5)
                contact_reward = min(stance_contacts, 1.0)
                # Symmetric step reward: the swinging foot should move forward (+Y).
                # Applied identically to both swing phases → discourages one-sided limp.
                if self.swing_step_reward_weight > 0.0:
                    swing_dy = (right_foot_y - self._prev_right_foot_y) if right_swing \
                        else (left_foot_y - self._prev_left_foot_y)
                    swing_speed = swing_dy / max(dt, 1e-6)
                    swing_step_reward = self.swing_step_reward_weight * float(
                        np.clip(swing_speed / max(self.swing_step_target_speed, 1e-6), -0.5, 1.0)
                    )
                # Weight-shift reward: load the stance foot so the swing foot can unload+lift.
                # Root fix for the one-legged gait (swing foot never leaves the ground).
                if self.weight_shift_reward_weight > 0.0:
                    stance_ratio = stats["left_force_ratio"] if right_swing else stats["right_force_ratio"]
                    weight_shift_reward = self.weight_shift_reward_weight * float(
                        np.clip((stance_ratio - 0.5) / 0.5, 0.0, 1.0)
                    )
                # Swing-contact penalty: the swing foot must leave the ground during its phase.
                # Direct (unavoidable) pressure to lift, needed to escape the one-legged gait.
                if self.swing_contact_penalty_weight > 0.0:
                    swing_contacts = stats["right_contacts"] if right_swing else stats["left_contacts"]
                    swing_contact_penalty = self.swing_contact_penalty_weight * min(swing_contacts, 1.0)
            else:
                gait_reward = np.clip(max(left_clr, right_clr) / 0.004, 0.0, 1.5)
                contact_reward = min(stats["left_contacts"] + stats["right_contacts"], 2.0) * 0.3
            both_air = (left_clr > 0.005) and (right_clr > 0.005)
            flat_foot_penalty = self._flat_foot_penalty(
                stats["left_contacts"] > 0, stats["right_contacts"] > 0
            )
            # Foot-split penalty: discourage a persistent fore-aft gap between the feet
            # (the lunge posture that accumulates and destabilizes the bilateral gait).
            if self.foot_split_penalty_weight > 0.0:
                split = abs(left_foot_y - right_foot_y)
                foot_split_penalty = self.foot_split_penalty_weight * float(
                    np.clip((split - self.foot_split_deadzone) / max(self.foot_split_scale, 1e-6), 0.0, 1.0)
                )
            task_reward = (
                vel_reward + gait_reward + swing_step_reward + weight_shift_reward
                - (2.0 if both_air else 0.0) - flat_foot_penalty - swing_contact_penalty
                - foot_split_penalty
            )
            self._prev_left_foot_y = left_foot_y
            self._prev_right_foot_y = right_foot_y
        else:
            raise ValueError(f"Unknown task: {self.task}")

        reward = (
            1.0
            + contact_reward
            + task_reward
            - self.upright_penalty_weight * upright_penalty
            - 0.02 * vel_penalty
            - self.action_penalty_weight * action_penalty
            - self.action_delta_penalty_weight * action_delta_penalty
            - 0.3 * posture_penalty
            - self.right_contact_penalty_weight * min(stats["right_contacts"], 4.0)
            - self.left_contact_penalty_weight * min(stats["left_contacts"], 4.0)
            - self.stability_excess_penalty_weight * stability_excess_penalty
            - (self.pose_tracking_penalty_weight - 0.3) * posture_penalty
        )
        terminated, _ = self._terminated()
        if terminated:
            reward -= self.fall_penalty
        return float(reward), {
            "reward_total": float(reward),
            "reward_contact": float(contact_reward),
            "reward_task": float(task_reward),
            "penalty_upright": float(upright_penalty),
            "penalty_joint_vel": float(vel_penalty),
            "penalty_action": float(action_penalty),
            "penalty_action_delta": float(action_delta_penalty),
            "penalty_right_contacts": float(min(stats["right_contacts"], 4.0)),
            "penalty_left_contacts": float(min(stats["left_contacts"], 4.0)),
            "penalty_fall": float(self.fall_penalty if terminated else 0.0),
            "penalty_stability_excess": float(stability_excess_penalty),
            "penalty_posture": float(posture_penalty),
            "penalty_flat_foot": float(flat_foot_penalty),
            "reward_swing_step": float(swing_step_reward),
            "reward_weight_shift": float(weight_shift_reward),
            "penalty_swing_contact": float(swing_contact_penalty),
            "penalty_foot_split": float(foot_split_penalty),
            "left_sole_tilt": float(self._foot_sole_tilt(self._left_foot_body_id)),
            "right_sole_tilt": float(self._foot_sole_tilt(self._right_foot_body_id)),
            "clearance_reward_gated": float(1.0 if (not self.gated_clearance_reward or stable_for_clearance) else 0.0)
            if self.task in {"right_clearance", "left_clearance"}
            else 0.0,
            "left_clearance": float(self._left_foot_clearance()),
            "right_clearance": float(clearance),
            **{key: float(value) for key, value in stats.items()},
        }

    def _terminated(self) -> tuple[bool, str]:
        roll, pitch, _ = quat_to_rpy(np.asarray(self.data.qpos[3:7], dtype=np.float64))
        if abs(roll) > self.termination_roll_limit:
            return True, "roll_limit"
        if abs(pitch) > self.termination_pitch_limit:
            return True, "pitch_limit"
        if float(self.data.qpos[2]) < self.base_z - self.termination_base_drop:
            return True, "base_height_drop"
        if not np.all(np.isfinite(self.data.qpos)) or not np.all(np.isfinite(self.data.qvel)):
            return True, "nonfinite_state"
        return False, ""

    def reset(
        self,
        *,
        seed: int | None = None,
        options: dict[str, Any] | None = None,
    ) -> tuple[np.ndarray, dict[str, Any]]:
        super().reset(seed=seed)
        if options:
            self.task = str(options.get("task", self.task))
        self._init_state()
        self.prev_action[:] = 0.0
        self._step_count = 0
        self._prev_left_foot_y = self._left_foot_y()
        self._prev_right_foot_y = self._right_foot_y()
        return self._obs(), self._info("")

    def step(self, action: np.ndarray) -> tuple[np.ndarray, float, bool, bool, dict[str, Any]]:
        action = np.clip(np.asarray(action, dtype=np.float32), -1.0, 1.0)
        target = self.nominal_q + self.action_scale * action.astype(np.float64)
        target = np.clip(target, self.joint_range[:, 0], self.joint_range[:, 1])
        tau = np.zeros(int(self.model.nu), dtype=np.float64)
        for _ in range(self.frame_skip):
            q = self.data.qpos[self.joint_qposadr]
            qd = self.data.qvel[self.joint_dofadr]
            roll, pitch, _ = quat_to_rpy(np.asarray(self.data.qpos[3:7], dtype=np.float64))
            tau = self.kp * (target - q) - self.kd * qd
            tau += self._nominal_stabilizer_torque(roll, pitch)
            tau = np.clip(tau, -self.torque_limit, self.torque_limit)
            self.data.ctrl[:] = tau
            mujoco.mj_step(self.model, self.data)

        self.prev_action = action.copy()
        self._step_count += 1
        reward, reward_info = self._reward(action)
        terminated, reason = self._terminated()
        truncated = self._step_count >= self.max_episode_steps
        info = self._info(reason)
        info.update(reward_info)
        info["max_abs_tau_cmd"] = float(np.max(np.abs(tau))) if tau.size else 0.0
        return self._obs(), reward, terminated, truncated, info

    def _info(self, terminated_reason: str) -> dict[str, Any]:
        roll, pitch, yaw = quat_to_rpy(np.asarray(self.data.qpos[3:7], dtype=np.float64))
        stats = self._contact_stats()
        return {
            "task": self.task,
            "step_count": self._step_count,
            "sim_time": float(self.data.time),
            "base_z": float(self.data.qpos[2]),
            "roll": float(roll),
            "pitch": float(pitch),
            "yaw": float(yaw),
            "left_clearance": float(self._left_foot_clearance()),
            "right_clearance": float(self._right_foot_clearance()),
            "terminated_reason": terminated_reason,
            **{key: float(value) for key, value in stats.items()},
        }

    def render(self) -> np.ndarray | None:
        if self.render_mode != "rgb_array":
            return None
        if self._renderer is None:
            self._renderer = mujoco.Renderer(self.model, height=self.height, width=self.width)
        camera = mujoco.MjvCamera()
        camera.type = mujoco.mjtCamera.mjCAMERA_FREE
        camera.lookat[:] = np.array([0.0, 0.0, max(float(self.data.qpos[2]), 0.2)])
        camera.distance = 1.35
        camera.azimuth = 135.0
        camera.elevation = -15.0
        self._renderer.update_scene(self.data, camera=camera)
        return self._renderer.render()

    def close(self) -> None:
        if self._renderer is not None:
            self._renderer.close()
            self._renderer = None
