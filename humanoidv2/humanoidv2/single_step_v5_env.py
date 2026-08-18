"""One forward step on the symmetric-inertia robot model."""

from __future__ import annotations

from dataclasses import replace
from pathlib import Path
from typing import Any

import mujoco
import numpy as np

from .khr3hv_env import KHR3HVEnv, KHRConfig


class SingleStepV5Env(KHR3HVEnv):
    """Double support -> single support -> forward landing -> double support."""

    _target_cache: dict[tuple[str, str, float, float, float], tuple[np.ndarray, ...]] = {}

    def __init__(
        self,
        model_path: str | Path | None = None,
        render_mode: str | None = None,
        reference_only: bool = False,
        fixed_side: str | None = None,
        stride_m: float = 0.030,
        sway_width_m: float = 0.090,
        lift_m: float = 0.030,
        kp: float = 80.0,
        landing_steps: int = 50,
    ) -> None:
        root = Path(__file__).resolve().parents[1]
        if model_path is None:
            model_path = root / "models/urdf_f_v2/URDF_F_v2_footprint_contact_symmetric_inertia.xml"
        self.fixed_side = fixed_side
        self.swing_side = "right"
        self.stride_m = stride_m
        self.sway_width_m = sway_width_m
        self.lift_m = lift_m
        self.kp = kp
        self.kd = 0.004 * kp
        self.lift_steps = 100
        self.advance_steps = 50
        self.landing_steps = landing_steps
        self.settle_steps = 50
        episode_steps = self.lift_steps + self.advance_steps + self.landing_steps + self.settle_steps
        config = replace(
            KHRConfig(),
            enhanced_collisions=True,
            max_episode_steps=episode_steps,
            torque_penalty_scale=0.002,
        )
        super().__init__(model_path=model_path, render_mode=render_mode, config=config, reference_only=reference_only)
        self._targets = {
            side: self._build_step_targets(side) for side in ("left", "right")
        }
        self._advance_forces: list[float] = []
        self._final_left_forces: list[float] = []
        self._final_right_forces: list[float] = []
        self._peak_landing_force = 0.0
        self._start_base_y = float(self.home_qpos[1])

    @staticmethod
    def _smoothstep(value: float) -> float:
        return value * value * (3.0 - 2.0 * value)

    def _build_step_targets(self, side: str) -> tuple[np.ndarray, ...]:
        key = (str(self.model_path), side, self.stride_m, self.sway_width_m, self.lift_m)
        if key in self._target_cache:
            return tuple(target.copy() for target in self._target_cache[key])
        base_qpos = self.home_qpos.copy()
        home = base_qpos[self.qpos_ids].copy()
        self.data.qpos[:] = base_qpos
        mujoco.mj_forward(self.model, self.data)
        starts = {
            "left": self._foot_output(self.left_foot_id).copy(),
            "right": self._foot_output(self.right_foot_id).copy(),
        }
        lateral_sign = 1.0 if side == "left" else -1.0

        def solve(lateral: float, lift: float, forward_fraction: float, seed: np.ndarray) -> np.ndarray:
            left_target = starts["left"].copy()
            right_target = starts["right"].copy()
            left_target[0] += lateral_sign * lateral
            right_target[0] += lateral_sign * lateral
            half_stride = 0.5 * self.stride_m * forward_fraction
            if side == "left":
                left_target[1] += half_stride
                right_target[1] -= half_stride
                left_target[2] += lift
            else:
                right_target[1] += half_stride
                left_target[1] -= half_stride
                right_target[2] += lift
            joints = self._solve_leg_ik(
                base_qpos, seed, slice(0, 5), self.left_foot_id, left_target
            )
            return self._solve_leg_ik(
                base_qpos, joints, slice(5, 10), self.right_foot_id, right_target
            )

        lifted = solve(self.sway_width_m, self.lift_m, 0.0, home)
        advanced = solve(self.sway_width_m, self.lift_m, 1.0, lifted)
        landed = solve(0.0, 0.0, 1.0, advanced)
        self._target_cache[key] = (home.copy(), lifted, advanced, landed)
        return tuple(target.copy() for target in self._target_cache[key])

    def _reference_target(self) -> tuple[np.ndarray, str, float]:
        home, lifted, advanced, landed = self._targets[self.swing_side]
        step = self._step_count
        if step < self.lift_steps:
            start, end, duration, phase, offset = home, lifted, self.lift_steps, "lift", 0
        elif step < self.lift_steps + self.advance_steps:
            start, end, duration, phase = lifted, advanced, self.advance_steps, "advance"
            offset = self.lift_steps
        elif step < self.lift_steps + self.advance_steps + self.landing_steps:
            start, end, duration, phase = advanced, landed, self.landing_steps, "land"
            offset = self.lift_steps + self.advance_steps
        else:
            start, end, duration, phase = landed, landed, self.settle_steps, "settle"
            offset = self.lift_steps + self.advance_steps + self.landing_steps
        fraction = min((step - offset + 1) / duration, 1.0)
        blend = self._smoothstep(fraction)
        return start + blend * (end - start), phase, fraction

    def _task_observation(self, advance_history: bool = True) -> np.ndarray:
        if advance_history:
            self._history[:-1] = self._history[1:]
            self._history[-1] = self._sensor_frame()
        side = 1.0 if self.swing_side == "right" else -1.0
        progress = min(self._step_count / self.config.max_episode_steps, 1.0)
        return np.concatenate((self._history.ravel(), [side, progress])).astype(np.float32)

    def reset(
        self,
        *,
        seed: int | None = None,
        options: dict[str, Any] | None = None,
    ) -> tuple[np.ndarray, dict[str, Any]]:
        _, info = super().reset(seed=seed, options=options)
        requested = (options or {}).get("side")
        self.swing_side = requested or self.fixed_side or (
            "right" if self.np_random.random() < 0.5 else "left"
        )
        self._advance_forces = []
        self._final_left_forces = []
        self._final_right_forces = []
        self._peak_landing_force = 0.0
        self._start_base_y = float(self.data.qpos[1])
        info.update({"swing_side": self.swing_side, "task_phase": "lift", "is_success": False})
        return self._task_observation(advance_history=False), info

    def step(self, action: np.ndarray):
        action = np.asarray(action, dtype=np.float64)
        if self.reference_only:
            action = np.zeros(10, dtype=np.float64)
        action = np.clip(action, -1.0, 1.0)
        self._filtered_action = (
            self.config.action_filter * action
            + (1.0 - self.config.action_filter) * self._filtered_action
        )
        reference_target, phase, phase_progress = self._reference_target()
        target = reference_target + 0.25 * self.action_scale * self._filtered_action
        target = np.clip(target, self.joint_ranges[:, 0], self.joint_ranges[:, 1])
        for _ in range(self.frame_skip):
            error = target - self.data.qpos[self.qpos_ids]
            torque = self.kp * error - self.kd * self.data.qvel[self.dof_ids]
            self.data.ctrl[:] = np.clip(torque, -self.torque_limit, self.torque_limit)
            mujoco.mj_step(self.model, self.data)
        self._step_count += 1

        left_contact, right_contact, left_force, right_force = self._foot_contacts()
        swing_force = left_force if self.swing_side == "left" else right_force
        stance_force = right_force if self.swing_side == "left" else left_force
        swing_id = self.left_foot_id if self.swing_side == "left" else self.right_foot_id
        stance_id = self.right_foot_id if self.swing_side == "left" else self.left_foot_id
        swing_height = float(self._sole_point(swing_id, True)[2])
        step_length = float(self.data.xpos[swing_id, 1] - self.data.xpos[stance_id, 1])
        if phase == "advance":
            self._advance_forces.append(swing_force)
        if phase == "land":
            self._peak_landing_force = max(self._peak_landing_force, swing_force)
        if phase == "settle":
            self._final_left_forces.append(left_force)
            self._final_right_forces.append(right_force)

        roll, pitch = self._roll_pitch(self.data.qpos[3:7])
        orientation_reward = float(np.exp(-40.0 * (roll * roll + pitch * pitch)))
        pose_error = self.data.qpos[self.qpos_ids] - target
        pose_reward = float(np.exp(-np.dot(pose_error, pose_error) / 0.05))
        unload_reward = float(np.exp(-swing_force / 5.0))
        stance_reward = float(np.exp(-((stance_force - 82.0) ** 2) / 800.0))
        double_support_reward = float(
            np.exp(-((left_force - 41.0) ** 2 + (right_force - 41.0) ** 2) / 800.0)
        )
        step_reward = float(np.clip(step_length / self.stride_m, 0.0, 1.0))
        if phase == "lift":
            unload_weight, contact_weight, step_weight = phase_progress, 0.0, 0.0
        elif phase == "advance":
            unload_weight, contact_weight, step_weight = 1.0, 0.0, 0.10
        elif phase == "land":
            unload_weight = 1.0 - phase_progress
            contact_weight, step_weight = phase_progress, 1.0
        else:
            unload_weight, contact_weight, step_weight = 0.0, 1.0, 1.0
        torso_height = float(self.data.xipos[self.base_id, 2])
        terminated = bool(torso_height < self.fall_height or not np.isfinite(self.data.qpos).all())
        truncated = bool(self._step_count >= self.config.max_episode_steps)
        torque_penalty = -self.config.torque_penalty_scale * float(np.abs(self.data.actuator_force).sum())
        reward = (
            0.20 * (0.0 if terminated else 1.0)
            + 0.15 * orientation_reward
            + 0.10 * pose_reward
            + 0.20 * unload_weight * unload_reward
            + 0.10 * unload_weight * stance_reward
            + 0.20 * contact_weight * double_support_reward
            + 0.10 * step_weight * step_reward
            + torque_penalty
            + (-10.0 if terminated else 0.0)
        )

        advance = np.asarray(self._advance_forces)
        final_left = np.asarray(self._final_left_forces)
        final_right = np.asarray(self._final_right_forces)
        final_both_fraction = float(
            np.mean((final_left > 10.0) & (final_right > 10.0))
        ) if len(final_left) else 0.0
        is_success = bool(
            truncated
            and not terminated
            and len(advance) == self.advance_steps
            and advance.mean() < 5.0
            and np.mean(advance < 5.0) >= 0.9
            and len(final_left) == self.settle_steps
            and final_both_fraction >= 0.9
            and step_length >= 0.020
        )
        info = {
            "swing_side": self.swing_side,
            "task_phase": phase,
            "phase_progress": phase_progress,
            "swing_force_n": swing_force,
            "stance_force_n": stance_force,
            "left_foot_force_n": left_force,
            "right_foot_force_n": right_force,
            "left_foot_contact": left_contact,
            "right_foot_contact": right_contact,
            "swing_height_m": swing_height,
            "step_length_m": step_length,
            "base_forward_displacement_m": float(self.data.qpos[1] - self._start_base_y),
            "peak_landing_force_n": self._peak_landing_force,
            "final_both_contact_fraction": final_both_fraction,
            "torso_height": torso_height,
            "orientation_reward": orientation_reward,
            "pose_reward": pose_reward,
            "unload_reward": unload_reward,
            "double_support_reward": double_support_reward,
            "torque_penalty": torque_penalty,
            "is_success": is_success,
        }
        return self._task_observation(), float(reward), terminated, truncated, info
