"""Two alternating forward steps on the symmetric-inertia robot."""

from __future__ import annotations

from dataclasses import replace
from pathlib import Path
from typing import Any

import mujoco
import numpy as np

from .khr3hv_env import KHR3HVEnv, KHRConfig


class TwoStepV6Env(KHR3HVEnv):
    """Execute first-side step, then advance the trailing opposite foot."""

    _target_cache: dict[tuple[str, str, float, float, float], tuple[np.ndarray, ...]] = {}

    def __init__(
        self,
        model_path: str | Path | None = None,
        render_mode: str | None = None,
        reference_only: bool = False,
        fixed_first_side: str | None = None,
        stride_m: float = 0.030,
        sway_width_m: float = 0.090,
        lift_m: float = 0.030,
        kp: float = 80.0,
    ) -> None:
        root = Path(__file__).resolve().parents[1]
        if model_path is None:
            model_path = root / "models/urdf_f_v2/URDF_F_v2_footprint_contact_symmetric_inertia.xml"
        self.fixed_first_side = fixed_first_side
        self.first_side = "left"
        self.second_side = "right"
        self.stride_m = stride_m
        self.sway_width_m = sway_width_m
        self.lift_m = lift_m
        self.kp = kp
        self.kd = 0.004 * kp
        self.lift_steps = 100
        self.advance_steps = 50
        self.land_steps = 50
        self.settle_steps = 50
        self.half_cycle_steps = 250
        config = replace(
            KHRConfig(),
            enhanced_collisions=True,
            max_episode_steps=2 * self.half_cycle_steps,
            torque_penalty_scale=0.002,
        )
        super().__init__(model_path=model_path, render_mode=render_mode, config=config, reference_only=reference_only)
        self._targets = {side: self._build_targets(side) for side in ("left", "right")}
        self._first_advance_forces: list[float] = []
        self._second_advance_forces: list[float] = []
        self._first_settle_left: list[float] = []
        self._first_settle_right: list[float] = []
        self._final_left: list[float] = []
        self._final_right: list[float] = []
        self._first_peak_landing_force = 0.0
        self._second_peak_landing_force = 0.0
        self._first_step_length = 0.0
        self._start_base_y = float(self.home_qpos[1])

    @staticmethod
    def _smoothstep(value: float) -> float:
        return value * value * (3.0 - 2.0 * value)

    def _build_targets(self, first_side: str) -> tuple[np.ndarray, ...]:
        key = (str(self.model_path), first_side, self.stride_m, self.sway_width_m, self.lift_m)
        if key in self._target_cache:
            return tuple(target.copy() for target in self._target_cache[key])
        second_side = "right" if first_side == "left" else "left"
        base_qpos = self.home_qpos.copy()
        home = base_qpos[self.qpos_ids].copy()
        self.data.qpos[:] = base_qpos
        mujoco.mj_forward(self.model, self.data)
        starts = {
            "left": self._foot_output(self.left_foot_id).copy(),
            "right": self._foot_output(self.right_foot_id).copy(),
        }

        def solve(
            lateral_sign: float,
            lateral: float,
            forward: dict[str, float],
            lifted_side: str | None,
            seed: np.ndarray,
        ) -> np.ndarray:
            targets = {side: starts[side].copy() for side in starts}
            for side in targets:
                targets[side][0] += lateral_sign * lateral
                targets[side][1] += forward[side]
            if lifted_side:
                targets[lifted_side][2] += self.lift_m
            joints = self._solve_leg_ik(
                base_qpos, seed, slice(0, 5), self.left_foot_id, targets["left"]
            )
            return self._solve_leg_ik(
                base_qpos, joints, slice(5, 10), self.right_foot_id, targets["right"]
            )

        first_sign = 1.0 if first_side == "left" else -1.0
        centered = {"left": 0.0, "right": 0.0}
        first_split = {first_side: 0.5 * self.stride_m, second_side: -0.5 * self.stride_m}
        second_split = {first_side: -0.5 * self.stride_m, second_side: 0.5 * self.stride_m}
        first_lifted = solve(first_sign, self.sway_width_m, centered, first_side, home)
        first_advanced = solve(first_sign, self.sway_width_m, first_split, first_side, first_lifted)
        first_landed = solve(0.0, 0.0, first_split, None, first_advanced)
        second_lifted = solve(-first_sign, self.sway_width_m, first_split, second_side, first_landed)
        second_advanced = solve(-first_sign, self.sway_width_m, second_split, second_side, second_lifted)
        second_landed = solve(0.0, 0.0, second_split, None, second_advanced)
        self._target_cache[key] = (
            home.copy(), first_lifted, first_advanced, first_landed,
            second_lifted, second_advanced, second_landed,
        )
        return tuple(target.copy() for target in self._target_cache[key])

    def _reference_target(self) -> tuple[np.ndarray, str, float, str]:
        home, first_lifted, first_advanced, first_landed, second_lifted, second_advanced, second_landed = self._targets[self.first_side]
        segments = (
            (home, first_lifted, self.lift_steps, "first_lift", self.first_side),
            (first_lifted, first_advanced, self.advance_steps, "first_advance", self.first_side),
            (first_advanced, first_landed, self.land_steps, "first_land", self.first_side),
            (first_landed, first_landed, self.settle_steps, "first_settle", self.first_side),
            (first_landed, second_lifted, self.lift_steps, "second_lift", self.second_side),
            (second_lifted, second_advanced, self.advance_steps, "second_advance", self.second_side),
            (second_advanced, second_landed, self.land_steps, "second_land", self.second_side),
            (second_landed, second_landed, self.settle_steps, "second_settle", self.second_side),
        )
        offset = 0
        for start, end, duration, phase, active_side in segments:
            if self._step_count < offset + duration:
                fraction = min((self._step_count - offset + 1) / duration, 1.0)
                blend = self._smoothstep(fraction)
                return start + blend * (end - start), phase, fraction, active_side
            offset += duration
        return second_landed.copy(), "second_settle", 1.0, self.second_side

    def _task_observation(self, advance_history: bool = True) -> np.ndarray:
        if advance_history:
            self._history[:-1] = self._history[1:]
            self._history[-1] = self._sensor_frame()
        active_side = self.first_side if self._step_count < self.half_cycle_steps else self.second_side
        side = 1.0 if active_side == "right" else -1.0
        progress = min(self._step_count / self.config.max_episode_steps, 1.0)
        return np.concatenate((self._history.ravel(), [side, progress])).astype(np.float32)

    def _step_length(self, side: str) -> float:
        swing_id = self.left_foot_id if side == "left" else self.right_foot_id
        other_id = self.right_foot_id if side == "left" else self.left_foot_id
        return float(self.data.xpos[swing_id, 1] - self.data.xpos[other_id, 1])

    def reset(
        self,
        *,
        seed: int | None = None,
        options: dict[str, Any] | None = None,
    ) -> tuple[np.ndarray, dict[str, Any]]:
        _, info = super().reset(seed=seed, options=options)
        requested = (options or {}).get("first_side")
        self.first_side = requested or self.fixed_first_side or (
            "right" if self.np_random.random() < 0.5 else "left"
        )
        self.second_side = "right" if self.first_side == "left" else "left"
        self._first_advance_forces = []
        self._second_advance_forces = []
        self._first_settle_left = []
        self._first_settle_right = []
        self._final_left = []
        self._final_right = []
        self._first_peak_landing_force = 0.0
        self._second_peak_landing_force = 0.0
        self._first_step_length = 0.0
        self._start_base_y = float(self.data.qpos[1])
        info.update({
            "first_side": self.first_side,
            "second_side": self.second_side,
            "task_phase": "first_lift",
            "is_success": False,
        })
        return self._task_observation(advance_history=False), info

    def step(self, action: np.ndarray):
        action = np.asarray(action, dtype=np.float64)
        if self.reference_only:
            action = np.zeros(10, dtype=np.float64)
        action = np.clip(action, -1.0, 1.0)
        self._filtered_action = self.config.action_filter * action + (
            1.0 - self.config.action_filter
        ) * self._filtered_action
        reference_target, phase, phase_progress, active_side = self._reference_target()
        target = reference_target + 0.25 * self.action_scale * self._filtered_action
        target = np.clip(target, self.joint_ranges[:, 0], self.joint_ranges[:, 1])
        for _ in range(self.frame_skip):
            error = target - self.data.qpos[self.qpos_ids]
            torque = self.kp * error - self.kd * self.data.qvel[self.dof_ids]
            self.data.ctrl[:] = np.clip(torque, -self.torque_limit, self.torque_limit)
            mujoco.mj_step(self.model, self.data)
        self._step_count += 1

        left_contact, right_contact, left_force, right_force = self._foot_contacts()
        swing_force = left_force if active_side == "left" else right_force
        stance_force = right_force if active_side == "left" else left_force
        swing_id = self.left_foot_id if active_side == "left" else self.right_foot_id
        swing_height = float(self._sole_point(swing_id, True)[2])
        active_step_length = self._step_length(active_side)
        if phase == "first_advance":
            self._first_advance_forces.append(swing_force)
        elif phase == "second_advance":
            self._second_advance_forces.append(swing_force)
        elif phase == "first_land":
            self._first_peak_landing_force = max(self._first_peak_landing_force, swing_force)
        elif phase == "second_land":
            self._second_peak_landing_force = max(self._second_peak_landing_force, swing_force)
        elif phase == "first_settle":
            self._first_settle_left.append(left_force)
            self._first_settle_right.append(right_force)
            self._first_step_length = active_step_length
        elif phase == "second_settle":
            self._final_left.append(left_force)
            self._final_right.append(right_force)

        phase_kind = phase.split("_", 1)[1]
        roll, pitch = self._roll_pitch(self.data.qpos[3:7])
        orientation_reward = float(np.exp(-40.0 * (roll * roll + pitch * pitch)))
        pose_error = self.data.qpos[self.qpos_ids] - target
        pose_reward = float(np.exp(-np.dot(pose_error, pose_error) / 0.05))
        unload_reward = float(np.exp(-swing_force / 5.0))
        stance_reward = float(np.exp(-((stance_force - 82.0) ** 2) / 800.0))
        double_support_reward = float(
            np.exp(-((left_force - 41.0) ** 2 + (right_force - 41.0) ** 2) / 800.0)
        )
        step_reward = float(np.clip(active_step_length / self.stride_m, 0.0, 1.0))
        if phase_kind == "lift":
            unload_weight, contact_weight, step_weight = phase_progress, 0.0, 0.0
        elif phase_kind == "advance":
            unload_weight, contact_weight, step_weight = 1.0, 0.0, 0.10
        elif phase_kind == "land":
            unload_weight, contact_weight, step_weight = 1.0 - phase_progress, phase_progress, 1.0
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

        first_advance = np.asarray(self._first_advance_forces)
        second_advance = np.asarray(self._second_advance_forces)
        first_left = np.asarray(self._first_settle_left)
        first_right = np.asarray(self._first_settle_right)
        final_left = np.asarray(self._final_left)
        final_right = np.asarray(self._final_right)
        first_both_fraction = float(np.mean((first_left > 10.0) & (first_right > 10.0))) if len(first_left) else 0.0
        final_both_fraction = float(np.mean((final_left > 10.0) & (final_right > 10.0))) if len(final_left) else 0.0
        base_forward = float(self.data.qpos[1] - self._start_base_y)
        is_success = bool(
            truncated
            and not terminated
            and len(first_advance) == self.advance_steps
            and len(second_advance) == self.advance_steps
            and first_advance.mean() < 5.0
            and second_advance.mean() < 5.0
            and np.mean(first_advance < 5.0) >= 0.9
            and np.mean(second_advance < 5.0) >= 0.9
            and first_both_fraction >= 0.9
            and final_both_fraction >= 0.9
            and self._first_step_length >= 0.020
            and active_step_length >= 0.020
            and base_forward >= 0.035
        )
        info = {
            "first_side": self.first_side,
            "second_side": self.second_side,
            "active_swing_side": active_side,
            "task_phase": phase,
            "phase_progress": phase_progress,
            "swing_force_n": swing_force,
            "stance_force_n": stance_force,
            "left_foot_force_n": left_force,
            "right_foot_force_n": right_force,
            "left_foot_contact": left_contact,
            "right_foot_contact": right_contact,
            "swing_height_m": swing_height,
            "active_step_length_m": active_step_length,
            "first_step_length_m": self._first_step_length,
            "base_forward_displacement_m": base_forward,
            "first_peak_landing_force_n": self._first_peak_landing_force,
            "second_peak_landing_force_n": self._second_peak_landing_force,
            "first_both_contact_fraction": first_both_fraction,
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
