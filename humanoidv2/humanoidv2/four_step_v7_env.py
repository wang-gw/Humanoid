"""Four alternating forward steps on the symmetric-inertia robot."""

from __future__ import annotations

from dataclasses import replace
from pathlib import Path
from typing import Any

import mujoco
import numpy as np

from .khr3hv_env import KHR3HVEnv, KHRConfig


class FourStepV7Env(KHR3HVEnv):
    """Execute four reset-free steps while carrying each landing into the next lift."""

    # Historical V7-V12 experiments assumed +Y was the robot's forward
    # direction. Keep that default for reproducibility; corrected environments
    # override this class attribute with -1.0.
    forward_sign = 1.0
    residual_action_scale = 0.25
    _target_cache: dict[tuple[str, str, float, float, float, float], tuple[np.ndarray, ...]] = {}

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
        self.stride_m = stride_m
        self.sway_width_m = sway_width_m
        self.lift_m = lift_m
        self.kp = kp
        self.kd = 0.004 * kp
        self.num_steps = 4
        self.lift_steps = 100
        self.lift_hold_steps = 0
        self.advance_steps = 50
        self.land_steps = 50
        self.settle_steps = 50
        self.step_cycle_steps = 250
        self.minimum_forward_m = 0.070
        config = replace(
            self.base_config(),
            enhanced_collisions=True,
            max_episode_steps=self.num_steps * self.step_cycle_steps,
            torque_penalty_scale=0.002,
        )
        super().__init__(
            model_path=model_path,
            render_mode=render_mode,
            config=config,
            reference_only=reference_only,
        )
        self._targets = {side: self._build_targets(side) for side in ("left", "right")}
        self._advance_forces: list[list[float]] = []
        self._settle_left: list[list[float]] = []
        self._settle_right: list[list[float]] = []
        self._landing_peaks = np.zeros(self.num_steps, dtype=np.float64)
        self._step_lengths = np.zeros(self.num_steps, dtype=np.float64)
        self._start_base_y = float(self.home_qpos[1])

    @staticmethod
    def _smoothstep(value: float) -> float:
        return value * value * (3.0 - 2.0 * value)

    def _step_sides(self, first_side: str | None = None) -> tuple[str, ...]:
        first = first_side or self.first_side
        second = "right" if first == "left" else "left"
        return first, second, first, second

    def _build_targets(self, first_side: str) -> tuple[np.ndarray, ...]:
        key = (
            str(self.model_path),
            first_side,
            self.stride_m,
            self.sway_width_m,
            self.lift_m,
            self.forward_sign,
        )
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
            forward: dict[str, float],
            lifted_side: str | None,
            seed: np.ndarray,
        ) -> np.ndarray:
            targets = {side: starts[side].copy() for side in starts}
            for side in targets:
                targets[side][0] += lateral_sign * self.sway_width_m
                targets[side][1] += forward[side]
            if lifted_side:
                targets[lifted_side][2] += self.lift_m
            joints = self._solve_leg_ik(
                base_qpos, seed, slice(0, 5), self.left_foot_id, targets["left"]
            )
            return self._solve_leg_ik(
                base_qpos, joints, slice(5, 10), self.right_foot_id, targets["right"]
            )

        centered = {"left": 0.0, "right": 0.0}
        half_stride = 0.5 * self.forward_sign * self.stride_m
        first_forward = {first_side: half_stride, second_side: -half_stride}
        second_forward = {first_side: -half_stride, second_side: half_stride}
        current_forward = centered
        seed = home
        targets: list[np.ndarray] = [home.copy()]
        for step_index, active_side in enumerate(self._step_sides(first_side)):
            next_forward = first_forward if step_index % 2 == 0 else second_forward
            lateral_sign = 1.0 if active_side == "left" else -1.0
            lifted = solve(lateral_sign, current_forward, active_side, seed)
            advanced = solve(lateral_sign, next_forward, active_side, lifted)
            landed = solve(0.0, next_forward, None, advanced)
            targets.extend((lifted, advanced, landed))
            current_forward = next_forward
            seed = landed

        self._target_cache[key] = tuple(target.copy() for target in targets)
        self.data.qpos[:] = base_qpos
        mujoco.mj_forward(self.model, self.data)
        return tuple(target.copy() for target in self._target_cache[key])

    def _reference_target(self) -> tuple[np.ndarray, str, float, str, int]:
        step_index = min(self._step_count // self.step_cycle_steps, self.num_steps - 1)
        local_step = self._step_count - step_index * self.step_cycle_steps
        active_side = self._step_sides()[step_index]
        start = self._targets[self.first_side][0] if step_index == 0 else self._targets[self.first_side][3 * step_index]
        lifted, advanced, landed = self._targets[self.first_side][1 + 3 * step_index:4 + 3 * step_index]
        segments = [(start, lifted, self.lift_steps, "lift")]
        if self.lift_hold_steps:
            segments.append((lifted, lifted, self.lift_hold_steps, "hold"))
        segments.extend(
            (
                (lifted, advanced, self.advance_steps, "advance"),
                (advanced, landed, self.land_steps, "land"),
                (landed, landed, self.settle_steps, "settle"),
            )
        )
        offset = 0
        for segment_start, segment_end, duration, phase_kind in segments:
            if local_step < offset + duration:
                fraction = min((local_step - offset + 1) / duration, 1.0)
                blend = self._smoothstep(fraction)
                target = segment_start + blend * (segment_end - segment_start)
                return target, f"step{step_index + 1}_{phase_kind}", fraction, active_side, step_index
            offset += duration
        return landed.copy(), f"step{step_index + 1}_settle", 1.0, active_side, step_index

    def _task_observation(self, advance_history: bool = True) -> np.ndarray:
        if advance_history:
            self._history[:-1] = self._history[1:]
            self._history[-1] = self._sensor_frame()
        step_index = min(self._step_count // self.step_cycle_steps, self.num_steps - 1)
        active_side = self._step_sides()[step_index]
        side = 1.0 if active_side == "right" else -1.0
        progress = min(self._step_count / self.config.max_episode_steps, 1.0)
        return np.concatenate((self._history.ravel(), [side, progress])).astype(np.float32)

    def _step_length(self, side: str) -> float:
        swing_id = self.left_foot_id if side == "left" else self.right_foot_id
        other_id = self.right_foot_id if side == "left" else self.left_foot_id
        return float(
            self.forward_sign
            * (self.data.xpos[swing_id, 1] - self.data.xpos[other_id, 1])
        )

    def _phase_pd_gains(self, phase_kind: str, phase_progress: float) -> tuple[float, float]:
        """Return phase-specific gains; historical environments use fixed gains."""
        return self.kp, self.kd

    def _residual_joint_target_offset(
        self, filtered_action: np.ndarray, phase_kind: str, phase_progress: float
    ) -> np.ndarray:
        """Convert the paper-style normalized residual to a joint target offset."""
        return self.residual_action_scale * self.action_scale * filtered_action

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
        self._advance_forces = [[] for _ in range(self.num_steps)]
        self._settle_left = [[] for _ in range(self.num_steps)]
        self._settle_right = [[] for _ in range(self.num_steps)]
        self._landing_peaks = np.zeros(self.num_steps, dtype=np.float64)
        self._step_lengths = np.zeros(self.num_steps, dtype=np.float64)
        self._start_base_y = float(self.data.qpos[1])
        info.update({
            "forward_axis": "-Y" if self.forward_sign < 0.0 else "+Y",
            "forward_sign": self.forward_sign,
            "first_side": self.first_side,
            "step_order": "->".join(self._step_sides()),
            "active_step": 1,
            "active_swing_side": self.first_side,
            "task_phase": "step1_lift",
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
        reference_target, phase, phase_progress, active_side, step_index = self._reference_target()
        phase_kind = phase.split("_", 1)[1]
        target = reference_target + self._residual_joint_target_offset(
            self._filtered_action, phase_kind, phase_progress
        )
        target = np.clip(target, self.joint_ranges[:, 0], self.joint_ranges[:, 1])
        effective_kp, effective_kd = self._phase_pd_gains(phase_kind, phase_progress)
        peak_actuator_torque = 0.0
        saturated_actuator_samples = 0
        for _ in range(self.frame_skip):
            error = target - self.data.qpos[self.qpos_ids]
            torque = effective_kp * error - effective_kd * self.data.qvel[self.dof_ids]
            self.data.ctrl[:] = np.clip(torque, -self.torque_limit, self.torque_limit)
            peak_actuator_torque = max(
                peak_actuator_torque, float(np.max(np.abs(self.data.ctrl)))
            )
            saturated_actuator_samples += int(
                np.count_nonzero(np.abs(self.data.ctrl) >= self.torque_limit - 1.0e-9)
            )
            mujoco.mj_step(self.model, self.data)
        self._step_count += 1

        left_contact, right_contact, left_force, right_force = self._foot_contacts()
        swing_force = left_force if active_side == "left" else right_force
        stance_force = right_force if active_side == "left" else left_force
        swing_id = self.left_foot_id if active_side == "left" else self.right_foot_id
        swing_height = float(self._sole_point(swing_id, True)[2])
        active_step_length = self._step_length(active_side)
        if phase_kind == "advance":
            self._advance_forces[step_index].append(swing_force)
        elif phase_kind == "land":
            self._landing_peaks[step_index] = max(self._landing_peaks[step_index], swing_force)
        elif phase_kind == "settle":
            self._settle_left[step_index].append(left_force)
            self._settle_right[step_index].append(right_force)
            self._step_lengths[step_index] = active_step_length

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
        elif phase_kind == "hold":
            unload_weight, contact_weight, step_weight = 1.0, 0.0, 0.0
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

        # Keep partial-episode diagnostics valid JSON. Completion is checked separately
        # through the exact number of advance samples, so zero is only a placeholder
        # until a step reaches its advance phase.
        advance_means = [float(np.mean(values)) if values else 0.0 for values in self._advance_forces]
        advance_under = [float(np.mean(np.asarray(values) < 5.0)) if values else 0.0 for values in self._advance_forces]
        settle_fractions = [
            float(np.mean((np.asarray(left) > 10.0) & (np.asarray(right) > 10.0))) if left else 0.0
            for left, right in zip(self._settle_left, self._settle_right)
        ]
        base_forward = float(
            self.forward_sign * (self.data.qpos[1] - self._start_base_y)
        )
        is_success = bool(
            truncated
            and not terminated
            and all(len(values) == self.advance_steps for values in self._advance_forces)
            and all(value < 5.0 for value in advance_means)
            and all(value >= 0.9 for value in advance_under)
            and all(value >= 0.9 for value in settle_fractions)
            and np.all(self._step_lengths >= 0.020)
            and base_forward >= self.minimum_forward_m
        )
        info: dict[str, Any] = {
            "forward_axis": "-Y" if self.forward_sign < 0.0 else "+Y",
            "forward_sign": self.forward_sign,
            "residual_action_scale": self.residual_action_scale,
            "effective_kp": effective_kp,
            "effective_kd": effective_kd,
            "peak_control_interval_torque_n_m": peak_actuator_torque,
            "actuator_saturation_fraction": saturated_actuator_samples
            / (self.frame_skip * len(self.torque_limit)),
            "first_side": self.first_side,
            "step_order": "->".join(self._step_sides()),
            "active_step": step_index + 1,
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
            "base_forward_displacement_m": base_forward,
            "torso_height": torso_height,
            "orientation_reward": orientation_reward,
            "pose_reward": pose_reward,
            "unload_reward": unload_reward,
            "double_support_reward": double_support_reward,
            "torque_penalty": torque_penalty,
            "is_success": is_success,
        }
        for index in range(self.num_steps):
            info[f"step_{index + 1}_advance_mean_force_n"] = advance_means[index]
            info[f"step_{index + 1}_advance_under_5n_fraction"] = advance_under[index]
            info[f"step_{index + 1}_length_m"] = float(self._step_lengths[index])
            info[f"step_{index + 1}_both_contact_fraction"] = settle_fractions[index]
            info[f"step_{index + 1}_peak_landing_force_n"] = float(self._landing_peaks[index])
        return self._task_observation(), float(reward), terminated, truncated, info
