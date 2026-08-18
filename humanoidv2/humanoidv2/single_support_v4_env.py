"""Static single-support curriculum preceding continuous walking."""

from __future__ import annotations

from dataclasses import replace
from pathlib import Path
from typing import Any

import mujoco
import numpy as np

from .khr3hv_env import KHR3HVEnv, KHRConfig


class SingleSupportV4Env(KHR3HVEnv):
    """Transition to and hold a randomly requested left/right swing foot."""

    _target_cache: dict[tuple[str, str, str], np.ndarray] = {}

    def __init__(
        self,
        model_path: str | Path | None = None,
        render_mode: str | None = None,
        reference_only: bool = False,
        fixed_side: str | None = None,
        task_profile: str = "original",
    ) -> None:
        if task_profile not in ("original", "symmetric"):
            raise ValueError("task_profile must be 'original' or 'symmetric'")
        self.task_profile = task_profile
        config = replace(
            KHRConfig(),
            enhanced_collisions=True,
            max_episode_steps=200,
            torque_penalty_scale=0.002,
        )
        super().__init__(model_path=model_path, render_mode=render_mode, config=config, reference_only=reference_only)
        self.fixed_side = fixed_side
        self.swing_side = "right"
        self.ramp_steps = 100
        self._start_joints = self.home_qpos[self.qpos_ids].copy()
        self._recent_swing_forces: list[float] = []
        self._targets = self._single_support_targets()

    def _single_support_targets(self) -> dict[str, np.ndarray]:
        if self.task_profile == "symmetric":
            specifications = {"right": (0.090, 0.035, 37), "left": (0.090, 0.035, 12)}
        else:
            specifications = {"right": (0.060, 0.035, 37), "left": (0.120, 0.055, 12)}
        targets = {}
        for side, (width, height, phase) in specifications.items():
            cache_key = (str(self.model_path), self.task_profile, side)
            if cache_key in self._target_cache:
                targets[side] = self._target_cache[cache_key].copy()
                continue
            target_config = replace(
                KHRConfig(),
                enhanced_collisions=True,
                sway_width=width,
                sway_offset=0.01,
                foot_height=height,
                foot_height_offset=0.005,
            )
            target_env = KHR3HVEnv(model_path=self.model_path, config=target_config)
            self._target_cache[cache_key] = target_env.reference[phase].copy()
            targets[side] = self._target_cache[cache_key].copy()
            target_env.close()
        return targets

    def _pd_gains(self) -> tuple[float, float]:
        if self.task_profile == "symmetric":
            return 80.0, 0.32
        return (50.0, 0.20) if self.swing_side == "right" else (80.0, 0.32)

    def _task_observation(self, advance_history: bool = True) -> np.ndarray:
        if advance_history:
            self._history[:-1] = self._history[1:]
            self._history[-1] = self._sensor_frame()
        side = 1.0 if self.swing_side == "right" else -1.0
        progress = min(self._step_count / self.ramp_steps, 1.0)
        return np.concatenate((self._history.ravel(), [side, progress])).astype(np.float32)

    def reset(
        self,
        *,
        seed: int | None = None,
        options: dict[str, Any] | None = None,
    ) -> tuple[np.ndarray, dict[str, Any]]:
        observation, info = super().reset(seed=seed, options=options)
        requested = (options or {}).get("side")
        self.swing_side = requested or self.fixed_side or ("right" if self.np_random.random() < 0.5 else "left")
        self._start_joints = self.data.qpos[self.qpos_ids].copy()
        self._recent_swing_forces = []
        info.update({"swing_side": self.swing_side, "is_success": False})
        return self._task_observation(advance_history=False), info

    def step(self, action: np.ndarray):
        action = np.asarray(action, dtype=np.float64)
        if self.reference_only:
            action = np.zeros(10, dtype=np.float64)
        action = np.clip(action, -1.0, 1.0)
        b = self.config.action_filter
        self._filtered_action = b * action + (1.0 - b) * self._filtered_action
        fraction = min((self._step_count + 1) / self.ramp_steps, 1.0)
        blend = fraction * fraction * (3.0 - 2.0 * fraction)
        target = self._start_joints + blend * (self._targets[self.swing_side] - self._start_joints)
        target += 0.25 * self.action_scale * self._filtered_action
        target = np.clip(target, self.joint_ranges[:, 0], self.joint_ranges[:, 1])
        kp, kd = self._pd_gains()
        for _ in range(self.frame_skip):
            error = target - self.data.qpos[self.qpos_ids]
            torque = kp * error - kd * self.data.qvel[self.dof_ids]
            self.data.ctrl[:] = np.clip(torque, -self.torque_limit, self.torque_limit)
            mujoco.mj_step(self.model, self.data)
        self._step_count += 1

        left_contact, right_contact, left_force, right_force = self._foot_contacts()
        swing_force = right_force if self.swing_side == "right" else left_force
        stance_force = left_force if self.swing_side == "right" else right_force
        swing_height = float(
            self._sole_point(self.right_foot_id if self.swing_side == "right" else self.left_foot_id, True)[2]
        )
        self._recent_swing_forces.append(swing_force)
        self._recent_swing_forces = self._recent_swing_forces[-50:]
        roll, pitch = self._roll_pitch(self.data.qpos[3:7])
        orientation_reward = float(np.exp(-40.0 * (roll * roll + pitch * pitch)))
        unload_reward = float(np.exp(-swing_force / 5.0))
        stance_reward = float(np.exp(-((stance_force - 82.0) ** 2) / 800.0))
        pose_error = self.data.qpos[self.qpos_ids] - target
        pose_reward = float(np.exp(-np.dot(pose_error, pose_error) / 0.05))
        torque_penalty = -self.config.torque_penalty_scale * float(np.abs(self.data.actuator_force).sum())
        torso_height = float(self.data.xipos[self.base_id, 2])
        terminated = bool(torso_height < self.fall_height or not np.isfinite(self.data.qpos).all())
        truncated = bool(self._step_count >= self.config.max_episode_steps)
        reward = (
            0.45 * blend * unload_reward
            + 0.10 * stance_reward
            + 0.10 * orientation_reward
            + 0.10 * pose_reward
            + 0.20 * (0.0 if terminated else 1.0)
            + torque_penalty
            + (-10.0 if terminated else 0.0)
        )
        hold_forces = np.asarray(self._recent_swing_forces)
        is_success = bool(
            truncated
            and not terminated
            and len(hold_forces) == 50
            and hold_forces.mean() < 5.0
            and np.mean(hold_forces < 5.0) >= 0.9
            and stance_force > 30.0
        )
        info = {
            "swing_side": self.swing_side,
            "task_profile": self.task_profile,
            "transition_progress": fraction,
            "swing_force_n": swing_force,
            "stance_force_n": stance_force,
            "swing_height_m": swing_height,
            "left_foot_contact": left_contact,
            "right_foot_contact": right_contact,
            "torso_height": torso_height,
            "orientation_reward": orientation_reward,
            "unload_reward": unload_reward,
            "stance_reward": stance_reward,
            "pose_reward": pose_reward,
            "torque_penalty": torque_penalty,
            "is_success": is_success,
        }
        return self._task_observation(), float(reward), terminated, truncated, info
