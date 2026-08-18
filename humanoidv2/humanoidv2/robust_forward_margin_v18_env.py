"""Robustness-evaluation wrapper around the accepted V17 walking task."""

from __future__ import annotations

from pathlib import Path
from typing import Any

import mujoco
import numpy as np

from .forward_margin_v17_env import ForwardMarginV17Env


class RobustForwardMarginV18Env(ForwardMarginV17Env):
    """Apply fixed plant/reset perturbations while preserving every V17 target."""

    def __init__(
        self,
        model_path: str | Path | None = None,
        render_mode: str | None = None,
        reference_only: bool = False,
        fixed_first_side: str | None = None,
        friction_scale: float = 1.0,
        mass_scale: float = 1.0,
        leg_mass_asymmetry: float = 0.0,
        initial_joint_noise_rad: float = 0.0,
        initial_joint_velocity_noise_rad_s: float = 0.0,
        motor_gain_scale: float = 1.0,
        control_delay_steps: int = 0,
    ) -> None:
        if friction_scale <= 0.0 or mass_scale <= 0.0 or motor_gain_scale <= 0.0:
            raise ValueError("friction, mass, and motor gain scales must be positive")
        if abs(leg_mass_asymmetry) >= 1.0:
            raise ValueError("abs(leg_mass_asymmetry) must be below 1")
        if initial_joint_noise_rad < 0.0 or initial_joint_velocity_noise_rad_s < 0.0:
            raise ValueError("initial-state noise standard deviations must be non-negative")
        if int(control_delay_steps) != control_delay_steps or control_delay_steps < 0:
            raise ValueError("control_delay_steps must be a non-negative integer")

        self.friction_scale = float(friction_scale)
        self.mass_scale = float(mass_scale)
        self.leg_mass_asymmetry = float(leg_mass_asymmetry)
        self.initial_joint_noise_rad = float(initial_joint_noise_rad)
        self.initial_joint_velocity_noise_rad_s = float(initial_joint_velocity_noise_rad_s)
        self.motor_gain_scale = float(motor_gain_scale)
        self.control_delay_steps = int(control_delay_steps)
        super().__init__(
            model_path=model_path,
            render_mode=render_mode,
            reference_only=reference_only,
            fixed_first_side=fixed_first_side,
            kp=130.0 * self.motor_gain_scale,
            landing_kp=80.0 * self.motor_gain_scale,
        )

        self._nominal_total_mass_kg = float(self.model.body_mass.sum())
        self.model.body_mass[1:] *= self.mass_scale
        self.model.body_inertia[1:] *= self.mass_scale
        for body_id in range(1, self.model.nbody):
            name = self.model.body(body_id).name or ""
            if "_L_" in name:
                factor = 1.0 + self.leg_mass_asymmetry
            elif "_R_" in name:
                factor = 1.0 - self.leg_mass_asymmetry
            else:
                continue
            self.model.body_mass[body_id] *= factor
            self.model.body_inertia[body_id] *= factor
        for geom_id in range(self.model.ngeom):
            name = self.model.geom(geom_id).name or ""
            if name == "floor" or "sole_pad" in name:
                self.model.geom_friction[geom_id] *= self.friction_scale
        mujoco.mj_setConst(self.model, self.data)
        self._delayed_actions: list[np.ndarray] = []

    def _robustness_info(self) -> dict[str, float | int]:
        return {
            "robustness_stage": 1,
            "friction_scale": self.friction_scale,
            "mass_scale": self.mass_scale,
            "leg_mass_asymmetry": self.leg_mass_asymmetry,
            "initial_joint_noise_rad": self.initial_joint_noise_rad,
            "initial_joint_velocity_noise_rad_s": self.initial_joint_velocity_noise_rad_s,
            "motor_gain_scale": self.motor_gain_scale,
            "control_delay_steps": self.control_delay_steps,
            "control_delay_seconds": self.control_delay_steps * self.config.control_dt,
            "perturbed_total_mass_kg": float(self.model.body_mass.sum()),
        }

    def reset(
        self,
        *,
        seed: int | None = None,
        options: dict[str, Any] | None = None,
    ) -> tuple[np.ndarray, dict[str, Any]]:
        _, info = super().reset(seed=seed, options=options)
        if self.initial_joint_noise_rad > 0.0:
            self.data.qpos[self.qpos_ids] = np.clip(
                self.data.qpos[self.qpos_ids]
                + self.np_random.normal(0.0, self.initial_joint_noise_rad, size=10),
                self.joint_ranges[:, 0],
                self.joint_ranges[:, 1],
            )
        if self.initial_joint_velocity_noise_rad_s > 0.0:
            self.data.qvel[self.dof_ids] = self.np_random.normal(
                0.0, self.initial_joint_velocity_noise_rad_s, size=10
            )
        mujoco.mj_forward(self.model, self.data)
        self._previous_linear_velocity = self.data.qvel[:3].copy()
        first = self._sensor_frame()
        self._history[:] = first
        self._delayed_actions = [
            np.zeros(self.action_space.shape, dtype=np.float32)
            for _ in range(self.control_delay_steps)
        ]
        info.update(self._robustness_info())
        info["perturbation_seed"] = seed
        info["torso_height"] = float(self.data.xipos[self.base_id, 2])
        return self._task_observation(advance_history=False), info

    def step(self, action: np.ndarray):
        requested_action = np.asarray(action, dtype=np.float32)
        if self.control_delay_steps:
            self._delayed_actions.append(requested_action.copy())
            applied_action = self._delayed_actions.pop(0)
        else:
            applied_action = requested_action
        observation, reward, terminated, truncated, info = super().step(applied_action)
        info.update(self._robustness_info())
        return observation, reward, terminated, truncated, info
