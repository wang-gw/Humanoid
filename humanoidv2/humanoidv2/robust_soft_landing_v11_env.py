"""Domain-perturbed V10 environment for robustness evaluation."""

from __future__ import annotations

from pathlib import Path
from typing import Any

import mujoco
import numpy as np

from .soft_landing_v10_env import SoftLandingEightStepV10Env


class RobustSoftLandingV11Env(SoftLandingEightStepV10Env):
    """Apply controlled physics and reset perturbations without relaxing V10 success."""

    def __init__(
        self,
        model_path: str | Path | None = None,
        render_mode: str | None = None,
        fixed_first_side: str | None = None,
        friction_scale: float = 1.0,
        mass_scale: float = 1.0,
        leg_mass_asymmetry: float = 0.0,
        initial_joint_noise_rad: float = 0.0,
        initial_joint_velocity_noise_rad_s: float = 0.0,
    ) -> None:
        if friction_scale <= 0.0 or mass_scale <= 0.0:
            raise ValueError("friction_scale and mass_scale must be positive")
        if abs(leg_mass_asymmetry) >= 1.0:
            raise ValueError("abs(leg_mass_asymmetry) must be below 1")
        super().__init__(
            model_path=model_path,
            render_mode=render_mode,
            fixed_first_side=fixed_first_side,
        )
        self.friction_scale = float(friction_scale)
        self.mass_scale = float(mass_scale)
        self.leg_mass_asymmetry = float(leg_mass_asymmetry)
        self.initial_joint_noise_rad = float(initial_joint_noise_rad)
        self.initial_joint_velocity_noise_rad_s = float(initial_joint_velocity_noise_rad_s)

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

    def reset(
        self,
        *,
        seed: int | None = None,
        options: dict[str, Any] | None = None,
    ) -> tuple[np.ndarray, dict[str, Any]]:
        _, info = super().reset(seed=seed, options=options)
        if self.initial_joint_noise_rad > 0.0:
            noise = self.np_random.normal(0.0, self.initial_joint_noise_rad, size=10)
            self.data.qpos[self.qpos_ids] = np.clip(
                self.data.qpos[self.qpos_ids] + noise,
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
        info.update(
            {
                "torso_height": float(self.data.xipos[self.base_id, 2]),
                "friction_scale": self.friction_scale,
                "mass_scale": self.mass_scale,
                "leg_mass_asymmetry": self.leg_mass_asymmetry,
                "initial_joint_noise_rad": self.initial_joint_noise_rad,
                "initial_joint_velocity_noise_rad_s": self.initial_joint_velocity_noise_rad_s,
                "perturbation_seed": seed,
            }
        )
        return self._task_observation(advance_history=False), info

    def step(self, action: np.ndarray):
        observation, reward, terminated, truncated, info = super().step(action)
        info.update(
            {
                "friction_scale": self.friction_scale,
                "mass_scale": self.mass_scale,
                "leg_mass_asymmetry": self.leg_mass_asymmetry,
                "initial_joint_noise_rad": self.initial_joint_noise_rad,
                "initial_joint_velocity_noise_rad_s": self.initial_joint_velocity_noise_rad_s,
            }
        )
        return observation, reward, terminated, truncated, info

