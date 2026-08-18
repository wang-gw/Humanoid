"""Curriculum domain-randomized V10 task for conservative PPO fine-tuning."""

from __future__ import annotations

from pathlib import Path
from typing import Any

import mujoco
import numpy as np

from .soft_landing_v10_env import SoftLandingEightStepV10Env


STAGE_RANGES: tuple[dict[str, float], ...] = (
    {
        "friction_delta": 0.02,
        "mass_delta": 0.005,
        "asymmetry_delta": 0.001,
        "joint_noise_rad": 0.001,
        "joint_velocity_noise_rad_s": 0.003,
    },
    {
        "friction_delta": 0.10,
        "mass_delta": 0.010,
        "asymmetry_delta": 0.005,
        "joint_noise_rad": 0.0025,
        "joint_velocity_noise_rad_s": 0.008,
    },
    {
        "friction_delta": 0.20,
        "mass_delta": 0.020,
        "asymmetry_delta": 0.010,
        "joint_noise_rad": 0.0035,
        "joint_velocity_noise_rad_s": 0.010,
    },
)


class CurriculumSoftLandingV12Env(SoftLandingEightStepV10Env):
    """Randomize a bounded domain at each reset using a selectable curriculum stage."""

    def __init__(
        self,
        model_path: str | Path | None = None,
        render_mode: str | None = None,
        fixed_first_side: str | None = None,
        curriculum_stage: int = 0,
    ) -> None:
        super().__init__(
            model_path=model_path,
            render_mode=render_mode,
            fixed_first_side=fixed_first_side,
        )
        self._nominal_body_mass = self.model.body_mass.copy()
        self._nominal_body_inertia = self.model.body_inertia.copy()
        self._nominal_geom_friction = self.model.geom_friction.copy()
        self.curriculum_stage = 0
        self.sampled_friction_scale = 1.0
        self.sampled_mass_scale = 1.0
        self.sampled_leg_mass_asymmetry = 0.0
        self.set_curriculum_stage(curriculum_stage)

    def set_curriculum_stage(self, stage: int) -> None:
        if stage < 0 or stage >= len(STAGE_RANGES):
            raise ValueError(f"curriculum stage must be in [0, {len(STAGE_RANGES) - 1}]")
        self.curriculum_stage = int(stage)

    def _apply_sampled_physics(self) -> None:
        self.model.body_mass[:] = self._nominal_body_mass
        self.model.body_inertia[:] = self._nominal_body_inertia
        self.model.geom_friction[:] = self._nominal_geom_friction
        self.model.body_mass[1:] *= self.sampled_mass_scale
        self.model.body_inertia[1:] *= self.sampled_mass_scale
        for body_id in range(1, self.model.nbody):
            name = self.model.body(body_id).name or ""
            if "_L_" in name:
                factor = 1.0 + self.sampled_leg_mass_asymmetry
            elif "_R_" in name:
                factor = 1.0 - self.sampled_leg_mass_asymmetry
            else:
                continue
            self.model.body_mass[body_id] *= factor
            self.model.body_inertia[body_id] *= factor
        for geom_id in range(self.model.ngeom):
            name = self.model.geom(geom_id).name or ""
            if name == "floor" or "sole_pad" in name:
                self.model.geom_friction[geom_id] *= self.sampled_friction_scale
        mujoco.mj_setConst(self.model, self.data)

    def reset(
        self,
        *,
        seed: int | None = None,
        options: dict[str, Any] | None = None,
    ) -> tuple[np.ndarray, dict[str, Any]]:
        _, info = super().reset(seed=seed, options=options)
        ranges = STAGE_RANGES[self.curriculum_stage]
        self.sampled_friction_scale = float(
            self.np_random.uniform(1.0 - ranges["friction_delta"], 1.0 + ranges["friction_delta"])
        )
        self.sampled_mass_scale = float(
            self.np_random.uniform(1.0 - ranges["mass_delta"], 1.0 + ranges["mass_delta"])
        )
        self.sampled_leg_mass_asymmetry = float(
            self.np_random.uniform(-ranges["asymmetry_delta"], ranges["asymmetry_delta"])
        )
        self._apply_sampled_physics()
        self.data.qpos[self.qpos_ids] = np.clip(
            self.data.qpos[self.qpos_ids]
            + self.np_random.normal(0.0, ranges["joint_noise_rad"], size=10),
            self.joint_ranges[:, 0],
            self.joint_ranges[:, 1],
        )
        self.data.qvel[self.dof_ids] = self.np_random.normal(
            0.0, ranges["joint_velocity_noise_rad_s"], size=10
        )
        mujoco.mj_forward(self.model, self.data)
        self._previous_linear_velocity = self.data.qvel[:3].copy()
        first = self._sensor_frame()
        self._history[:] = first
        info.update(self._domain_info())
        info["torso_height"] = float(self.data.xipos[self.base_id, 2])
        return self._task_observation(advance_history=False), info

    def _domain_info(self) -> dict[str, float | int]:
        return {
            "curriculum_stage": self.curriculum_stage,
            "sampled_friction_scale": self.sampled_friction_scale,
            "sampled_mass_scale": self.sampled_mass_scale,
            "sampled_leg_mass_asymmetry": self.sampled_leg_mass_asymmetry,
        }

    def step(self, action: np.ndarray):
        observation, reward, terminated, truncated, info = super().step(action)
        info.update(self._domain_info())
        return observation, reward, terminated, truncated, info

