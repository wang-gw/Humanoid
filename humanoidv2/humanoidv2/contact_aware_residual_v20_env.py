"""Contact-aware early landing correction built on the V19 controller split."""

from __future__ import annotations

from pathlib import Path
from typing import Mapping

import numpy as np
from gymnasium import spaces

from .landing_residual_v19_env import LandingResidualV19Env, PredictPolicy


class ContactAwareResidualV20Env(LandingResidualV19Env):
    """Expose foot state and start bounded correction near the end of hold."""

    auxiliary_observation_names = (
        "left_force_100n",
        "right_force_100n",
        "left_sole_height_5cm",
        "right_sole_height_5cm",
        "left_vertical_velocity_0p5ms",
        "right_vertical_velocity_0p5ms",
        "correction_gate",
        "phase_progress",
    )

    def __init__(
        self,
        model_path: str | Path | None = None,
        render_mode: str | None = None,
        fixed_first_side: str | None = None,
        base_policy: PredictPolicy | None = None,
        curriculum_stage: int = 1,
        fixed_domain: Mapping[str, float | int] | None = None,
        landing_correction_scale: float = 0.025,
        hold_gate_fraction: float = 0.50,
        early_gate_blend: float = 1.0,
        unload_excess_reward_scale: float = 0.60,
        contact_recovery_reward_scale: float = 0.40,
        **kwargs,
    ) -> None:
        if not 0.0 <= hold_gate_fraction <= 1.0:
            raise ValueError("hold_gate_fraction must be in [0, 1]")
        if not 0.0 <= early_gate_blend <= 1.0:
            raise ValueError("early_gate_blend must be in [0, 1]")
        self.hold_gate_fraction = float(hold_gate_fraction)
        self.early_gate_blend = float(early_gate_blend)
        self.unload_excess_reward_scale_v20 = float(unload_excess_reward_scale)
        self.contact_recovery_reward_scale_v20 = float(contact_recovery_reward_scale)
        self._previous_foot_heights = np.zeros(2, dtype=np.float64)
        super().__init__(
            model_path=model_path,
            render_mode=render_mode,
            fixed_first_side=fixed_first_side,
            base_policy=base_policy,
            curriculum_stage=curriculum_stage,
            fixed_domain=fixed_domain,
            landing_correction_scale=landing_correction_scale,
            **kwargs,
        )
        self.observation_space = spaces.Box(
            -np.inf,
            np.inf,
            shape=(82 + len(self.auxiliary_observation_names),),
            dtype=np.float32,
        )
        self._current_observation = np.zeros(
            self.observation_space.shape, dtype=np.float32
        )

    def _landing_gate(
        self, phase_kind: str, phase_progress: float, prelanding_fraction: float
    ) -> float:
        original = LandingResidualV19Env._landing_gate(
            phase_kind, phase_progress, prelanding_fraction
        )
        early = 0.0
        if phase_kind == "hold" and self.hold_gate_fraction > 0.0:
            start = 1.0 - self.hold_gate_fraction
            if phase_progress > start:
                fraction = (phase_progress - start) / self.hold_gate_fraction
                early = float(3.0 * fraction**2 - 2.0 * fraction**3)
        elif phase_kind in {"advance", "land"}:
            early = 1.0
        elif phase_kind == "settle":
            fraction = min(max(phase_progress, 0.0), 1.0)
            early = float(1.0 - (3.0 * fraction**2 - 2.0 * fraction**3))
        return float(original + self.early_gate_blend * (early - original))

    def set_v20_stage(self, stage: int) -> None:
        if stage not in (0, 1, 2):
            raise ValueError("V20 stage must be in [0, 2]")
        domain_stage = (1, 2, 3)[stage]
        self.set_curriculum_stage(domain_stage)
        self.early_gate_blend = (0.0, 0.5, 1.0)[stage]

    def _corrector_observation(
        self,
        base_observation: np.ndarray,
        *,
        phase_kind: str,
        phase_progress: float,
        gate: float,
        reset: bool,
    ) -> np.ndarray:
        _, _, left_force, right_force = self._foot_contacts()
        heights = np.array(
            [
                self._sole_point(self.left_foot_id, True)[2],
                self._sole_point(self.right_foot_id, True)[2],
            ],
            dtype=np.float64,
        )
        if reset:
            velocities = np.zeros(2, dtype=np.float64)
        else:
            velocities = (heights - self._previous_foot_heights) / self.config.control_dt
        self._previous_foot_heights = heights
        auxiliary = np.array(
            [
                np.clip(left_force / 100.0, 0.0, 2.0),
                np.clip(right_force / 100.0, 0.0, 2.0),
                np.clip(heights[0] / 0.05, -0.4, 1.6),
                np.clip(heights[1] / 0.05, -0.4, 1.6),
                np.clip(velocities[0] / 0.5, -1.0, 1.0),
                np.clip(velocities[1] / 0.5, -1.0, 1.0),
                gate,
                phase_progress,
            ],
            dtype=np.float32,
        )
        return np.concatenate((base_observation, auxiliary)).astype(np.float32)

    def reset(self, *, seed=None, options=None):
        observation, info = super().reset(seed=seed, options=options)
        info.update(
            {
                "contact_aware_stage": self.curriculum_stage,
                "hold_gate_fraction": self.hold_gate_fraction,
                "early_gate_blend": self.early_gate_blend,
                "corrector_observation_size": self.observation_space.shape[0],
            }
        )
        return observation, info

    def step(self, action: np.ndarray):
        observation, reward, terminated, truncated, info = super().step(action)
        phase_kind = str(info["task_phase"]).split("_", 1)[1]
        gate = float(info["landing_correction_gate"])
        unload_excess_penalty = 0.0
        if phase_kind in {"hold", "advance"} and gate > 0.0:
            excess = max(float(info["swing_force_n"]) - 5.0, 0.0)
            unload_excess_penalty = -self.unload_excess_reward_scale_v20 * gate * min(
                excess / 10.0, 4.0
            ) ** 2
        contact_recovery_penalty = 0.0
        if phase_kind == "settle":
            minimum_force = min(
                float(info["left_foot_force_n"]), float(info["right_foot_force_n"])
            )
            missing = max(10.0 - minimum_force, 0.0)
            contact_recovery_penalty = -self.contact_recovery_reward_scale_v20 * (
                missing / 10.0
            ) ** 2
        reward += unload_excess_penalty + contact_recovery_penalty
        info.update(
            {
                "contact_aware_stage": self.curriculum_stage,
                "hold_gate_fraction": self.hold_gate_fraction,
                "early_gate_blend": self.early_gate_blend,
                "corrector_observation_size": self.observation_space.shape[0],
                "unload_excess_penalty_v20": unload_excess_penalty,
                "contact_recovery_penalty_v20": contact_recovery_penalty,
            }
        )
        return observation, float(reward), terminated, truncated, info
