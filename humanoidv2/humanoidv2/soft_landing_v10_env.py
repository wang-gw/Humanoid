"""Six-second eight-step task with an explicit soft-landing objective."""

from __future__ import annotations

from pathlib import Path
import numpy as np

from .fast_eight_step_v9_env import FastEightStepV9Env


class SoftLandingEightStepV10Env(FastEightStepV9Env):
    """Retain V9 speed while limiting every measured landing peak to 50 N."""

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
        impact_soft_target_n: float = 40.0,
        impact_limit_n: float = 50.0,
        impact_penalty_scale: float = 0.08,
    ) -> None:
        super().__init__(
            model_path=model_path,
            render_mode=render_mode,
            reference_only=reference_only,
            fixed_first_side=fixed_first_side,
            stride_m=stride_m,
            sway_width_m=sway_width_m,
            lift_m=lift_m,
            kp=kp,
        )
        # Preserve 150 control samples (6 s) per step. Only the land/settle
        # allocation changes, so a V9 policy remains a useful warm start.
        self.lift_steps = 60
        self.advance_steps = 30
        self.land_steps = 50
        self.settle_steps = 10
        self.step_cycle_steps = 150
        self.impact_soft_target_n = impact_soft_target_n
        self.impact_limit_n = impact_limit_n
        self.impact_penalty_scale = impact_penalty_scale

    def step(self, action: np.ndarray):
        observation, reward, terminated, truncated, info = super().step(action)
        phase_kind = str(info["task_phase"]).split("_", 1)[1]
        step_index = int(info["active_step"]) - 1
        # V7-V9 recorded only the interpolated landing phase. Extend the
        # impact window through settle so a late contact spike cannot hide at
        # the phase boundary.
        if phase_kind in {"land", "settle"}:
            self._landing_peaks[step_index] = max(
                self._landing_peaks[step_index], float(info["swing_force_n"])
            )
            info[f"step_{step_index + 1}_peak_landing_force_n"] = float(
                self._landing_peaks[step_index]
            )
        impact_penalty = 0.0
        if phase_kind in {"land", "settle"}:
            excess = max(float(info["swing_force_n"]) - self.impact_soft_target_n, 0.0)
            normalized_excess = excess / max(self.impact_limit_n - self.impact_soft_target_n, 1.0e-6)
            impact_penalty = -self.impact_penalty_scale * normalized_excess**2
            reward += impact_penalty

        maximum_landing_force = float(np.max(self._landing_peaks))
        impact_limit_satisfied = bool(
            truncated and np.all(self._landing_peaks <= self.impact_limit_n)
        )
        gait_success = bool(info["is_success"])
        info.update(
            {
                "gait_success": gait_success,
                "impact_soft_target_n": self.impact_soft_target_n,
                "impact_limit_n": self.impact_limit_n,
                "impact_penalty": float(impact_penalty),
                "maximum_landing_force_n": maximum_landing_force,
                "impact_limit_satisfied": impact_limit_satisfied,
                "is_success": bool(gait_success and impact_limit_satisfied),
            }
        )
        return observation, float(reward), terminated, truncated, info
