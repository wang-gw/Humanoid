"""Third speed-curriculum stage at 2.56 seconds per footstep."""

from dataclasses import replace
from pathlib import Path

from .dynamic_forward_v15_env import DynamicForwardV15Env


class DynamicForwardV16Env(DynamicForwardV15Env):
    """Compress V15 and optionally blend swing gain into landing gain."""

    def __init__(
        self,
        model_path: str | Path | None = None,
        render_mode: str | None = None,
        reference_only: bool = False,
        fixed_first_side: str | None = None,
        stride_m: float = 0.035,
        sway_width_m: float = 0.090,
        lift_m: float = 0.030,
        kp: float = 130.0,
        landing_kp: float = 80.0,
        gain_transition_fraction: float = 0.12,
        impact_soft_target_n: float = 40.0,
        impact_limit_n: float = 50.0,
        impact_penalty_scale: float = 0.20,
    ) -> None:
        if not 0.0 <= gain_transition_fraction <= 1.0:
            raise ValueError("gain_transition_fraction must be in [0, 1]")
        self.gain_transition_fraction = gain_transition_fraction
        super().__init__(
            model_path=model_path,
            render_mode=render_mode,
            reference_only=reference_only,
            fixed_first_side=fixed_first_side,
            stride_m=stride_m,
            sway_width_m=sway_width_m,
            lift_m=lift_m,
            kp=kp,
            landing_kp=landing_kp,
            impact_soft_target_n=impact_soft_target_n,
            impact_limit_n=impact_limit_n,
            impact_penalty_scale=impact_penalty_scale,
        )
        self.lift_steps = 20
        self.lift_hold_steps = 8
        self.advance_steps = 10
        self.land_steps = 21
        self.settle_steps = 5
        self.step_cycle_steps = 64
        self.config = replace(
            self.config,
            max_episode_steps=self.num_steps * self.step_cycle_steps,
        )

    def _phase_pd_gains(self, phase_kind: str, phase_progress: float):
        if phase_kind in {"land", "settle"}:
            return self.landing_kp, self.landing_kd
        if phase_kind == "advance" and self.gain_transition_fraction > 0.0:
            start = 1.0 - self.gain_transition_fraction
            if phase_progress > start:
                fraction = (phase_progress - start) / self.gain_transition_fraction
                blend = self._smoothstep(min(max(fraction, 0.0), 1.0))
                kp = self.kp + blend * (self.landing_kp - self.kp)
                kd = self.kd + blend * (self.landing_kd - self.kd)
                return kp, kd
        return self.kp, self.kd

    def reset(self, *, seed=None, options=None):
        observation, info = super().reset(seed=seed, options=options)
        info.update(
            {
                "speed_curriculum_stage": 3,
                "step_cycle_steps": self.step_cycle_steps,
                "step_cycle_seconds": self.step_cycle_steps * self.config.control_dt,
                "gain_transition_fraction": self.gain_transition_fraction,
            }
        )
        return observation, info

    def step(self, action):
        observation, reward, terminated, truncated, info = super().step(action)
        info.update(
            {
                "speed_curriculum_stage": 3,
                "step_cycle_steps": self.step_cycle_steps,
                "step_cycle_seconds": self.step_cycle_steps * self.config.control_dt,
                "gain_transition_fraction": self.gain_transition_fraction,
            }
        )
        return observation, reward, terminated, truncated, info
