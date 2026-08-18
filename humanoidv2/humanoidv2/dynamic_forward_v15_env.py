"""Second speed-curriculum stage at 3.40 seconds per footstep."""

from dataclasses import replace
from pathlib import Path

from .dynamic_forward_v14_env import DynamicForwardV14Env


class DynamicForwardV15Env(DynamicForwardV14Env):
    """Compress V14 from 4.52 s to 3.40 s while retaining its gait geometry."""

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
        landing_kp: float | None = 70.0,
        impact_soft_target_n: float = 40.0,
        impact_limit_n: float = 50.0,
        impact_penalty_scale: float = 0.20,
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
            impact_soft_target_n=impact_soft_target_n,
            impact_limit_n=impact_limit_n,
            impact_penalty_scale=impact_penalty_scale,
        )
        # Proportional compression of V14's 45/15/23/23/7 schedule.
        self.lift_steps = 32
        self.lift_hold_steps = 11
        self.advance_steps = 15
        self.land_steps = 21
        self.settle_steps = 6
        self.step_cycle_steps = 85
        self.config = replace(
            self.config,
            max_episode_steps=self.num_steps * self.step_cycle_steps,
        )
        self.landing_kp = kp if landing_kp is None else landing_kp
        self.landing_kd = 0.004 * self.landing_kp

    def _phase_pd_gains(self, phase_kind: str, phase_progress: float):
        if phase_kind in {"land", "settle"}:
            return self.landing_kp, self.landing_kd
        return self.kp, self.kd

    def reset(self, *, seed=None, options=None):
        observation, info = super().reset(seed=seed, options=options)
        info.update(
            {
                "speed_curriculum_stage": 2,
                "step_cycle_steps": self.step_cycle_steps,
                "step_cycle_seconds": self.step_cycle_steps * self.config.control_dt,
                "landing_kp": self.landing_kp,
            }
        )
        return observation, info

    def step(self, action):
        observation, reward, terminated, truncated, info = super().step(action)
        info.update(
            {
                "speed_curriculum_stage": 2,
                "step_cycle_steps": self.step_cycle_steps,
                "step_cycle_seconds": self.step_cycle_steps * self.config.control_dt,
                "landing_kp": self.landing_kp,
            }
        )
        return observation, reward, terminated, truncated, info
