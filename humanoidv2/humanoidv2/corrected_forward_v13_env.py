"""Corrected anatomical-forward (-Y) version of the V10 soft-landing task."""

from pathlib import Path

from .soft_landing_v10_env import SoftLandingEightStepV10Env


class CorrectedForwardV13Env(SoftLandingEightStepV10Env):
    """Run the V10 task toward the KHR-3HV model's anatomical -Y front."""

    forward_sign = -1.0
    residual_action_scale = 0.10

    def __init__(
        self,
        model_path: str | Path | None = None,
        render_mode: str | None = None,
        reference_only: bool = False,
        fixed_first_side: str | None = None,
        stride_m: float = 0.035,
        sway_width_m: float = 0.090,
        lift_m: float = 0.030,
        kp: float = 90.0,
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
            impact_soft_target_n=impact_soft_target_n,
            impact_limit_n=impact_limit_n,
            impact_penalty_scale=impact_penalty_scale,
        )
        # Reach the lifted pose, hold it until contact transients settle, then
        # advance. The 150-sample (6 s) step duration is unchanged.
        self.lift_steps = 60
        self.lift_hold_steps = 20
        self.advance_steps = 30
        self.land_steps = 30
        self.settle_steps = 10
        self.step_cycle_steps = 150
