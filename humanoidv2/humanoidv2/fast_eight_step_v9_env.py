"""Six-second-per-step forward walking task derived from V8."""

from __future__ import annotations

from dataclasses import replace
from pathlib import Path

from .eight_step_v8_env import EightStepV8Env


class FastEightStepV9Env(EightStepV8Env):
    """Compress the V8 eight-step cycle from 10 seconds to 6 seconds per step."""

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
        super().__init__(
            model_path=model_path,
            render_mode=render_mode,
            reference_only=reference_only,
            fixed_first_side=fixed_first_side,
            stride_m=stride_m,
            sway_width_m=sway_width_m,
            lift_m=lift_m,
            kp=kp,
            reanchor_mode="joint",
        )
        self.lift_steps = 60
        self.advance_steps = 30
        self.land_steps = 30
        self.settle_steps = 30
        self.step_cycle_steps = 150
        self.config = replace(
            self.config,
            max_episode_steps=self.num_steps * self.step_cycle_steps,
        )

