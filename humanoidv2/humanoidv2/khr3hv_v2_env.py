"""V2: KHR reference-residual structure with a strict walking objective."""

from __future__ import annotations

from dataclasses import replace
from pathlib import Path

from .khr3hv_env import KHR3HVEnv, KHRConfig


class KHR3HVV2Env(KHR3HVEnv):
    """Removes V1's zero-speed reward loophole and adds collision proxies."""

    def __init__(
        self,
        model_path: str | Path | None = None,
        render_mode: str | None = None,
        config: KHRConfig | None = None,
        reference_only: bool = False,
    ) -> None:
        selected = config or KHRConfig()
        selected = replace(selected, strict_velocity_reward=True, enhanced_collisions=True)
        super().__init__(
            model_path=model_path,
            render_mode=render_mode,
            config=selected,
            reference_only=reference_only,
        )


class KHR3HVV21Env(KHR3HVEnv):
    """V2.1: gait-cycle velocity filtering and robot-correct symmetry."""

    def __init__(
        self,
        model_path: str | Path | None = None,
        render_mode: str | None = None,
        config: KHRConfig | None = None,
        reference_only: bool = False,
    ) -> None:
        selected = config or KHRConfig()
        selected = replace(
            selected,
            strict_velocity_reward=True,
            enhanced_collisions=True,
            velocity_window_steps=selected.gait_period_steps,
            mirrored_roll_symmetry=True,
        )
        super().__init__(
            model_path=model_path,
            render_mode=render_mode,
            config=selected,
            reference_only=reference_only,
        )


class KHR3HVV22Env(KHR3HVEnv):
    """V2.2: V2.1 plus explicit incentives against early termination."""

    def __init__(
        self,
        model_path: str | Path | None = None,
        render_mode: str | None = None,
        config: KHRConfig | None = None,
        reference_only: bool = False,
    ) -> None:
        selected = config or KHRConfig()
        selected = replace(
            selected,
            strict_velocity_reward=True,
            enhanced_collisions=True,
            velocity_window_steps=selected.gait_period_steps,
            mirrored_roll_symmetry=True,
            alive_weight=0.20,
            fall_penalty=-10.0,
        )
        super().__init__(
            model_path=model_path,
            render_mode=render_mode,
            config=selected,
            reference_only=reference_only,
        )
