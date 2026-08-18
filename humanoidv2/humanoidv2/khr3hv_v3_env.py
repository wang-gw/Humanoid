"""V3: robot-scaled fore-aft reference motion and velocity curriculum."""

from __future__ import annotations

from dataclasses import replace
from pathlib import Path

from .khr3hv_env import KHR3HVEnv, KHRConfig


class KHR3HVV3Env(KHR3HVEnv):
    """KHR residual controller with a 50 mm fore-aft reference stride."""

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
            stride_half_length=0.025,
            foot_height=0.015,
            foot_height_offset=0.005,
        )
        super().__init__(
            model_path=model_path,
            render_mode=render_mode,
            config=selected,
            reference_only=reference_only,
        )


class KHR3HVV31Env(KHR3HVEnv):
    """V3.1: V3 reference with swing phase matched to this robot's load shift."""

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
            stride_half_length=0.025,
            foot_height=0.015,
            foot_height_offset=0.005,
            swap_swing_legs=True,
            swap_forward_legs=True,
        )
        super().__init__(
            model_path=model_path,
            render_mode=render_mode,
            config=selected,
            reference_only=reference_only,
        )


class KHR3HVV32Env(KHR3HVEnv):
    """V3.2: lift/stride quadrature with a split-foot initial stance."""

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
            stride_half_length=0.015,
            foot_height=0.025,
            foot_height_offset=0.005,
            swap_swing_legs=True,
            swap_forward_legs=True,
            phased_stride=True,
            reset_to_reference=True,
        )
        super().__init__(
            model_path=model_path,
            render_mode=render_mode,
            config=selected,
            reference_only=reference_only,
        )


class KHR3HVV33Env(KHR3HVEnv):
    """V3.3: stable KHR lift phase with stride on the unloaded opposite foot."""

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
            stride_half_length=0.015,
            foot_height=0.025,
            foot_height_offset=0.005,
            swap_swing_legs=False,
            swap_forward_legs=True,
            phased_stride=True,
            reset_to_reference=True,
        )
        super().__init__(
            model_path=model_path,
            render_mode=render_mode,
            config=selected,
            reference_only=reference_only,
        )


class KHR3HVV34Env(KHR3HVEnv):
    """V3.4: lateral curriculum and explicit swing-foot unloading reward."""

    def __init__(
        self,
        model_path: str | Path | None = None,
        render_mode: str | None = None,
        config: KHRConfig | None = None,
        reference_only: bool = False,
    ) -> None:
        selected = config or KHRConfig(sway_width=0.035, sway_offset=0.00875)
        selected = replace(
            selected,
            strict_velocity_reward=True,
            enhanced_collisions=True,
            velocity_window_steps=selected.gait_period_steps,
            mirrored_roll_symmetry=True,
            alive_weight=0.20,
            fall_penalty=-10.0,
            stride_half_length=0.005,
            foot_height=0.025,
            foot_height_offset=0.005,
            swap_swing_legs=False,
            swap_forward_legs=True,
            phased_stride=True,
            reset_to_reference=True,
            contact_schedule_reward_weight=0.15,
        )
        super().__init__(
            model_path=model_path,
            render_mode=render_mode,
            config=selected,
            reference_only=reference_only,
        )
