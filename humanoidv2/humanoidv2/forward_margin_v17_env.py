"""Recover forward-distance margin at the fixed V16 cycle time."""

from pathlib import Path

import numpy as np

from .dynamic_forward_v16_env import DynamicForwardV16Env


class ForwardMarginV17Env(DynamicForwardV16Env):
    """Use a longer stride and reward phase-aware base progress without forcing it."""

    def __init__(
        self,
        model_path: str | Path | None = None,
        render_mode: str | None = None,
        reference_only: bool = False,
        fixed_first_side: str | None = None,
        stride_m: float = 0.042,
        sway_width_m: float = 0.090,
        lift_m: float = 0.030,
        kp: float = 130.0,
        landing_kp: float = 80.0,
        gain_transition_fraction: float = 0.12,
        body_progress_per_step_m: float = 0.0275,
        body_progress_reward_scale: float = 0.10,
        body_stall_penalty_scale: float = 0.05,
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
            landing_kp=landing_kp,
            gain_transition_fraction=gain_transition_fraction,
            impact_soft_target_n=impact_soft_target_n,
            impact_limit_n=impact_limit_n,
            impact_penalty_scale=impact_penalty_scale,
        )
        self.minimum_forward_m = 0.200
        self.body_progress_per_step_m = body_progress_per_step_m
        self.body_progress_reward_scale = body_progress_reward_scale
        self.body_stall_penalty_scale = body_stall_penalty_scale
        self._previous_base_forward_m = 0.0

    @staticmethod
    def _phase_body_fraction(phase_kind: str, phase_progress: float) -> float:
        if phase_kind in {"lift", "hold"}:
            return 0.0
        if phase_kind == "advance":
            return 0.70 * phase_progress
        if phase_kind == "land":
            return 0.70 + 0.30 * phase_progress
        return 1.0

    def reset(self, *, seed=None, options=None):
        observation, info = super().reset(seed=seed, options=options)
        self._previous_base_forward_m = 0.0
        info.update(
            {
                "forward_margin_stage": 1,
                "body_progress_per_step_m": self.body_progress_per_step_m,
                "minimum_forward_m": self.minimum_forward_m,
                "desired_body_forward_m": 0.0,
                "body_progress_error_m": 0.0,
            }
        )
        return observation, info

    def step(self, action):
        observation, reward, terminated, truncated, info = super().step(action)
        phase_kind = str(info["task_phase"]).split("_", 1)[1]
        step_index = int(info["active_step"]) - 1
        phase_fraction = self._phase_body_fraction(
            phase_kind, float(info["phase_progress"])
        )
        desired_forward = self.body_progress_per_step_m * (
            step_index + phase_fraction
        )
        actual_forward = float(info["base_forward_displacement_m"])
        progress_error = actual_forward - desired_forward
        # Reward meeting or exceeding the target equally; only shortfall is
        # penalized so the shaping cannot pull an already-good policy backward.
        progress_shortfall = max(-progress_error, 0.0)
        body_progress_reward = self.body_progress_reward_scale * float(
            np.exp(-(progress_shortfall / 0.020) ** 2)
        )
        forward_delta = actual_forward - self._previous_base_forward_m
        body_stall_penalty = 0.0
        if phase_kind in {"advance", "land"} and forward_delta < 0.0:
            body_stall_penalty = -self.body_stall_penalty_scale * min(
                -forward_delta / 0.002, 1.0
            )
        reward += body_progress_reward + body_stall_penalty
        self._previous_base_forward_m = actual_forward

        # The inherited gait check already uses minimum_forward_m=0.200.
        gait_success = bool(info["gait_success"])
        info.update(
            {
                "forward_margin_stage": 1,
                "body_progress_per_step_m": self.body_progress_per_step_m,
                "minimum_forward_m": self.minimum_forward_m,
                "desired_body_forward_m": desired_forward,
                "body_progress_error_m": progress_error,
                "body_progress_shortfall_m": progress_shortfall,
                "body_progress_reward": body_progress_reward,
                "body_forward_delta_m": forward_delta,
                "body_stall_penalty": body_stall_penalty,
                "gait_success": gait_success,
                "is_success": bool(gait_success and info["impact_limit_satisfied"]),
            }
        )
        return observation, float(reward), terminated, truncated, info
