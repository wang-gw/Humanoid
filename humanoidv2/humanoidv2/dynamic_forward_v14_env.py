"""First speed-curriculum stage for corrected anatomical-forward stepping."""

from dataclasses import replace
from pathlib import Path

from .corrected_forward_v13_env import CorrectedForwardV13Env


class DynamicForwardV14Env(CorrectedForwardV13Env):
    """Compress V13 from 6.00 s to 4.52 s per step without changing its gait target."""

    def __init__(
        self,
        model_path: str | Path | None = None,
        render_mode: str | None = None,
        reference_only: bool = False,
        fixed_first_side: str | None = None,
        stride_m: float = 0.035,
        sway_width_m: float = 0.090,
        lift_m: float = 0.030,
        kp: float = 110.0,
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
        # Proportionally shorten the validated V13 60/20/30/30/10 schedule.
        # At 25 Hz, 113 samples (4.52 s) is the closest symmetric allocation
        # to the requested first curriculum stage of about 4.5 s per step.
        self.lift_steps = 45
        self.lift_hold_steps = 15
        self.advance_steps = 23
        self.land_steps = 23
        self.settle_steps = 7
        self.step_cycle_steps = 113
        self.config = replace(
            self.config,
            max_episode_steps=self.num_steps * self.step_cycle_steps,
        )
        self.unload_excess_penalty_scale = 0.12
        self.swing_clearance_reward_scale = 0.08
        self.swing_clearance_target_m = 0.006

    def reset(self, *, seed=None, options=None):
        observation, info = super().reset(seed=seed, options=options)
        info.update(
            {
                "speed_curriculum_stage": 1,
                "step_cycle_steps": self.step_cycle_steps,
                "step_cycle_seconds": self.step_cycle_steps * self.config.control_dt,
            }
        )
        return observation, info

    def step(self, action):
        observation, reward, terminated, truncated, info = super().step(action)
        phase_kind = str(info["task_phase"]).split("_", 1)[1]
        if phase_kind == "lift":
            unload_weight = float(info["phase_progress"]) ** 2
        elif phase_kind in {"hold", "advance"}:
            unload_weight = 1.0
        else:
            unload_weight = 0.0
        force_excess = max(float(info["swing_force_n"]) - 5.0, 0.0)
        unload_excess_penalty = (
            -self.unload_excess_penalty_scale
            * unload_weight
            * min(force_excess / 10.0, 4.0)
        )
        clearance_fraction = float(
            max(
                0.0,
                min(float(info["swing_height_m"]) / self.swing_clearance_target_m, 1.0),
            )
        )
        swing_clearance_reward = (
            self.swing_clearance_reward_scale * unload_weight * clearance_fraction
        )
        reward += unload_excess_penalty + swing_clearance_reward
        info.update(
            {
                "speed_curriculum_stage": 1,
                "step_cycle_steps": self.step_cycle_steps,
                "step_cycle_seconds": self.step_cycle_steps * self.config.control_dt,
                "unload_excess_penalty": unload_excess_penalty,
                "swing_clearance_reward": swing_clearance_reward,
                "swing_clearance_target_m": self.swing_clearance_target_m,
            }
        )
        return observation, reward, terminated, truncated, info
