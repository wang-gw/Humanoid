"""Phase-split V21 corrector with one-step landing memory."""

from __future__ import annotations

from pathlib import Path
from typing import Mapping

import numpy as np
from gymnasium import spaces

from .contact_aware_residual_v20_env import ContactAwareResidualV20Env
from .landing_residual_v19_env import PredictPolicy


class PhaseSplitResidualV21Env(ContactAwareResidualV20Env):
    """Separate unloading and landing corrections while freezing the V17 base."""

    memory_observation_names = (
        "previous_step_peak_force_100n",
        "previous_step_advance_under_5n_fraction",
        "previous_step_both_contact_fraction",
        "previous_step_length_42mm",
    )

    def __init__(
        self,
        model_path: str | Path | None = None,
        render_mode: str | None = None,
        fixed_first_side: str | None = None,
        base_policy: PredictPolicy | None = None,
        curriculum_stage: int = 2,
        fixed_domain: Mapping[str, float | int] | None = None,
        early_gate_blend: float = 0.75,
        head_transition_start: float = 0.50,
        conditional_lift_gate_scale: float = 0.0,
        conditional_lift_start: float = 0.25,
        conditional_lift_force_n: float = 5.0,
        conditional_lift_full_force_n: float = 15.0,
        head_action_penalty_scale: float = 0.001,
        **kwargs,
    ) -> None:
        if not 0.0 <= head_transition_start < 1.0:
            raise ValueError("head_transition_start must be in [0, 1)")
        if not 0.0 <= conditional_lift_gate_scale <= 1.0:
            raise ValueError("conditional_lift_gate_scale must be in [0, 1]")
        if not 0.0 <= conditional_lift_start < 1.0:
            raise ValueError("conditional_lift_start must be in [0, 1)")
        if conditional_lift_full_force_n <= conditional_lift_force_n:
            raise ValueError(
                "conditional_lift_full_force_n must exceed conditional_lift_force_n"
            )
        self.head_transition_start = float(head_transition_start)
        self.conditional_lift_gate_scale = float(conditional_lift_gate_scale)
        self.conditional_lift_start = float(conditional_lift_start)
        self.conditional_lift_force_n = float(conditional_lift_force_n)
        self.conditional_lift_full_force_n = float(conditional_lift_full_force_n)
        self.head_action_penalty_scale_v21 = float(head_action_penalty_scale)
        self._step_memory = np.zeros(4, dtype=np.float32)
        self._last_recorded_step = -1
        self._head_mix = 0.0
        self._unload_head_max_abs = 0.0
        self._landing_head_max_abs = 0.0
        super().__init__(
            model_path=model_path,
            render_mode=render_mode,
            fixed_first_side=fixed_first_side,
            base_policy=base_policy,
            curriculum_stage=curriculum_stage,
            fixed_domain=fixed_domain,
            early_gate_blend=early_gate_blend,
            **kwargs,
        )
        self.action_space = spaces.Box(-1.0, 1.0, shape=(20,), dtype=np.float32)
        self.observation_space = spaces.Box(
            -np.inf,
            np.inf,
            shape=(90 + len(self.memory_observation_names),),
            dtype=np.float32,
        )
        self._current_observation = np.zeros(
            self.observation_space.shape, dtype=np.float32
        )

    @staticmethod
    def _smooth_unit(value: float) -> float:
        clipped = min(max(value, 0.0), 1.0)
        return float(clipped * clipped * (3.0 - 2.0 * clipped))

    def _landing_head_mix(self, phase_kind: str, phase_progress: float) -> float:
        if phase_kind in {"land", "settle"}:
            return 1.0
        if phase_kind != "advance" or phase_progress <= self.head_transition_start:
            return 0.0
        fraction = (phase_progress - self.head_transition_start) / (
            1.0 - self.head_transition_start
        )
        return self._smooth_unit(fraction)

    def _active_swing_force(self) -> float:
        step_index = min(
            self._step_count // self.step_cycle_steps, self.num_steps - 1
        )
        active_side = self._step_sides()[step_index]
        _, _, left_force, right_force = self._foot_contacts()
        return float(left_force if active_side == "left" else right_force)

    def _landing_gate(
        self, phase_kind: str, phase_progress: float, prelanding_fraction: float
    ) -> float:
        base_gate = super()._landing_gate(
            phase_kind, phase_progress, prelanding_fraction
        )
        if phase_kind != "lift" or self.conditional_lift_gate_scale <= 0.0:
            return base_gate
        phase_fraction = (phase_progress - self.conditional_lift_start) / (
            1.0 - self.conditional_lift_start
        )
        force_fraction = (
            self._active_swing_force() - self.conditional_lift_force_n
        ) / (self.conditional_lift_full_force_n - self.conditional_lift_force_n)
        conditional_gate = (
            self.conditional_lift_gate_scale
            * self._smooth_unit(phase_fraction)
            * self._smooth_unit(force_fraction)
        )
        return float(max(base_gate, conditional_gate))

    def set_v21_stage(self, stage: int) -> None:
        """Advance from exact V20 behavior to bounded lift assistance."""
        if stage not in (0, 1, 2):
            raise ValueError("V21 stage must be in [0, 2]")
        self.set_curriculum_stage((2, 3, 2)[stage])
        self.early_gate_blend = 0.75
        self.conditional_lift_gate_scale = (0.0, 0.20, 0.35)[stage]

    def _corrector_observation(
        self,
        base_observation: np.ndarray,
        *,
        phase_kind: str,
        phase_progress: float,
        gate: float,
        reset: bool,
    ) -> np.ndarray:
        contact_observation = super()._corrector_observation(
            base_observation,
            phase_kind=phase_kind,
            phase_progress=phase_progress,
            gate=gate,
            reset=reset,
        )
        return np.concatenate((contact_observation, self._step_memory)).astype(
            np.float32
        )

    def _record_completed_step(self, info: dict[str, object]) -> bool:
        phase_kind = str(info["task_phase"]).split("_", 1)[1]
        step_index = int(info["active_step"]) - 1
        if (
            phase_kind != "settle"
            or float(info["phase_progress"]) < 1.0
            or step_index == self._last_recorded_step
        ):
            return False
        number = step_index + 1
        self._step_memory = np.array(
            [
                np.clip(
                    float(info[f"step_{number}_peak_landing_force_n"]) / 100.0,
                    0.0,
                    2.0,
                ),
                np.clip(
                    float(info[f"step_{number}_advance_under_5n_fraction"]),
                    0.0,
                    1.0,
                ),
                np.clip(
                    float(info[f"step_{number}_both_contact_fraction"]),
                    0.0,
                    1.0,
                ),
                np.clip(
                    float(info[f"step_{number}_length_m"]) / self.stride_m,
                    -1.0,
                    2.0,
                ),
            ],
            dtype=np.float32,
        )
        self._last_recorded_step = step_index
        return True

    def reset(self, *, seed=None, options=None):
        self._step_memory.fill(0.0)
        self._last_recorded_step = -1
        self._head_mix = 0.0
        observation, info = super().reset(seed=seed, options=options)
        # V19's delayed physical correction is always ten-dimensional; the
        # externally visible V21 action contains two such heads.
        delay = int(self.sampled_domain["control_delay_steps"])
        self._delayed_actions = [
            (np.zeros(10, dtype=np.float32), np.zeros(10, dtype=np.float64))
            for _ in range(delay)
        ]
        self._previous_correction = np.zeros(10, dtype=np.float64)
        self._pending_joint_correction_rad = np.zeros(10, dtype=np.float64)
        info.update(
            {
                "phase_split_stage": self.curriculum_stage,
                "corrector_observation_size": 94,
                "corrector_action_size": 20,
                "landing_head_mix": 0.0,
                "conditional_lift_gate_scale": self.conditional_lift_gate_scale,
                "step_memory": self._step_memory.tolist(),
            }
        )
        return observation, info

    def step(self, action: np.ndarray):
        split_action = np.clip(np.asarray(action, dtype=np.float64), -1.0, 1.0)
        if split_action.shape != (20,):
            raise ValueError(f"V21 action must have shape (20,), got {split_action.shape}")
        _, phase, phase_progress, _, _ = self._reference_target()
        phase_kind = phase.split("_", 1)[1]
        unload_head = split_action[:10]
        landing_head = split_action[10:]
        self._head_mix = self._landing_head_mix(phase_kind, phase_progress)
        combined = (1.0 - self._head_mix) * unload_head + self._head_mix * landing_head
        self._unload_head_max_abs = float(np.max(np.abs(unload_head)))
        self._landing_head_max_abs = float(np.max(np.abs(landing_head)))

        observation, reward, terminated, truncated, info = super().step(combined)
        head_action_penalty = -self.head_action_penalty_scale_v21 * float(
            np.mean(split_action**2)
        )
        reward += head_action_penalty
        memory_updated = self._record_completed_step(info)
        if memory_updated:
            observation[-4:] = self._step_memory
            self._current_observation[-4:] = self._step_memory
        info.update(
            {
                "phase_split_stage": self.curriculum_stage,
                "corrector_observation_size": 94,
                "corrector_action_size": 20,
                "landing_head_mix": self._head_mix,
                "unload_head_max_abs": self._unload_head_max_abs,
                "landing_head_max_abs": self._landing_head_max_abs,
                "combined_correction_action_max_abs": float(
                    np.max(np.abs(combined))
                ),
                "conditional_lift_gate_scale": self.conditional_lift_gate_scale,
                "head_action_penalty_v21": head_action_penalty,
                "step_memory_updated": memory_updated,
                "step_memory": self._step_memory.tolist(),
            }
        )
        return observation, float(reward), terminated, truncated, info
