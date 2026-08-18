"""Deterministic local joint-offset probes for V22 failure diagnosis."""

from __future__ import annotations

from pathlib import Path
from typing import Mapping

import numpy as np

from .contact_aware_residual_v20_env import ContactAwareResidualV20Env
from .khr3hv_env import JOINT_NAMES
from .landing_residual_v19_env import PredictPolicy


class CounterfactualProbeV22Env(ContactAwareResidualV20Env):
    """Inject one bounded, smooth probe without changing the learned policies."""

    valid_probe_phases = ("lift", "hold", "advance", "land", "settle")

    def __init__(
        self,
        model_path: str | Path | None = None,
        render_mode: str | None = None,
        fixed_first_side: str | None = None,
        base_policy: PredictPolicy | None = None,
        curriculum_stage: int = 2,
        fixed_domain: Mapping[str, float | int] | None = None,
        probe_step: int | None = None,
        probe_phase: str | None = None,
        probe_joint: int | str | None = None,
        probe_offset_rad: float = 0.0,
        probe_ramp_fraction: float = 0.25,
        **kwargs,
    ) -> None:
        if probe_step is not None and not 1 <= probe_step <= 8:
            raise ValueError("probe_step must be in [1, 8]")
        if probe_phase is not None and probe_phase not in self.valid_probe_phases:
            raise ValueError(f"unknown probe phase: {probe_phase}")
        if isinstance(probe_joint, str):
            if probe_joint not in JOINT_NAMES:
                raise ValueError(f"unknown probe joint: {probe_joint}")
            probe_joint = JOINT_NAMES.index(probe_joint)
        if probe_joint is not None and not 0 <= probe_joint < len(JOINT_NAMES):
            raise ValueError("probe_joint must be in [0, 9]")
        if abs(probe_offset_rad) > 0.025:
            raise ValueError("probe_offset_rad must be within +/-0.025 rad")
        if not 0.0 < probe_ramp_fraction <= 0.5:
            raise ValueError("probe_ramp_fraction must be in (0, 0.5]")
        self.probe_step = probe_step
        self.probe_phase = probe_phase
        self.probe_joint = probe_joint
        self.probe_offset_rad = float(probe_offset_rad)
        self.probe_ramp_fraction = float(probe_ramp_fraction)
        self._applied_probe_max_abs = 0.0
        super().__init__(
            model_path=model_path,
            render_mode=render_mode,
            fixed_first_side=fixed_first_side,
            base_policy=base_policy,
            curriculum_stage=curriculum_stage,
            fixed_domain=fixed_domain,
            **kwargs,
        )

    @staticmethod
    def _smooth_unit(value: float) -> float:
        clipped = min(max(value, 0.0), 1.0)
        return float(clipped * clipped * (3.0 - 2.0 * clipped))

    def _probe_envelope(self, phase_progress: float) -> float:
        rise = self._smooth_unit(phase_progress / self.probe_ramp_fraction)
        fall = self._smooth_unit(
            (1.0 - phase_progress) / self.probe_ramp_fraction
        )
        return min(rise, fall)

    def _residual_joint_target_offset(
        self, filtered_action: np.ndarray, phase_kind: str, phase_progress: float
    ) -> np.ndarray:
        offset = super()._residual_joint_target_offset(
            filtered_action, phase_kind, phase_progress
        )
        step_index = min(
            self._step_count // self.step_cycle_steps, self.num_steps - 1
        )
        if (
            self.probe_step == step_index + 1
            and self.probe_phase == phase_kind
            and self.probe_joint is not None
            and self.probe_offset_rad != 0.0
        ):
            applied = self.probe_offset_rad * self._probe_envelope(phase_progress)
            offset = offset.copy()
            offset[self.probe_joint] += applied
            self._applied_probe_max_abs = max(
                self._applied_probe_max_abs, abs(applied)
            )
        return offset

    def reset(self, *, seed=None, options=None):
        self._applied_probe_max_abs = 0.0
        observation, info = super().reset(seed=seed, options=options)
        info.update(self._probe_info())
        return observation, info

    def _probe_info(self) -> dict[str, object]:
        return {
            "probe_step": self.probe_step,
            "probe_phase": self.probe_phase,
            "probe_joint": None
            if self.probe_joint is None
            else JOINT_NAMES[self.probe_joint],
            "probe_offset_rad": self.probe_offset_rad,
            "applied_probe_max_abs_rad": self._applied_probe_max_abs,
        }

    def step(self, action: np.ndarray):
        observation, reward, terminated, truncated, info = super().step(action)
        info.update(self._probe_info())
        return observation, reward, terminated, truncated, info


class FirstStepStanceHipRollV22Env(CounterfactualProbeV22Env):
    """Apply the symmetric counterfactual found by the first V22 sweep."""

    def __init__(
        self,
        *args,
        stance_hip_roll_lift_offset_rad: float = 0.008,
        **kwargs,
    ) -> None:
        if not 0.0 <= stance_hip_roll_lift_offset_rad <= 0.025:
            raise ValueError(
                "stance_hip_roll_lift_offset_rad must be in [0, 0.025]"
            )
        self.stance_hip_roll_lift_offset_rad = float(
            stance_hip_roll_lift_offset_rad
        )
        self._applied_stance_hip_roll_max_abs = 0.0
        super().__init__(*args, **kwargs)

    def _residual_joint_target_offset(
        self, filtered_action: np.ndarray, phase_kind: str, phase_progress: float
    ) -> np.ndarray:
        offset = super()._residual_joint_target_offset(
            filtered_action, phase_kind, phase_progress
        )
        if (
            self._step_count < self.step_cycle_steps
            and phase_kind == "lift"
            and self.stance_hip_roll_lift_offset_rad > 0.0
        ):
            active_side = self._step_sides()[0]
            # The mirrored stance shift has opposite numeric signs in the two
            # hip-roll coordinates of this MJCF.
            joint_index = 5 if active_side == "left" else 0
            sign = 1.0 if active_side == "left" else -1.0
            applied = (
                sign
                * self.stance_hip_roll_lift_offset_rad
                * self._probe_envelope(phase_progress)
            )
            offset = offset.copy()
            offset[joint_index] += applied
            self._applied_stance_hip_roll_max_abs = max(
                self._applied_stance_hip_roll_max_abs, abs(applied)
            )
        return offset

    def reset(self, *, seed=None, options=None):
        self._applied_stance_hip_roll_max_abs = 0.0
        observation, info = super().reset(seed=seed, options=options)
        info.update(self._stance_probe_info())
        return observation, info

    def _stance_probe_info(self) -> dict[str, float]:
        return {
            "stance_hip_roll_lift_offset_rad": self.stance_hip_roll_lift_offset_rad,
            "applied_stance_hip_roll_max_abs_rad": self._applied_stance_hip_roll_max_abs,
        }

    def step(self, action: np.ndarray):
        observation, reward, terminated, truncated, info = super().step(action)
        info.update(self._stance_probe_info())
        return observation, reward, terminated, truncated, info
