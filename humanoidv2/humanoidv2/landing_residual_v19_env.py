"""Landing-only bounded correction on top of a frozen V17 base policy."""

from __future__ import annotations

from pathlib import Path
from typing import Any, Mapping, Protocol

import mujoco
import numpy as np

from .forward_margin_v17_env import ForwardMarginV17Env


class PredictPolicy(Protocol):
    def predict(self, observation: np.ndarray, deterministic: bool = True): ...


V19_DOMAIN_RANGES: tuple[dict[str, float | int], ...] = (
    {
        "friction_delta": 0.05,
        "mass_delta": 0.005,
        "asymmetry_delta": 0.0025,
        "joint_noise_rad": 0.0010,
        "joint_velocity_noise_rad_s": 0.003,
        "motor_gain_delta": 0.01,
        "maximum_control_delay_steps": 0,
    },
    {
        "friction_delta": 0.10,
        "mass_delta": 0.010,
        "asymmetry_delta": 0.0050,
        "joint_noise_rad": 0.0020,
        "joint_velocity_noise_rad_s": 0.006,
        "motor_gain_delta": 0.02,
        "maximum_control_delay_steps": 1,
    },
    {
        "friction_delta": 0.20,
        "mass_delta": 0.020,
        "asymmetry_delta": 0.0100,
        "joint_noise_rad": 0.0035,
        "joint_velocity_noise_rad_s": 0.010,
        "motor_gain_delta": 0.05,
        "maximum_control_delay_steps": 2,
    },
    {
        "friction_delta": 0.20,
        "mass_delta": 0.020,
        "asymmetry_delta": 0.0100,
        "joint_noise_rad": 0.0035,
        "joint_velocity_noise_rad_s": 0.010,
        "motor_gain_delta": 0.05,
        "maximum_control_delay_steps": 2,
    },
)


class LandingResidualV19Env(ForwardMarginV17Env):
    """Train a small landing correction while evaluating the complete frozen controller."""

    def __init__(
        self,
        model_path: str | Path | None = None,
        render_mode: str | None = None,
        fixed_first_side: str | None = None,
        base_policy: PredictPolicy | None = None,
        curriculum_stage: int = 0,
        fixed_domain: Mapping[str, float | int] | None = None,
        landing_correction_scale: float = 0.025,
        prelanding_fraction: float = 0.50,
        landing_force_reward_scale: float = 0.35,
        downward_velocity_reward_scale: float = 0.08,
        correction_penalty_scale: float = 0.01,
        correction_rate_penalty_scale: float = 0.005,
        terminal_success_bonus: float = 20.0,
    ) -> None:
        if not 0.0 < landing_correction_scale <= 0.08:
            raise ValueError("landing_correction_scale must be in (0, 0.08] rad")
        if not 0.0 <= prelanding_fraction <= 1.0:
            raise ValueError("prelanding_fraction must be in [0, 1]")
        self.base_policy = base_policy
        self.landing_correction_scale = float(landing_correction_scale)
        self.prelanding_fraction = float(prelanding_fraction)
        self.landing_force_reward_scale = float(landing_force_reward_scale)
        self.downward_velocity_reward_scale = float(downward_velocity_reward_scale)
        self.correction_penalty_scale = float(correction_penalty_scale)
        self.correction_rate_penalty_scale = float(correction_rate_penalty_scale)
        self.terminal_success_bonus = float(terminal_success_bonus)
        self.fixed_domain = dict(fixed_domain) if fixed_domain is not None else None
        super().__init__(
            model_path=model_path,
            render_mode=render_mode,
            fixed_first_side=fixed_first_side,
        )

        self._nominal_body_mass = self.model.body_mass.copy()
        self._nominal_body_inertia = self.model.body_inertia.copy()
        self._nominal_geom_friction = self.model.geom_friction.copy()
        self._nominal_kp = self.kp
        self._nominal_landing_kp = self.landing_kp
        self.curriculum_stage = 0
        self.sampled_domain: dict[str, float | int] = {}
        self._current_base_observation = np.zeros(
            self.observation_space.shape, dtype=np.float32
        )
        self._current_observation = np.zeros(self.observation_space.shape, dtype=np.float32)
        self._delayed_actions: list[tuple[np.ndarray, np.ndarray]] = []
        # The frozen base controller and the physical robot both remain
        # ten-dimensional even when a later corrector exposes more policy heads.
        self._previous_correction = np.zeros(10, dtype=np.float64)
        self._pending_joint_correction_rad = np.zeros(10, dtype=np.float64)
        self.set_curriculum_stage(curriculum_stage)

    def set_curriculum_stage(self, stage: int) -> None:
        if stage < 0 or stage >= len(V19_DOMAIN_RANGES):
            raise ValueError(f"curriculum stage must be in [0, {len(V19_DOMAIN_RANGES) - 1}]")
        self.curriculum_stage = int(stage)

    @staticmethod
    def _landing_gate(phase_kind: str, phase_progress: float, prelanding_fraction: float) -> float:
        if phase_kind == "advance" and prelanding_fraction > 0.0:
            start = 1.0 - prelanding_fraction
            if phase_progress > start:
                fraction = (phase_progress - start) / prelanding_fraction
                return float(3.0 * fraction**2 - 2.0 * fraction**3)
        if phase_kind == "land":
            return 1.0
        if phase_kind == "settle":
            fraction = min(max(phase_progress, 0.0), 1.0)
            return float(1.0 - (3.0 * fraction**2 - 2.0 * fraction**3))
        return 0.0

    def _sample_domain(self) -> dict[str, float | int]:
        if self.fixed_domain is not None:
            defaults: dict[str, float | int] = {
                "friction_scale": 1.0,
                "mass_scale": 1.0,
                "leg_mass_asymmetry": 0.0,
                "initial_joint_noise_rad": 0.0,
                "initial_joint_velocity_noise_rad_s": 0.0,
                "motor_gain_scale": 1.0,
                "control_delay_steps": 0,
            }
            defaults.update(self.fixed_domain)
            return defaults
        ranges = V19_DOMAIN_RANGES[self.curriculum_stage]
        if self.curriculum_stage == 3:
            friction_scale = float(
                self.np_random.uniform(1.05, 1.20)
                if self.np_random.random() < 0.75
                else self.np_random.uniform(0.80, 1.05)
            )
        else:
            friction_scale = float(
                self.np_random.uniform(
                    1.0 - ranges["friction_delta"], 1.0 + ranges["friction_delta"]
                )
            )
        return {
            "friction_scale": friction_scale,
            "mass_scale": float(
                self.np_random.uniform(1.0 - ranges["mass_delta"], 1.0 + ranges["mass_delta"])
            ),
            "leg_mass_asymmetry": float(
                self.np_random.uniform(-ranges["asymmetry_delta"], ranges["asymmetry_delta"])
            ),
            "initial_joint_noise_rad": float(ranges["joint_noise_rad"]),
            "initial_joint_velocity_noise_rad_s": float(ranges["joint_velocity_noise_rad_s"]),
            "motor_gain_scale": float(
                self.np_random.uniform(
                    1.0 - ranges["motor_gain_delta"], 1.0 + ranges["motor_gain_delta"]
                )
            ),
            "control_delay_steps": int(
                self.np_random.integers(0, int(ranges["maximum_control_delay_steps"]) + 1)
            ),
        }

    def _apply_domain(self) -> None:
        domain = self.sampled_domain
        stored_qpos = self.data.qpos.copy()
        stored_qvel = self.data.qvel.copy()
        stored_ctrl = self.data.ctrl.copy()
        stored_time = float(self.data.time)
        self.model.body_mass[:] = self._nominal_body_mass
        self.model.body_inertia[:] = self._nominal_body_inertia
        self.model.geom_friction[:] = self._nominal_geom_friction
        self.model.body_mass[1:] *= float(domain["mass_scale"])
        self.model.body_inertia[1:] *= float(domain["mass_scale"])
        asymmetry = float(domain["leg_mass_asymmetry"])
        for body_id in range(1, self.model.nbody):
            name = self.model.body(body_id).name or ""
            factor = 1.0 + asymmetry if "_L_" in name else 1.0 - asymmetry if "_R_" in name else 1.0
            self.model.body_mass[body_id] *= factor
            self.model.body_inertia[body_id] *= factor
        for geom_id in range(self.model.ngeom):
            name = self.model.geom(geom_id).name or ""
            if name == "floor" or "sole_pad" in name:
                self.model.geom_friction[geom_id] *= float(domain["friction_scale"])
        gain_scale = float(domain["motor_gain_scale"])
        self.kp = self._nominal_kp * gain_scale
        self.kd = 0.004 * self.kp
        self.landing_kp = self._nominal_landing_kp * gain_scale
        self.landing_kd = 0.004 * self.landing_kp
        mujoco.mj_setConst(self.model, self.data)
        # mj_setConst resets MjData to qpos0. Domain randomization happens
        # after the task reset, so restore the accepted V17 start state.
        self.data.qpos[:] = stored_qpos
        self.data.qvel[:] = stored_qvel
        self.data.ctrl[:] = stored_ctrl
        self.data.time = stored_time
        mujoco.mj_forward(self.model, self.data)

    def _domain_info(self) -> dict[str, float | int]:
        return {
            "landing_residual_stage": self.curriculum_stage,
            **self.sampled_domain,
            "control_delay_seconds": int(self.sampled_domain["control_delay_steps"])
            * self.config.control_dt,
            "perturbed_total_mass_kg": float(self.model.body_mass.sum()),
            "landing_correction_limit_rad": self.landing_correction_scale,
        }

    def _corrector_observation(
        self,
        base_observation: np.ndarray,
        *,
        phase_kind: str,
        phase_progress: float,
        gate: float,
        reset: bool,
    ) -> np.ndarray:
        """Return the correction-policy observation; V19 uses the original 82 values."""
        return base_observation.copy()

    def reset(self, *, seed: int | None = None, options: dict[str, Any] | None = None):
        _, info = super().reset(seed=seed, options=options)
        self.sampled_domain = self._sample_domain()
        self._apply_domain()
        joint_noise = float(self.sampled_domain["initial_joint_noise_rad"])
        velocity_noise = float(self.sampled_domain["initial_joint_velocity_noise_rad_s"])
        if joint_noise > 0.0:
            self.data.qpos[self.qpos_ids] = np.clip(
                self.data.qpos[self.qpos_ids]
                + self.np_random.normal(0.0, joint_noise, size=10),
                self.joint_ranges[:, 0],
                self.joint_ranges[:, 1],
            )
        if velocity_noise > 0.0:
            self.data.qvel[self.dof_ids] = self.np_random.normal(0.0, velocity_noise, size=10)
        mujoco.mj_forward(self.model, self.data)
        self._previous_linear_velocity = self.data.qvel[:3].copy()
        first = self._sensor_frame()
        self._history[:] = first
        delay = int(self.sampled_domain["control_delay_steps"])
        self._delayed_actions = [
            (
                np.zeros(10, dtype=np.float32),
                np.zeros(10, dtype=np.float64),
            )
            for _ in range(delay)
        ]
        self._previous_correction.fill(0.0)
        self._pending_joint_correction_rad.fill(0.0)
        self._current_base_observation = self._task_observation(advance_history=False)
        self._current_observation = self._corrector_observation(
            self._current_base_observation,
            phase_kind="lift",
            phase_progress=0.0,
            gate=0.0,
            reset=True,
        )
        info.update(self._domain_info())
        info.update(
            {
                "perturbation_seed": seed,
                "landing_correction_gate": 0.0,
                "applied_correction_max_abs": 0.0,
                "torso_height": float(self.data.xipos[self.base_id, 2]),
            }
        )
        return self._current_observation.copy(), info

    def step(self, action: np.ndarray):
        correction = np.clip(np.asarray(action, dtype=np.float64), -1.0, 1.0)
        if self.base_policy is None:
            base_action = np.zeros(10, dtype=np.float64)
        else:
            base_action = np.asarray(
                self.base_policy.predict(
                    self._current_base_observation, deterministic=True
                )[0],
                dtype=np.float64,
            )
        _, phase, phase_progress, active_side, _ = self._reference_target()
        phase_kind = phase.split("_", 1)[1]
        gate = self._landing_gate(phase_kind, phase_progress, self.prelanding_fraction)
        applied_correction = self.landing_correction_scale * gate * correction
        delay = int(self.sampled_domain["control_delay_steps"])
        if delay:
            self._delayed_actions.append(
                (base_action.astype(np.float32), applied_correction.copy())
            )
            plant_action, self._pending_joint_correction_rad = self._delayed_actions.pop(0)
        else:
            plant_action = base_action
            self._pending_joint_correction_rad = applied_correction

        swing_id = self.left_foot_id if active_side == "left" else self.right_foot_id
        previous_swing_height = float(self._sole_point(swing_id, True)[2])
        observation, reward, terminated, truncated, info = super().step(plant_action)
        swing_vertical_velocity = (
            float(self._sole_point(swing_id, True)[2]) - previous_swing_height
        ) / self.config.control_dt

        force_excess = max(float(info["swing_force_n"]) - 35.0, 0.0)
        landing_force_penalty = 0.0
        if gate > 0.0:
            landing_force_penalty = -self.landing_force_reward_scale * gate * (force_excess / 15.0) ** 2
        downward_velocity_penalty = 0.0
        if gate > 0.0 and float(info["swing_force_n"]) < 5.0:
            downward_excess = max(-swing_vertical_velocity - 0.02, 0.0)
            downward_velocity_penalty = -self.downward_velocity_reward_scale * gate * min(
                downward_excess / 0.10, 2.0
            ) ** 2
        correction_penalty = -self.correction_penalty_scale * gate * float(
            np.mean(correction**2)
        )
        correction_rate_penalty = -self.correction_rate_penalty_scale * gate * float(
            np.mean((correction - self._previous_correction) ** 2)
        )
        terminal_bonus = 0.0
        if truncated:
            terminal_bonus = self.terminal_success_bonus if info["is_success"] else -self.terminal_success_bonus
        reward += (
            landing_force_penalty
            + downward_velocity_penalty
            + correction_penalty
            + correction_rate_penalty
            + terminal_bonus
        )
        self._previous_correction = correction.copy()
        self._current_base_observation = observation.copy()
        self._current_observation = self._corrector_observation(
            self._current_base_observation,
            phase_kind=phase_kind,
            phase_progress=phase_progress,
            gate=gate,
            reset=False,
        )
        info.update(self._domain_info())
        info.update(
            {
                "landing_correction_gate": gate,
                "base_action_max_abs": float(np.max(np.abs(base_action))),
                "correction_action_max_abs": float(np.max(np.abs(correction))),
                "applied_correction_max_abs": float(np.max(np.abs(applied_correction))),
                "swing_vertical_velocity_m_s": swing_vertical_velocity,
                "landing_force_penalty_v19": landing_force_penalty,
                "downward_velocity_penalty_v19": downward_velocity_penalty,
                "correction_penalty_v19": correction_penalty,
                "correction_rate_penalty_v19": correction_rate_penalty,
                "terminal_bonus_v19": terminal_bonus,
            }
        )
        return self._current_observation.copy(), float(reward), terminated, truncated, info

    def _residual_joint_target_offset(
        self, filtered_action: np.ndarray, phase_kind: str, phase_progress: float
    ) -> np.ndarray:
        base_offset = super()._residual_joint_target_offset(
            filtered_action, phase_kind, phase_progress
        )
        return base_offset + self._pending_joint_correction_rad
