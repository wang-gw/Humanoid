"""Eight forward steps with measured-state online IK re-anchoring."""

from __future__ import annotations

from dataclasses import replace
from pathlib import Path
from typing import Any

import mujoco
import numpy as np

from .four_step_v7_env import FourStepV7Env


class EightStepV8Env(FourStepV7Env):
    """Rebuild each step reference from the measured landing state."""

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
        reanchor_height_gain: float = 0.20,
        reanchor_mode: str = "joint",
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
        )
        self.num_steps = 8
        self.minimum_forward_m = 0.180
        self.reanchor_height_gain = reanchor_height_gain
        if reanchor_mode not in {"joint", "world"}:
            raise ValueError("reanchor_mode must be 'joint' or 'world'")
        self.reanchor_mode = reanchor_mode
        self.config = replace(
            self.config,
            max_episode_steps=self.num_steps * self.step_cycle_steps,
        )
        self._online_targets: dict[int, tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]] = {}
        self._anchor_forward = np.zeros(self.num_steps, dtype=np.float64)
        self._anchor_base_forward = np.zeros(self.num_steps, dtype=np.float64)

    def _step_sides(self, first_side: str | None = None) -> tuple[str, ...]:
        first = first_side or self.first_side
        second = "right" if first == "left" else "left"
        return tuple(first if index % 2 == 0 else second for index in range(self.num_steps))

    def _build_online_step_targets(
        self, step_index: int
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        active_side = self._step_sides()[step_index]
        stored_qpos = self.data.qpos.copy()
        stored_qvel = self.data.qvel.copy()
        stored_ctrl = self.data.ctrl.copy()
        stored_time = float(self.data.time)
        # Re-anchor horizontal position to the measured state. Correct only a
        # fraction of height sag per step; preserving all sag compounds crouch,
        # while an immediate nominal-height correction is dynamically abrupt.
        base_qpos = stored_qpos.copy()
        base_qpos[2] += self.reanchor_height_gain * (self.home_qpos[2] - stored_qpos[2])
        start_joints = stored_qpos[self.qpos_ids].copy()
        starts = {
            "left": self._foot_output(self.left_foot_id).copy(),
            "right": self._foot_output(self.right_foot_id).copy(),
        }
        foot_center_y = 0.5 * (starts["left"][1] + starts["right"][1])
        self._anchor_forward[step_index] = self.forward_sign * foot_center_y
        self._anchor_base_forward[step_index] = self.forward_sign * (
            stored_qpos[1] - self._start_base_y
        )

        if self.reanchor_mode == "joint":
            # Start from the measured landing joints, then converge to the
            # validated nominal alternating target. Reusing step 3/4 targets
            # preserves the periodic A<->B split stance beyond four steps.
            template_index = step_index if step_index < 4 else 2 + step_index % 2
            nominal = self._targets[self.first_side]
            lifted, advanced, landed = nominal[
                1 + 3 * template_index:4 + 3 * template_index
            ]
            return start_joints, lifted.copy(), advanced.copy(), landed.copy()

        half_stride = 0.5 * self.forward_sign * self.stride_m
        desired_y = {
            active_side: foot_center_y + half_stride,
            ("right" if active_side == "left" else "left"): foot_center_y - half_stride,
        }
        lateral_sign = 1.0 if active_side == "left" else -1.0

        def solve(targets: dict[str, np.ndarray], seed: np.ndarray) -> np.ndarray:
            joints = self._solve_leg_ik(
                base_qpos, seed, slice(0, 5), self.left_foot_id, targets["left"]
            )
            return self._solve_leg_ik(
                base_qpos, joints, slice(5, 10), self.right_foot_id, targets["right"]
            )

        lifted_targets = {side: output.copy() for side, output in starts.items()}
        for target in lifted_targets.values():
            target[0] += lateral_sign * self.sway_width_m
        lifted_targets[active_side][2] += self.lift_m
        lifted = solve(lifted_targets, start_joints)

        advanced_targets = {side: output.copy() for side, output in lifted_targets.items()}
        for side in advanced_targets:
            advanced_targets[side][1] = desired_y[side]
        advanced = solve(advanced_targets, lifted)

        landed_targets = {side: output.copy() for side, output in starts.items()}
        for side in landed_targets:
            landed_targets[side][1] = desired_y[side]
        landed = solve(landed_targets, advanced)

        self.data.qpos[:] = stored_qpos
        self.data.qvel[:] = stored_qvel
        self.data.ctrl[:] = stored_ctrl
        self.data.time = stored_time
        mujoco.mj_forward(self.model, self.data)
        return start_joints, lifted, advanced, landed

    def _reference_target(self) -> tuple[np.ndarray, str, float, str, int]:
        step_index = min(self._step_count // self.step_cycle_steps, self.num_steps - 1)
        local_step = self._step_count - step_index * self.step_cycle_steps
        active_side = self._step_sides()[step_index]
        if step_index not in self._online_targets:
            self._online_targets[step_index] = self._build_online_step_targets(step_index)
        start, lifted, advanced, landed = self._online_targets[step_index]
        segments = [(start, lifted, self.lift_steps, "lift")]
        if self.lift_hold_steps:
            segments.append((lifted, lifted, self.lift_hold_steps, "hold"))
        segments.extend(
            (
                (lifted, advanced, self.advance_steps, "advance"),
                (advanced, landed, self.land_steps, "land"),
                (landed, landed, self.settle_steps, "settle"),
            )
        )
        offset = 0
        for segment_start, segment_end, duration, phase_kind in segments:
            if local_step < offset + duration:
                fraction = min((local_step - offset + 1) / duration, 1.0)
                blend = self._smoothstep(fraction)
                target = segment_start + blend * (segment_end - segment_start)
                return target, f"step{step_index + 1}_{phase_kind}", fraction, active_side, step_index
            offset += duration
        return landed.copy(), f"step{step_index + 1}_settle", 1.0, active_side, step_index

    def reset(
        self,
        *,
        seed: int | None = None,
        options: dict[str, Any] | None = None,
    ) -> tuple[np.ndarray, dict[str, Any]]:
        observation, info = super().reset(seed=seed, options=options)
        self._online_targets = {}
        self._anchor_forward = np.zeros(self.num_steps, dtype=np.float64)
        self._anchor_base_forward = np.zeros(self.num_steps, dtype=np.float64)
        info["reanchor_mode"] = self.reanchor_mode
        return observation, info

    def step(self, action: np.ndarray):
        observation, reward, terminated, truncated, info = super().step(action)
        info["reanchor_mode"] = self.reanchor_mode
        for index in range(self.num_steps):
            info[f"step_{index + 1}_anchor_forward_m"] = float(self._anchor_forward[index])
            info[f"step_{index + 1}_anchor_base_forward_m"] = float(self._anchor_base_forward[index])
        return observation, reward, terminated, truncated, info
