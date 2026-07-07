from __future__ import annotations

import numpy as np
from gymnasium import spaces

from envs.urdf_f_env import UrdfFEnv, DEFAULT_MODEL, DEFAULT_POSE


class GaitWalkingEnv(UrdfFEnv):
    """End-to-end walking env with a gait clock signal in the observation.

    A periodic phase variable cycles 0→2π every gait_period_steps.
    sin(phase) > 0  → right-foot swing phase
    sin(phase) < 0  → left-foot swing phase
    The policy receives [sin(phase), cos(phase)] so it can infer timing.
    """

    def __init__(
        self,
        gait_period_steps: int = 400,
        **kwargs,
    ) -> None:
        kwargs.setdefault("task", "walking")
        super().__init__(**kwargs)
        self.gait_period_steps = int(gait_period_steps)
        self._gait_step: int = 0

        # Extend obs space for [sin(phase), cos(phase)]
        low = np.concatenate([self.observation_space.low, np.array([-1.0, -1.0], dtype=np.float32)])
        high = np.concatenate([self.observation_space.high, np.array([1.0, 1.0], dtype=np.float32)])
        self.observation_space = spaces.Box(low=low, high=high, dtype=np.float32)

    def _gait_phase(self) -> float:
        return 2.0 * np.pi * (self._gait_step % self.gait_period_steps) / self.gait_period_steps

    def _obs(self) -> np.ndarray:
        base = super()._obs()
        phase = self._gait_phase()
        return np.concatenate([base, [np.sin(phase), np.cos(phase)]]).astype(np.float32)

    def reset(self, **kwargs):
        self._gait_step = 0
        return super().reset(**kwargs)

    def step(self, action):
        obs, reward, terminated, truncated, info = super().step(action)
        self._gait_step += 1
        return obs, reward, terminated, truncated, info
