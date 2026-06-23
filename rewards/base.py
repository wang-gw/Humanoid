from __future__ import annotations

from typing import Protocol

import numpy as np


class RewardModule(Protocol):
    """Interface for pluggable reward modules."""

    def compute(self, env: object, action: np.ndarray) -> tuple[float, dict[str, float]]:
        ...


class ZeroReward:
    """Placeholder reward for environment/debug tests."""

    def compute(self, env: object, action: np.ndarray) -> tuple[float, dict[str, float]]:
        del env, action
        return 0.0, {"reward_total": 0.0}


class ForwardVelocityReward:
    """Simple baseline reward before task-specific design."""

    def compute(self, env: object, action: np.ndarray) -> tuple[float, dict[str, float]]:
        del action
        reward = float(env.data.qvel[0]) if env.data.qvel.size > 0 else 0.0
        return reward, {"reward_total": reward, "reward_forward_velocity": reward}
