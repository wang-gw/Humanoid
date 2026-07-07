"""UrdfFEnv with mj_copyData warm-start for accurate sequence-context training."""
from __future__ import annotations

import mujoco
import numpy as np

from envs.urdf_f_env import UrdfFEnv


class WarmStartEnv(UrdfFEnv):
    """Each episode resets to a pre-computed warm state (including contact history).

    This replicates the exact MuJoCo state (qpos, qvel, contact forces) that
    occurs at a specific point in the walking sequence, without re-simulating
    the prior stages every episode. Cost: one mj_copyData call per reset.

    jitter_qpos_std / jitter_qvel_std: Gaussian noise added to qpos/qvel after
    copying, so VecNormalize learns a non-zero variance distribution rather than
    a single point (which would cause all obs to saturate at ±clip when the
    policy is used in the actual sequence context).
    """

    def __init__(
        self,
        warm_data: mujoco.MjData,
        warm_base_z: float,
        jitter_qpos_std: float = 0.002,
        jitter_qvel_std: float = 0.05,
        **kwargs,
    ):
        super().__init__(**kwargs)
        self._warm_data = warm_data
        self._warm_base_z = warm_base_z
        self._jitter_qpos_std = jitter_qpos_std
        self._jitter_qvel_std = jitter_qvel_std

    def reset(self, seed: int | None = None, options: dict | None = None):
        # Standard init (sets model, data structures) then override with warm state
        super().reset(seed=seed, options=options)
        mujoco.mj_copyData(self.data, self.model, self._warm_data)
        # Add jitter so VecNorm learns a distribution, not a degenerate point
        if self._jitter_qpos_std > 0:
            self.data.qpos[7:] += self.np_random.normal(
                0, self._jitter_qpos_std, self.data.qpos[7:].shape
            )
        if self._jitter_qvel_std > 0:
            self.data.qvel[6:] += self.np_random.normal(
                0, self._jitter_qvel_std, self.data.qvel[6:].shape
            )
        mujoco.mj_forward(self.model, self.data)
        self.prev_action[:] = 0.0
        self.base_z = self._warm_base_z
        obs = self._obs()
        return obs, self._info("")
