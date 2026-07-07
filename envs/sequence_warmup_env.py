"""Gymnasium env wrapper that runs a policy sequence as warmup before each episode."""
from __future__ import annotations

from pathlib import Path
from typing import Any

import numpy as np
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv, VecNormalize

from envs.urdf_f_env import UrdfFEnv


class SequenceWarmupEnv(UrdfFEnv):
    """UrdfFEnv that replays prior-stage policies before each training episode.

    This ensures the wsr (or any target stage) policy is trained from the
    *exact* physical state that occurs in the walking sequence, rather than
    from an isolated pose-file reset which has different contact history.
    """

    def __init__(
        self,
        warmup_stages: list[dict],
        **kwargs,
    ):
        super().__init__(**kwargs)
        self._warmup_stages = warmup_stages
        self._warmup_policies: list[tuple[PPO, VecNormalize]] | None = None

    def _load_warmup_policies(self) -> list[tuple[PPO, VecNormalize]]:
        policies = []
        for stage in self._warmup_stages:
            dummy = DummyVecEnv([lambda: UrdfFEnv()])
            vn = VecNormalize.load(stage["vecnormalize_path"], dummy)
            vn.training = False
            vn.norm_reward = False
            m = PPO.load(stage["policy_path"], device="cpu")
            policies.append((m, vn))
        return policies

    def reset(self, seed: int | None = None, options: dict | None = None):
        obs, info = super().reset(seed=seed, options=options)

        if self._warmup_policies is None:
            self._warmup_policies = self._load_warmup_policies()

        import mujoco

        for (m, vn), stage in zip(self._warmup_policies, self._warmup_stages):
            # Apply stage env params
            for k, v in stage.get("env_params", {}).items():
                setattr(self, k, v)

            # Run warmup stage
            for _ in range(stage["steps"]):
                n = vn.normalize_obs(obs.reshape(1, -1))
                act, _ = m.predict(n, deterministic=True)
                obs, _, terminated, _, info = self.step(
                    np.asarray(act).reshape(self.action_space.shape).astype(np.float32)
                )
                if terminated:
                    # If terminated during warmup, restart from scratch
                    obs, info = super().reset(seed=seed, options=options)
                    break

            # Apply transition resets if specified
            if stage.get("reset_qvel", False):
                self.data.qvel[:] = 0.0
                mujoco.mj_forward(self.model, self.data)
            if stage.get("reset_prev_action", False):
                self.prev_action[:] = 0.0
            if stage.get("reset_base_z", False):
                self.base_z = float(self.data.qpos[2])

            obs = self._obs()

        # Now apply the target stage env params
        for k, v in (options or {}).get("target_env_params", {}).items():
            setattr(self, k, v)

        return obs, self._info("")
