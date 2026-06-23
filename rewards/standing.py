from __future__ import annotations

import numpy as np


class StandingReward:
    """Standing-stage reward with alive bonus and penalties."""

    def __init__(
        self,
        alive_bonus: float = 1.0,
        w_upright: float = 1.0,
        w_ang: float = 0.05,
        w_height: float = 0.0,
        w_dact: float = 0.01,
        w_joint_dev: float = 0.02,
        terminal_penalty: float = 1.0,
    ) -> None:
        self.alive_bonus = float(alive_bonus)
        self.w_upright = float(w_upright)
        self.w_ang = float(w_ang)
        self.w_height = float(w_height)
        self.w_dact = float(w_dact)
        self.w_joint_dev = float(w_joint_dev)
        self.terminal_penalty = float(terminal_penalty)

        self._z0: float | None = None
        self._prev_action: np.ndarray | None = None

    def reset(self, env: object) -> None:
        # Use XML home pose height as fixed reference.
        self._z0 = float(getattr(env, "_home_z", env.data.xpos[env._trunk_body_id][2]))
        self._prev_action = np.asarray(env.prev_action, dtype=np.float32).copy()

    def compute(self, env: object, action: np.ndarray) -> tuple[float, dict[str, float]]:
        if self._z0 is None or self._prev_action is None:
            self.reset(env)

        projected_gravity = env._get_projected_gravity()
        p_upright = float(projected_gravity[0] ** 2 + projected_gravity[1] ** 2)

        ang_vel = np.asarray(env.data.qvel[3:6], dtype=np.float32)
        p_ang = float(np.sum(ang_vel ** 2))

        z = float(env.data.xpos[env._trunk_body_id][2])
        p_height = float((z - self._z0) ** 2)

        action = np.asarray(action, dtype=np.float32)
        dact = action - self._prev_action
        p_dact = float(np.mean(dact ** 2)) if dact.size > 0 else 0.0

        joint_pos = np.asarray(env.data.qpos[7:], dtype=np.float32)
        home_joint = np.asarray(env.home_joint_pos, dtype=np.float32)

        # Normalize by physical actuator ctrl range, not residual policy range.
        if env.action_space.shape[0] > 0:
            ctrl_high = np.asarray(getattr(env, "_ctrl_high", env.action_space.high), dtype=np.float32)
            ctrl_low = np.asarray(getattr(env, "_ctrl_low", env.action_space.low), dtype=np.float32)
            joint_range = ctrl_high - ctrl_low
            joint_range = np.maximum(joint_range, 1e-6)
            n = min(joint_pos.shape[0], home_joint.shape[0], joint_range.shape[0])
            joint_err = (joint_pos[:n] - home_joint[:n]) / joint_range[:n]
            p_joint_dev = float(np.mean(joint_err ** 2))
        else:
            p_joint_dev = 0.0

        is_fallen, _, _ = env._check_fall()
        p_terminal = 1.0 if is_fallen else 0.0

        penalty = (
            self.w_upright * p_upright
            + self.w_ang * p_ang
            + self.w_height * p_height
            + self.w_dact * p_dact
            + self.w_joint_dev * p_joint_dev
            + self.terminal_penalty * p_terminal
        )

        reward = self.alive_bonus - penalty

        # Keep legacy metric names for compatibility with existing evaluators.
        r_upright = -p_upright
        r_ang = -p_ang
        r_height = -p_height
        r_dact = -p_dact
        r_joint_dev = -p_joint_dev

        self._prev_action = action.copy()

        return reward, {
            "reward_total": reward,
            "reward_alive": self.alive_bonus,
            "reward_penalty_total": penalty,
            "reward_upright": r_upright,
            "reward_ang": r_ang,
            "reward_height": r_height,
            "reward_dact": r_dact,
            "reward_act": r_dact,
            "reward_joint_dev": r_joint_dev,
            "reward_terminal": -self.terminal_penalty * p_terminal,
        }
