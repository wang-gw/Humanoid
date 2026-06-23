from __future__ import annotations

from typing import Any

import numpy as np


def home_action(env: Any) -> np.ndarray:
    if env.model.nu == 0:
        return np.zeros(0, dtype=np.float32)

    home_joint = np.asarray(env.home_joint_pos, dtype=np.float32)
    if home_joint.shape[0] >= env.model.nu:
        base = home_joint[: env.model.nu].copy()
    else:
        base = 0.5 * (env._ctrl_low + env._ctrl_high)
    return np.clip(base, env._ctrl_low, env._ctrl_high).astype(np.float32)
