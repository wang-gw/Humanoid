from __future__ import annotations

from typing import Any

import numpy as np


def build_nominal_action(env: Any, robot: str, controller: str) -> np.ndarray:
    if robot == "go1":
        from policies.go1 import nominal_action

        return nominal_action(env, controller)
    if robot == "g1":
        from policies.g1 import nominal_action

        return nominal_action(env, controller)
    raise ValueError(f"Unknown robot '{robot}'.")
