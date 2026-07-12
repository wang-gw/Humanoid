"""Environment for the urdf_f_v2 model (new CAD geometry).

Keeps the old urdf_f pipeline frozen: this subclass pins the v2 model/pose/foot
defaults and routes the walking reward to `rewards/urdf_f_v2_walking.py`, so the
v2 gait curriculum can diverge without ever editing `UrdfFEnv`/`GaitWalkingEnv`.

Divergence lives here:
  - model/pose/foot-body/stabilizer defaults → this class's __init__
  - walking reward shaping → rewards.urdf_f_v2_walking.compute_walking_reward
  - termination / obs tweaks → override the relevant method here as needed

The generic biped machinery (PD loop, contact sensing, gait clock, obs) is
inherited unchanged — it reads model dimensions dynamically, so it is genuinely
geometry-agnostic and does not need copying.
"""
from __future__ import annotations

from pathlib import Path

from envs.gait_walking_env import GaitWalkingEnv
from rewards.urdf_f_v2_walking import compute_walking_reward

PROJECT_ROOT = Path(__file__).resolve().parents[1]

V2_MODEL = str(PROJECT_ROOT / "envs/robots/urdf_f_v2/URDF_F_v2_footprint_contact.xml")
V2_POSE = str(PROJECT_ROOT / "configs/urdf_f_v2/quasistatic_standing_pose.json")
V2_LEFT_FOOT = "foot_L_1"
V2_RIGHT_FOOT = "foot_R_1"  # old model uses foot_R_v1_1


class GaitWalkingV2Env(GaitWalkingEnv):
    """Walking env for urdf_f_v2. Same control loop, v2 defaults + v2 reward."""

    def __init__(self, **kwargs) -> None:
        kwargs.setdefault("model_path", V2_MODEL)
        kwargs.setdefault("pose_path", V2_POSE)
        kwargs.setdefault("left_foot_body", V2_LEFT_FOOT)
        kwargs.setdefault("right_foot_body", V2_RIGHT_FOOT)
        super().__init__(**kwargs)

    def _reward(self, action):
        # v2 owns its walking reward; any non-walking task defers to the base.
        if self.task == "walking":
            return compute_walking_reward(self, action)
        return super()._reward(action)
