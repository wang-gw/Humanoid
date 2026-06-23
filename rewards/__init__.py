from __future__ import annotations

from .base import ForwardVelocityReward, RewardModule, ZeroReward
from .standing import StandingReward
from .walking import WalkingReward
from .reference_walking import ReferenceWalkingReward


def build_reward(name: str) -> RewardModule:
    key = name.strip().lower()
    if key in {"forward_velocity", "forward", "baseline"}:
        return ForwardVelocityReward()
    if key in {"standing", "stand"}:
        return StandingReward()
    if key in {"walking", "walk", "locomotion"}:
        return WalkingReward()
    if key in {"reference_walking", "reference_walk", "imitation_walk"}:
        raise ValueError(
            "reference_walking requires an explicit motion_path. Use scripts/train.py "
            "--reward reference_walking --motion-path <path>."
        )
    if key in {"zero", "none", "stub"}:
        return ZeroReward()
    valid = "forward_velocity, standing, walking, reference_walking, zero"
    raise ValueError(f"Unknown reward '{name}'. Available: {valid}")


__all__ = [
    "RewardModule",
    "ZeroReward",
    "ForwardVelocityReward",
    "StandingReward",
    "WalkingReward",
    "ReferenceWalkingReward",
    "build_reward",
]
