from __future__ import annotations

STAND = "STAND"
WALK = "WALK"

MODE_TO_STAGE = {
    STAND: "stand",
    WALK: "walk",
}

STAGE_TO_MODE = {stage: mode for mode, stage in MODE_TO_STAGE.items()}
