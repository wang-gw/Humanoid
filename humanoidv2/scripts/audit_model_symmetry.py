#!/usr/bin/env python3
"""Measure sagittal-mirror COM and inertia errors in a MuJoCo model."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from humanoidv2 import KHR3HVEnv  # noqa: E402
from scripts.build_symmetric_inertia_model import BODY_PAIRS, MIRROR, inertia_tensor  # noqa: E402


def audit(path: Path) -> dict[str, object]:
    path = path.resolve()
    model = mujoco.MjModel.from_xml_path(str(path))
    pair_results = []
    for left_name, right_name in BODY_PAIRS:
        left_id = model.body(left_name).id
        right_id = model.body(right_name).id
        center_error = model.body_ipos[right_id] - MIRROR @ model.body_ipos[left_id]
        left_tensor = inertia_tensor(model, left_id)
        tensor_error = inertia_tensor(model, right_id) - MIRROR @ left_tensor @ MIRROR
        pair_results.append(
            {
                "left": left_name,
                "right": right_name,
                "mass_error_kg": float(model.body_mass[right_id] - model.body_mass[left_id]),
                "com_mirror_error_m": center_error.tolist(),
                "com_mirror_error_norm_m": float(np.linalg.norm(center_error)),
                "inertia_mirror_relative_error": float(
                    np.linalg.norm(tensor_error) / max(np.linalg.norm(left_tensor), 1.0e-12)
                ),
            }
        )

    base_id = model.body("base_link").id
    base_tensor = inertia_tensor(model, base_id)
    base_mirror_error = base_tensor - MIRROR @ base_tensor @ MIRROR
    env = KHR3HVEnv(model_path=path)
    env.reset(seed=7)
    whole_com = (env.model.body_mass[:, None] * env.data.xipos).sum(axis=0) / env.model.body_mass.sum()
    feet_midpoint = 0.5 * (
        env.data.xpos[env.left_foot_id] + env.data.xpos[env.right_foot_id]
    )
    result = {
        "model": str(path),
        "total_mass_kg": float(model.body_mass.sum()),
        "neutral_whole_body_com_world_m": whole_com.tolist(),
        "neutral_feet_midpoint_world_m": feet_midpoint.tolist(),
        "neutral_com_lateral_offset_from_feet_midpoint_m": float(whole_com[0] - feet_midpoint[0]),
        "base_com_lateral_offset_m": float(model.body_ipos[base_id, 0]),
        "base_inertia_mirror_relative_error": float(
            np.linalg.norm(base_mirror_error) / np.linalg.norm(base_tensor)
        ),
        "max_pair_com_mirror_error_m": max(row["com_mirror_error_norm_m"] for row in pair_results),
        "max_pair_inertia_mirror_relative_error": max(
            row["inertia_mirror_relative_error"] for row in pair_results
        ),
        "pairs": pair_results,
    }
    env.close()
    return result


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", type=Path, required=True)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    result = audit(args.model)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(result, indent=2), encoding="utf-8")
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
