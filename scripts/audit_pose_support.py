from __future__ import annotations

import argparse
import json
from pathlib import Path

import mujoco
import numpy as np

from render_pd_standing import hinge_joint_info, load_targets
from sweep_stabilized_standing import initialize, total_com


FOOT_GEOMS = ("foot_L_1_sole_collision", "foot_R_v1_1_sole_collision")


def box_corners(data: mujoco.MjData, model: mujoco.MjModel, geom_id: int) -> np.ndarray:
    center = np.asarray(data.geom_xpos[geom_id], dtype=np.float64)
    xmat = np.asarray(data.geom_xmat[geom_id], dtype=np.float64).reshape(3, 3)
    half = np.asarray(model.geom_size[geom_id], dtype=np.float64)
    corners = []
    for sx in (-1.0, 1.0):
        for sy in (-1.0, 1.0):
            for sz in (-1.0, 1.0):
                local = np.array([sx * half[0], sy * half[1], sz * half[2]], dtype=np.float64)
                corners.append(center + xmat @ local)
    return np.asarray(corners)


def margin(point_xy: np.ndarray, bounds: dict[str, float]) -> dict[str, float | bool]:
    x = float(point_xy[0])
    y = float(point_xy[1])
    return {
        "inside_x": bounds["x_min"] <= x <= bounds["x_max"],
        "inside_y": bounds["y_min"] <= y <= bounds["y_max"],
        "x_min_margin": x - bounds["x_min"],
        "x_max_margin": bounds["x_max"] - x,
        "y_min_margin": y - bounds["y_min"],
        "y_max_margin": bounds["y_max"] - y,
        "min_margin": min(x - bounds["x_min"], bounds["x_max"] - x, y - bounds["y_min"], bounds["y_max"] - y),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description="Audit COM/support/contact geometry for a standing pose.")
    parser.add_argument("--model", type=Path, required=True)
    parser.add_argument("--pose-json", type=Path, required=True)
    parser.add_argument("--out-dir", type=Path, default=Path("outputs/analysis/pose_support_audit"))
    args = parser.parse_args()

    out_dir = args.out_dir.resolve()
    out_dir.mkdir(parents=True, exist_ok=True)

    model = mujoco.MjModel.from_xml_path(str(args.model.resolve()))
    data = mujoco.MjData(model)
    joints = hinge_joint_info(model)
    base_z, target_q = load_targets(model, joints, args.pose_json.resolve())
    initialize(model, data, joints, base_z, target_q)

    com = total_com(model, data)
    all_corners = []
    foot_records = []
    for name in FOOT_GEOMS:
        geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name)
        if geom_id < 0:
            raise ValueError(f"Missing foot geom: {name}")
        corners = box_corners(data, model, geom_id)
        all_corners.append(corners)
        foot_records.append(
            {
                "name": name,
                "center": [float(v) for v in data.geom_xpos[geom_id]],
                "halfsize": [float(v) for v in model.geom_size[geom_id]],
                "lowest_z": float(np.min(corners[:, 2])),
                "highest_z": float(np.max(corners[:, 2])),
                "x_min": float(np.min(corners[:, 0])),
                "x_max": float(np.max(corners[:, 0])),
                "y_min": float(np.min(corners[:, 1])),
                "y_max": float(np.max(corners[:, 1])),
                "corners": [[float(v) for v in row] for row in corners],
            }
        )

    corners_all = np.vstack(all_corners)
    bounds = {
        "x_min": float(np.min(corners_all[:, 0])),
        "x_max": float(np.max(corners_all[:, 0])),
        "y_min": float(np.min(corners_all[:, 1])),
        "y_max": float(np.max(corners_all[:, 1])),
    }
    payload = {
        "model": str(args.model.resolve()),
        "pose_json": str(args.pose_json.resolve()),
        "base_z": float(base_z),
        "total_mass": float(np.sum(model.body_mass)),
        "com_world": [float(v) for v in com],
        "support_bounds_xy": bounds,
        "support_margin": margin(com[:2], bounds),
        "lowest_contact_z": float(np.min(corners_all[:, 2])),
        "highest_contact_z": float(np.max(corners_all[:, 2])),
        "initial_contacts": int(data.ncon),
        "feet": foot_records,
    }
    out_path = out_dir / "pose_support_audit.json"
    out_path.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(json.dumps(payload, indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
