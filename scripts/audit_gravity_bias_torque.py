from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import mujoco
import numpy as np

from render_pd_standing import hinge_joint_info, load_targets
from sweep_stabilized_standing import initialize


def actuator_force_limit(model: mujoco.MjModel, actuator_id: int) -> float | None:
    if not bool(model.actuator_forcelimited[actuator_id]):
        return None
    low, high = model.actuator_forcerange[actuator_id]
    return float(max(abs(low), abs(high)))


def main() -> int:
    parser = argparse.ArgumentParser(description="Audit static gravity bias torque at a standing pose.")
    parser.add_argument("--model", type=Path, required=True)
    parser.add_argument("--pose-json", type=Path, required=True)
    parser.add_argument("--out-dir", type=Path, default=Path("outputs/analysis/gravity_bias_torque"))
    args = parser.parse_args()

    out_dir = args.out_dir.resolve()
    out_dir.mkdir(parents=True, exist_ok=True)

    model = mujoco.MjModel.from_xml_path(str(args.model.resolve()))
    data = mujoco.MjData(model)
    joints = hinge_joint_info(model)
    base_z, target_q = load_targets(model, joints, args.pose_json.resolve())
    initialize(model, data, joints, base_z, target_q)
    data.qvel[:] = 0.0
    data.qacc[:] = 0.0
    mujoco.mj_forward(model, data)

    actuator_by_joint = {}
    for actuator_id in range(model.nu):
        joint_id = int(model.actuator_trnid[actuator_id, 0])
        if joint_id < 0:
            continue
        joint_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, joint_id) or f"joint_{joint_id}"
        actuator_by_joint[joint_name] = actuator_id

    rows = []
    for joint in joints:
        joint_name = str(joint["name"])
        dofadr = int(joint["dofadr"])
        qposadr = int(joint["qposadr"])
        actuator_id = actuator_by_joint.get(joint_name)
        actuator_name = None
        limit = None
        usage = None
        if actuator_id is not None:
            actuator_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, actuator_id) or f"actuator_{actuator_id}"
            limit = actuator_force_limit(model, actuator_id)
            if limit is not None and limit > 0.0:
                usage = abs(float(data.qfrc_bias[dofadr])) / limit
        rows.append(
            {
                "joint": joint_name,
                "actuator": actuator_name or "",
                "q_rad": float(data.qpos[qposadr]),
                "dofadr": dofadr,
                "gravity_bias_torque_nm": float(data.qfrc_bias[dofadr]),
                "abs_gravity_bias_torque_nm": abs(float(data.qfrc_bias[dofadr])),
                "actuator_force_limit_nm": limit if limit is not None else "",
                "limit_usage": usage if usage is not None else "",
            }
        )

    csv_path = out_dir / "gravity_bias_torque.csv"
    with csv_path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)

    max_row = max(rows, key=lambda row: float(row["abs_gravity_bias_torque_nm"]))
    payload = {
        "model": str(args.model.resolve()),
        "pose_json": str(args.pose_json.resolve()),
        "base_z": float(base_z),
        "note": "qfrc_bias at zero velocity/acceleration. This is a free-base gravity-bias check, not a full constrained contact inverse-dynamics solution.",
        "max_abs_gravity_bias_torque_nm": float(max_row["abs_gravity_bias_torque_nm"]),
        "max_abs_joint": max_row["joint"],
        "rows": rows,
        "csv": str(csv_path),
    }
    json_path = out_dir / "gravity_bias_torque_summary.json"
    json_path.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(json.dumps(payload, indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
