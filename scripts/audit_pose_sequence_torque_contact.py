from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import mujoco
import numpy as np

from render_axis_aware_standing import PITCH_ROLE_ACTUATORS, ROLL_ROLE_ACTUATORS
from render_pd_standing import hinge_joint_info, load_targets, quat_to_roll_pitch_yaw
from render_pose_sequence import load_pose_list, phase_target
from render_weight_shift import contact_stats, foot_geom_ids
from sweep_stabilized_standing import initialize, support_center, total_com


def contact_force_by_geom(model: mujoco.MjModel, data: mujoco.MjData) -> dict[str, float]:
    force = np.zeros(6, dtype=np.float64)
    totals: dict[str, float] = {}
    for contact_id in range(data.ncon):
        contact = data.contact[contact_id]
        mujoco.mj_contactForce(model, data, contact_id, force)
        normal_force = max(float(force[0]), 0.0)
        for geom_id in (int(contact.geom1), int(contact.geom2)):
            name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, geom_id) or f"geom_{geom_id}"
            if "sole" in name:
                totals[name] = totals.get(name, 0.0) + normal_force
    return totals


def main() -> int:
    parser = argparse.ArgumentParser(description="Audit torque/contact timeline for a pose sequence.")
    parser.add_argument("--model", type=Path, required=True)
    parser.add_argument("--poses", type=Path, nargs="+", required=True)
    parser.add_argument("--out-dir", type=Path, required=True)
    parser.add_argument("--ramp", type=float, default=4.0)
    parser.add_argument("--hold", type=float, default=4.0)
    parser.add_argument("--duration", type=float, default=0.0)
    parser.add_argument("--log-every", type=int, default=1)
    parser.add_argument("--joint-kp", type=float, default=20.0)
    parser.add_argument("--joint-kd", type=float, default=12.0)
    parser.add_argument("--torque-limit", type=float, default=30.0)
    parser.add_argument("--kp-att", type=float, default=0.0)
    parser.add_argument("--kd-att", type=float, default=1.0)
    parser.add_argument("--kcom", type=float, default=1.0)
    parser.add_argument("--roll-sign", type=float, default=-1.0)
    parser.add_argument("--pitch-sign", type=float, default=1.0)
    args = parser.parse_args()

    out_dir = args.out_dir.resolve()
    out_dir.mkdir(parents=True, exist_ok=True)
    model = mujoco.MjModel.from_xml_path(str(args.model.resolve()))
    data = mujoco.MjData(model)
    joints = hinge_joint_info(model)
    actuator_names = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, idx) or f"actuator_{idx}" for idx in range(model.nu)]
    base_zs, targets = load_pose_list(model, joints, args.poses)
    initialize(model, data, joints, base_zs[0], targets[0])
    geom_to_side = foot_geom_ids(model)

    duration = args.duration if args.duration > 0.0 else (len(targets) - 1) * (args.ramp + args.hold)
    steps = int(duration / model.opt.timestep)
    rows: list[dict[str, float]] = []
    geom_force_rows: list[dict[str, float | str]] = []
    max_abs_tau = 0.0
    max_qvel = 0.0
    max_contact = 0.0
    max_abs_roll = 0.0
    first_roll_thresholds: dict[str, float] = {}

    for step in range(steps + 1):
        phase, alpha, target_base_z, target_q = phase_target(float(data.time), targets, base_zs, args.ramp, args.hold)
        q = np.array([data.qpos[int(joint["qposadr"])] for joint in joints], dtype=np.float64)
        qd = np.array([data.qvel[int(joint["dofadr"])] for joint in joints], dtype=np.float64)
        roll, pitch, yaw = quat_to_roll_pitch_yaw(data.qpos[3:7])
        com = total_com(model, data)
        center, half = support_center(model, data)
        com_err = (com[:2] - center) / np.maximum(half, 1e-6)
        wx = float(data.qvel[3]) if model.nv >= 6 else 0.0
        wy = float(data.qvel[4]) if model.nv >= 6 else 0.0
        roll_term = args.kp_att * roll + args.kd_att * wx + args.kcom * float(com_err[1])
        pitch_term = args.kp_att * pitch + args.kd_att * wy + args.kcom * float(com_err[0])

        pd_tau = args.joint_kp * (target_q - q) - args.joint_kd * qd
        stabilizer_tau = np.zeros(model.nu, dtype=np.float64)
        for idx, name in enumerate(actuator_names):
            if name in ROLL_ROLE_ACTUATORS:
                stabilizer_tau[idx] += args.roll_sign * roll_term
            if name in PITCH_ROLE_ACTUATORS:
                stabilizer_tau[idx] += args.pitch_sign * pitch_term
        raw_tau = pd_tau + stabilizer_tau
        tau = np.clip(raw_tau, -args.torque_limit, args.torque_limit)
        data.ctrl[:] = tau

        stats = contact_stats(model, data, geom_to_side)
        qvel = float(np.linalg.norm(data.qvel))
        max_abs_tau = max(max_abs_tau, float(np.max(np.abs(data.actuator_force))) if model.nu else 0.0)
        max_qvel = max(max_qvel, qvel)
        max_contact = max(max_contact, stats["normal_force"])
        max_abs_roll = max(max_abs_roll, abs(float(roll)))
        for threshold in (0.1, 0.25, 0.5, 1.0, 1.5, 2.0):
            key = f"roll_abs_ge_{threshold:g}"
            if key not in first_roll_thresholds and abs(float(roll)) >= threshold:
                first_roll_thresholds[key] = float(data.time)

        if step % max(1, args.log_every) == 0 or step == steps:
            row: dict[str, float] = {
                "time": float(data.time),
                "phase": float(phase),
                "alpha": float(alpha),
                "target_base_z": float(target_base_z),
                "base_z": float(data.qpos[2]),
                "roll": float(roll),
                "pitch": float(pitch),
                "yaw": float(yaw),
                "wx": wx,
                "wy": wy,
                "qvel_norm": qvel,
                "com_x": float(com[0]),
                "com_y": float(com[1]),
                "com_z": float(com[2]),
                "support_x": float(center[0]),
                "support_y": float(center[1]),
                "com_err_x": float(com_err[0]),
                "com_err_y": float(com_err[1]),
                "roll_term": float(roll_term),
                "pitch_term": float(pitch_term),
                **{key: float(value) for key, value in stats.items()},
                "max_abs_tau_so_far": float(max_abs_tau),
            }
            for idx, name in enumerate(actuator_names):
                row[f"{name}_pd_tau"] = float(pd_tau[idx])
                row[f"{name}_stabilizer_tau"] = float(stabilizer_tau[idx])
                row[f"{name}_raw_tau"] = float(raw_tau[idx])
                row[f"{name}_ctrl_tau"] = float(tau[idx])
                row[f"{name}_actuator_force"] = float(data.actuator_force[idx])
            rows.append(row)
            geom_forces = contact_force_by_geom(model, data)
            for geom_name, force in geom_forces.items():
                geom_force_rows.append({"time": float(data.time), "phase": float(phase), "geom": geom_name, "normal_force": float(force)})

        if step < steps:
            mujoco.mj_step(model, data)

    csv_path = out_dir / "sequence_torque_contact_timeline.csv"
    with csv_path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)

    geom_csv_path = out_dir / "sequence_contact_force_by_geom.csv"
    with geom_csv_path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=["time", "phase", "geom", "normal_force"])
        writer.writeheader()
        writer.writerows(geom_force_rows)

    final = rows[-1]
    summary = {
        "model": str(args.model.resolve()),
        "poses": [str(path.resolve()) for path in args.poses],
        "duration": duration,
        "ramp": args.ramp,
        "hold": args.hold,
        "log_every": args.log_every,
        "first_roll_thresholds": first_roll_thresholds,
        "max_qvel_norm": max_qvel,
        "max_contact_force": max_contact,
        "max_abs_tau": max_abs_tau,
        "max_abs_roll": max_abs_roll,
        "final": {key: final[key] for key in ("time", "phase", "alpha", "left_force_ratio", "right_force_ratio", "roll", "pitch", "yaw", "qvel_norm", "normal_force", "left_contacts", "right_contacts", "max_abs_tau_so_far")},
        "csv": str(csv_path),
        "geom_csv": str(geom_csv_path),
    }
    (out_dir / "sequence_torque_contact_summary.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(json.dumps(summary, indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
