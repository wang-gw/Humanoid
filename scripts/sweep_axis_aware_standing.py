from __future__ import annotations

import argparse
import csv
import itertools
import json
from pathlib import Path

import mujoco
import numpy as np

from render_axis_aware_standing import PITCH_ROLE_ACTUATORS, ROLL_ROLE_ACTUATORS
from render_pd_standing import hinge_joint_info, load_targets, quat_to_roll_pitch_yaw, total_contact_force
from sweep_stabilized_standing import initialize, support_center, total_com


def simulate(
    model: mujoco.MjModel,
    joints: list[dict[str, int | str]],
    actuator_names: list[str],
    base_z: float,
    target_q: np.ndarray,
    cfg: dict[str, float],
    duration: float,
    log_every: int,
    keep_rows: bool,
) -> tuple[dict[str, float], list[dict[str, float]]]:
    data = mujoco.MjData(model)
    initialize(model, data, joints, base_z, target_q)
    steps = int(duration / model.opt.timestep)
    rows: list[dict[str, float]] = []
    max_qvel = 0.0
    max_contact = 0.0
    max_abs_roll = 0.0
    max_abs_pitch = 0.0
    max_abs_tau = 0.0
    contact_samples = 0
    sat_samples = 0
    samples = 0

    for step in range(steps + 1):
        q = np.array([data.qpos[int(joint["qposadr"])] for joint in joints], dtype=np.float64)
        qd = np.array([data.qvel[int(joint["dofadr"])] for joint in joints], dtype=np.float64)
        roll, pitch, yaw = quat_to_roll_pitch_yaw(data.qpos[3:7])
        com = total_com(model, data)
        center, half = support_center(model, data)
        com_err = (com[:2] - center) / np.maximum(half, 1e-6)
        wx = float(data.qvel[3]) if model.nv >= 6 else 0.0
        wy = float(data.qvel[4]) if model.nv >= 6 else 0.0

        tau = cfg["joint_kp"] * (target_q - q) - cfg["joint_kd"] * qd
        roll_term = cfg["kp_att"] * roll + cfg["kd_att"] * wx + cfg["kcom"] * float(com_err[1])
        pitch_term = cfg["kp_att"] * pitch + cfg["kd_att"] * wy + cfg["kcom"] * float(com_err[0])
        for idx, name in enumerate(actuator_names):
            if name in ROLL_ROLE_ACTUATORS:
                tau[idx] += cfg["roll_sign"] * roll_term
            if name in PITCH_ROLE_ACTUATORS:
                tau[idx] += cfg["pitch_sign"] * pitch_term
        tau = np.clip(tau, -cfg["torque_limit"], cfg["torque_limit"])
        data.ctrl[:] = tau

        if step % max(log_every, 1) == 0 or step == steps:
            qvel = float(np.linalg.norm(data.qvel))
            contact_force = total_contact_force(model, data)
            max_qvel = max(max_qvel, qvel)
            max_contact = max(max_contact, contact_force)
            max_abs_roll = max(max_abs_roll, abs(roll))
            max_abs_pitch = max(max_abs_pitch, abs(pitch))
            max_abs_tau = max(max_abs_tau, float(np.max(np.abs(data.actuator_force))) if model.nu else 0.0)
            contact_samples += int(data.ncon > 0)
            sat_samples += int(np.any(np.abs(tau) >= cfg["torque_limit"] * 0.999))
            samples += 1
            if keep_rows:
                rows.append(
                    {
                        "step": float(step),
                        "time": float(data.time),
                        "base_z": float(data.qpos[2]),
                        "base_roll": float(roll),
                        "base_pitch": float(pitch),
                        "base_yaw": float(yaw),
                        "contacts": float(data.ncon),
                        "contact_normal_force": contact_force,
                        "qvel_norm": qvel,
                        "max_abs_ctrl": float(np.max(np.abs(tau))) if model.nu else 0.0,
                    }
                )

        if step < steps:
            mujoco.mj_step(model, data)

    final_roll, final_pitch, final_yaw = quat_to_roll_pitch_yaw(data.qpos[3:7])
    contact_fraction = contact_samples / max(samples, 1)
    saturation_fraction = sat_samples / max(samples, 1)
    score = (
        6.0 * abs(final_roll)
        + 6.0 * abs(final_pitch)
        + 2.0 * max_abs_roll
        + 2.0 * max_abs_pitch
        + 0.25 * max_qvel
        + 0.002 * max_contact
        + 2.0 * max(0.0, 0.1 - contact_fraction)
        + 3.0 * saturation_fraction
    )
    return (
        {
            **cfg,
            "score": float(score),
            "final_base_z": float(data.qpos[2]),
            "final_roll_rad": float(final_roll),
            "final_pitch_rad": float(final_pitch),
            "final_yaw_rad": float(final_yaw),
            "max_abs_roll_rad": float(max_abs_roll),
            "max_abs_pitch_rad": float(max_abs_pitch),
            "max_qvel_norm": float(max_qvel),
            "max_contact_normal_force": float(max_contact),
            "max_abs_tau": float(max_abs_tau),
            "contact_fraction": float(contact_fraction),
            "saturation_fraction": float(saturation_fraction),
        },
        rows,
    )


def parse_floats(value: str) -> list[float]:
    return [float(part.strip()) for part in value.split(",") if part.strip()]


def main() -> int:
    parser = argparse.ArgumentParser(description="Sweep axis-aware standing stabilizer gains.")
    parser.add_argument("--model", type=Path, required=True)
    parser.add_argument("--pose-json", type=Path, required=True)
    parser.add_argument("--duration", type=float, default=2.0)
    parser.add_argument("--joint-kp", type=float, default=20.0)
    parser.add_argument("--joint-kd", type=float, default=12.0)
    parser.add_argument("--torque-limit", type=float, default=30.0)
    parser.add_argument("--kp-att", default="0,0.5,1,2,4")
    parser.add_argument("--kd-att", default="0,0.5,1,2,4")
    parser.add_argument("--kcom", default="0,0.5,1,2")
    parser.add_argument("--log-every", type=int, default=5)
    parser.add_argument("--out-dir", type=Path, default=Path("outputs/analysis/axis_aware_standing_sweep"))
    args = parser.parse_args()

    out_dir = args.out_dir.resolve()
    out_dir.mkdir(parents=True, exist_ok=True)

    model = mujoco.MjModel.from_xml_path(str(args.model.resolve()))
    joints = hinge_joint_info(model)
    actuator_names = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, idx) or f"actuator_{idx}" for idx in range(model.nu)]
    base_z, target_q = load_targets(model, joints, args.pose_json.resolve())

    summaries = []
    for kp_att, kd_att, kcom, roll_sign, pitch_sign in itertools.product(
        parse_floats(args.kp_att),
        parse_floats(args.kd_att),
        parse_floats(args.kcom),
        (-1.0, 1.0),
        (-1.0, 1.0),
    ):
        cfg = {
            "joint_kp": args.joint_kp,
            "joint_kd": args.joint_kd,
            "torque_limit": args.torque_limit,
            "kp_att": kp_att,
            "kd_att": kd_att,
            "kcom": kcom,
            "roll_sign": roll_sign,
            "pitch_sign": pitch_sign,
        }
        summary, _ = simulate(model, joints, actuator_names, base_z, target_q, cfg, args.duration, args.log_every, False)
        summaries.append(summary)

    summaries.sort(key=lambda row: row["score"])
    best = summaries[0]
    best_summary, best_rows = simulate(model, joints, actuator_names, base_z, target_q, best, args.duration, args.log_every, True)

    summary_csv = out_dir / "axis_aware_sweep_summary.csv"
    with summary_csv.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(summaries[0].keys()))
        writer.writeheader()
        writer.writerows(summaries)
    best_csv = out_dir / "best_axis_aware_timeseries.csv"
    with best_csv.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(best_rows[0].keys()))
        writer.writeheader()
        writer.writerows(best_rows)

    payload = {
        "model": str(args.model.resolve()),
        "pose_json": str(args.pose_json.resolve()),
        "duration": args.duration,
        "case_count": len(summaries),
        "best": best_summary,
        "top10": summaries[:10],
        "summary_csv": str(summary_csv),
        "best_csv": str(best_csv),
        "roll_role_actuators": list(ROLL_ROLE_ACTUATORS),
        "pitch_role_actuators": list(PITCH_ROLE_ACTUATORS),
    }
    json_path = out_dir / "axis_aware_sweep_summary.json"
    json_path.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(json.dumps(payload, indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
