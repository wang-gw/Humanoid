from __future__ import annotations

import argparse
import csv
import itertools
import json
from pathlib import Path

import mujoco
import numpy as np

from render_pd_standing import hinge_joint_info, load_targets, quat_to_roll_pitch_yaw, total_contact_force
from sweep_stabilized_standing import initialize


def simulate(
    model: mujoco.MjModel,
    joints: list[dict[str, int | str]],
    base_z: float,
    target_q: np.ndarray,
    kp: float,
    kd: float,
    torque_limit: float,
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
        tau = kp * (target_q - q) - kd * qd
        tau = np.clip(tau, -torque_limit, torque_limit)
        data.ctrl[:] = tau

        if step % max(log_every, 1) == 0 or step == steps:
            roll, pitch, yaw = quat_to_roll_pitch_yaw(data.qpos[3:7])
            qvel = float(np.linalg.norm(data.qvel))
            contact_force = total_contact_force(model, data)
            max_qvel = max(max_qvel, qvel)
            max_contact = max(max_contact, contact_force)
            max_abs_roll = max(max_abs_roll, abs(roll))
            max_abs_pitch = max(max_abs_pitch, abs(pitch))
            max_abs_tau = max(max_abs_tau, float(np.max(np.abs(data.actuator_force))) if model.nu else 0.0)
            contact_samples += int(data.ncon > 0)
            sat_samples += int(np.any(np.abs(tau) >= torque_limit * 0.999))
            samples += 1
            row = {
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
            if keep_rows:
                rows.append(row)

        if step < steps:
            mujoco.mj_step(model, data)

    final_roll, final_pitch, final_yaw = quat_to_roll_pitch_yaw(data.qpos[3:7])
    contact_fraction = contact_samples / max(samples, 1)
    saturation_fraction = sat_samples / max(samples, 1)
    score = (
        5.0 * abs(final_roll)
        + 5.0 * abs(final_pitch)
        + 2.0 * max_abs_roll
        + 2.0 * max_abs_pitch
        + 0.2 * max_qvel
        + 0.002 * max_contact
        + 2.0 * max(0.0, 0.1 - contact_fraction)
        + 3.0 * saturation_fraction
        + 2.0 * max(0.0, -float(data.qpos[2]))
    )
    summary = {
        "kp": float(kp),
        "kd": float(kd),
        "torque_limit": float(torque_limit),
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
    }
    return summary, rows


def parse_floats(value: str) -> list[float]:
    return [float(part.strip()) for part in value.split(",") if part.strip()]


def main() -> int:
    parser = argparse.ArgumentParser(description="Sweep joint-space PD gains for standing.")
    parser.add_argument("--model", type=Path, required=True)
    parser.add_argument("--pose-json", type=Path, required=True)
    parser.add_argument("--duration", type=float, default=2.0)
    parser.add_argument("--kps", default="5,10,20,40,60,80")
    parser.add_argument("--kds", default="0.5,1,2,4,8,12")
    parser.add_argument("--torque-limits", default="30,60,100")
    parser.add_argument("--log-every", type=int, default=5)
    parser.add_argument("--out-dir", type=Path, default=Path("outputs/analysis/pd_gain_sweep"))
    args = parser.parse_args()

    out_dir = args.out_dir.resolve()
    out_dir.mkdir(parents=True, exist_ok=True)

    model = mujoco.MjModel.from_xml_path(str(args.model.resolve()))
    joints = hinge_joint_info(model)
    base_z, target_q = load_targets(model, joints, args.pose_json.resolve())
    summaries = []
    best_rows: list[dict[str, float]] = []
    best_summary: dict[str, float] | None = None

    for kp, kd, limit in itertools.product(parse_floats(args.kps), parse_floats(args.kds), parse_floats(args.torque_limits)):
        summary, _ = simulate(model, joints, base_z, target_q, kp, kd, limit, args.duration, args.log_every, False)
        summaries.append(summary)
        if best_summary is None or summary["score"] < best_summary["score"]:
            best_summary = summary

    if best_summary is None:
        raise RuntimeError("No sweep cases executed.")
    best_summary, best_rows = simulate(
        model,
        joints,
        base_z,
        target_q,
        best_summary["kp"],
        best_summary["kd"],
        best_summary["torque_limit"],
        args.duration,
        args.log_every,
        True,
    )

    summaries.sort(key=lambda row: row["score"])
    summary_csv = out_dir / "pd_gain_sweep_summary.csv"
    with summary_csv.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(summaries[0].keys()))
        writer.writeheader()
        writer.writerows(summaries)

    best_csv = out_dir / "best_pd_timeseries.csv"
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
    }
    json_path = out_dir / "pd_gain_sweep_summary.json"
    json_path.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(json.dumps(payload, indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
