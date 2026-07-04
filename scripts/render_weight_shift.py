from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import imageio.v2 as imageio
import mujoco
import numpy as np

from render_axis_aware_standing import PITCH_ROLE_ACTUATORS, ROLL_ROLE_ACTUATORS
from render_pd_standing import (
    hinge_joint_info,
    load_targets,
    make_camera,
    overlay_text,
    quat_to_roll_pitch_yaw,
    write_video,
)
from sweep_stabilized_standing import initialize, support_center, total_com


FOOT_GEOMS = {
    "left": "foot_L_1_sole_collision",
    "right": "foot_R_v1_1_sole_collision",
}


def foot_geom_ids(model: mujoco.MjModel) -> dict[int, str]:
    output: dict[int, str] = {}
    for geom_id in range(model.ngeom):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, geom_id) or ""
        if name == FOOT_GEOMS["left"] or name.startswith("foot_L_1_sole_pad_"):
            output[geom_id] = "left"
        if name == FOOT_GEOMS["right"] or name.startswith("foot_R_v1_1_sole_pad_"):
            output[geom_id] = "right"
    return output


def contact_stats(model: mujoco.MjModel, data: mujoco.MjData, geom_to_side: dict[int, str]) -> dict[str, float]:
    stats = {
        "normal_force": 0.0,
        "left_force": 0.0,
        "right_force": 0.0,
        "left_contacts": 0.0,
        "right_contacts": 0.0,
    }
    force = np.zeros(6, dtype=np.float64)
    for contact_id in range(data.ncon):
        contact = data.contact[contact_id]
        side = geom_to_side.get(int(contact.geom1)) or geom_to_side.get(int(contact.geom2))
        if side is None:
            continue
        mujoco.mj_contactForce(model, data, contact_id, force)
        normal_force = max(float(force[0]), 0.0)
        stats["normal_force"] += normal_force
        stats[f"{side}_force"] += normal_force
        stats[f"{side}_contacts"] += 1.0
    total = max(stats["left_force"] + stats["right_force"], 1e-9)
    stats["left_force_ratio"] = stats["left_force"] / total
    stats["right_force_ratio"] = stats["right_force"] / total
    return stats


def target_lateral(time: float, amplitude: float, period: float, ramp: float) -> float:
    scale = min(max(time / max(ramp, 1e-9), 0.0), 1.0)
    return scale * amplitude * math.sin(2.0 * math.pi * time / period)


def main() -> int:
    parser = argparse.ArgumentParser(description="Render lateral weight-shift using axis-aware COM stabilizer.")
    parser.add_argument("--model", type=Path, required=True)
    parser.add_argument("--pose-json", type=Path, required=True)
    parser.add_argument("--out-dir", type=Path, default=Path("outputs/analysis/render_weight_shift"))
    parser.add_argument("--duration", type=float, default=10.0)
    parser.add_argument("--joint-kp", type=float, default=20.0)
    parser.add_argument("--joint-kd", type=float, default=12.0)
    parser.add_argument("--torque-limit", type=float, default=30.0)
    parser.add_argument("--kp-att", type=float, default=0.0)
    parser.add_argument("--kd-att", type=float, default=1.0)
    parser.add_argument("--kcom", type=float, default=1.0)
    parser.add_argument("--roll-sign", type=float, default=-1.0)
    parser.add_argument("--pitch-sign", type=float, default=1.0)
    parser.add_argument("--lateral-amplitude", type=float, default=0.18)
    parser.add_argument("--period", type=float, default=6.0)
    parser.add_argument("--ramp", type=float, default=1.0)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--azimuth", type=float, default=135.0)
    parser.add_argument("--elevation", type=float, default=-12.0)
    parser.add_argument("--distance", type=float, default=1.45)
    args = parser.parse_args()

    out_dir = args.out_dir.resolve()
    out_dir.mkdir(parents=True, exist_ok=True)

    model = mujoco.MjModel.from_xml_path(str(args.model.resolve()))
    data = mujoco.MjData(model)
    joints = hinge_joint_info(model)
    actuator_names = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, idx) or f"actuator_{idx}" for idx in range(model.nu)]
    base_z, target_q = load_targets(model, joints, args.pose_json.resolve())
    initialize(model, data, joints, base_z, target_q)
    geom_to_side = foot_geom_ids(model)

    renderer = mujoco.Renderer(model, height=args.height, width=args.width)
    steps = int(args.duration / model.opt.timestep)
    render_every = max(1, int((1.0 / args.fps) / model.opt.timestep))
    frames: list[np.ndarray] = []
    timeline: list[dict[str, float]] = []
    max_abs_tau = 0.0
    max_qvel = 0.0
    max_contact = 0.0
    min_left_ratio = 1.0
    max_left_ratio = 0.0
    contact_samples = 0
    render_samples = 0
    sat_samples = 0
    samples = 0

    for step in range(steps + 1):
        q = np.array([data.qpos[int(joint["qposadr"])] for joint in joints], dtype=np.float64)
        qd = np.array([data.qvel[int(joint["dofadr"])] for joint in joints], dtype=np.float64)
        roll, pitch, yaw = quat_to_roll_pitch_yaw(data.qpos[3:7])
        com = total_com(model, data)
        center, half = support_center(model, data)
        com_err = (com[:2] - center) / np.maximum(half, 1e-6)
        lateral_target = target_lateral(float(data.time), args.lateral_amplitude, args.period, args.ramp)
        wx = float(data.qvel[3]) if model.nv >= 6 else 0.0
        wy = float(data.qvel[4]) if model.nv >= 6 else 0.0

        roll_term = args.kp_att * roll + args.kd_att * wx + args.kcom * (float(com_err[1]) - lateral_target)
        pitch_term = args.kp_att * pitch + args.kd_att * wy + args.kcom * float(com_err[0])

        tau = args.joint_kp * (target_q - q) - args.joint_kd * qd
        for idx, name in enumerate(actuator_names):
            if name in ROLL_ROLE_ACTUATORS:
                tau[idx] += args.roll_sign * roll_term
            if name in PITCH_ROLE_ACTUATORS:
                tau[idx] += args.pitch_sign * pitch_term
        tau = np.clip(tau, -args.torque_limit, args.torque_limit)
        data.ctrl[:] = tau

        stats = contact_stats(model, data, geom_to_side)
        qvel = float(np.linalg.norm(data.qvel))
        max_abs_tau = max(max_abs_tau, float(np.max(np.abs(data.actuator_force))) if model.nu else 0.0)
        max_qvel = max(max_qvel, qvel)
        max_contact = max(max_contact, stats["normal_force"])
        min_left_ratio = min(min_left_ratio, stats["left_force_ratio"])
        max_left_ratio = max(max_left_ratio, stats["left_force_ratio"])
        sat_samples += int(np.any(np.abs(tau) >= args.torque_limit * 0.999))
        samples += 1

        if step % render_every == 0 or step == steps:
            render_samples += 1
            contact_samples += int(data.ncon > 0)
            camera = make_camera(data, args.azimuth, args.elevation, args.distance)
            renderer.update_scene(data, camera=camera)
            frame = renderer.render()
            rows = [
                f"weight shift  t={data.time:5.3f}s  contacts={data.ncon}",
                f"targetY={lateral_target:+.2f}  comYerr={com_err[1]:+.2f}  L={stats['left_force_ratio']:.2f}",
                f"roll={roll:+.2f}  pitch={pitch:+.2f}  |qvel|={qvel:4.2f}",
                f"F={stats['normal_force']:6.1f}N  max|tau|={max_abs_tau:4.1f}Nm",
            ]
            frames.append(overlay_text(frame, rows))
            timeline.append(
                {
                    "time": float(data.time),
                    "target_lateral": float(lateral_target),
                    "com_err_x": float(com_err[0]),
                    "com_err_y": float(com_err[1]),
                    "base_z": float(data.qpos[2]),
                    "roll": float(roll),
                    "pitch": float(pitch),
                    "yaw": float(yaw),
                    "qvel_norm": qvel,
                    "contacts": float(data.ncon),
                    **{key: float(value) for key, value in stats.items()},
                    "max_abs_tau_so_far": float(max_abs_tau),
                }
            )

        if step < steps:
            mujoco.mj_step(model, data)

    renderer.close()
    mp4_path = out_dir / "weight_shift_render.mp4"
    gif_path = out_dir / "weight_shift_render.gif"
    write_video(frames, mp4_path, gif_path, args.fps)
    if frames:
        imageio.imwrite(out_dir / "first_frame.png", frames[0])
        imageio.imwrite(out_dir / "mid_frame.png", frames[len(frames) // 2])
        imageio.imwrite(out_dir / "last_frame.png", frames[-1])

    import csv

    csv_path = out_dir / "weight_shift_timeline.csv"
    if timeline:
        with csv_path.open("w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=list(timeline[0].keys()))
            writer.writeheader()
            writer.writerows(timeline)

    summary = {
        "model": str(args.model.resolve()),
        "pose_json": str(args.pose_json.resolve()),
        "duration": args.duration,
        "controller": {
            "joint_kp": args.joint_kp,
            "joint_kd": args.joint_kd,
            "torque_limit": args.torque_limit,
            "kp_att": args.kp_att,
            "kd_att": args.kd_att,
            "kcom": args.kcom,
            "roll_sign": args.roll_sign,
            "pitch_sign": args.pitch_sign,
            "lateral_amplitude": args.lateral_amplitude,
            "period": args.period,
            "ramp": args.ramp,
        },
        "frames": len(frames),
        "contact_frame_fraction": contact_samples / max(render_samples, 1),
        "saturation_fraction": sat_samples / max(samples, 1),
        "max_qvel_norm": max_qvel,
        "max_contact_force": max_contact,
        "max_abs_tau": max_abs_tau,
        "left_force_ratio_min": min_left_ratio,
        "left_force_ratio_max": max_left_ratio,
        "left_force_ratio_range": max_left_ratio - min_left_ratio,
        "final": timeline[-1] if timeline else {},
        "mp4": str(mp4_path),
        "gif": str(gif_path),
        "csv": str(csv_path),
        "first_frame": str(out_dir / "first_frame.png"),
        "mid_frame": str(out_dir / "mid_frame.png"),
        "last_frame": str(out_dir / "last_frame.png"),
    }
    (out_dir / "weight_shift_summary.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(json.dumps(summary, indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
