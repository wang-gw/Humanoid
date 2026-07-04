from __future__ import annotations

import argparse
import json
from pathlib import Path

import cv2
import imageio.v2 as imageio
import mujoco
import numpy as np

from render_pd_standing import (
    hinge_joint_info,
    load_targets,
    make_camera,
    overlay_text,
    quat_to_roll_pitch_yaw,
    total_contact_force,
    write_video,
)
from sweep_stabilized_standing import initialize, support_center, total_com


ROLL_ROLE_ACTUATORS = (
    "motor_left_hip_pitch",
    "motor_right_hip_pitch",
    "motor_left_ankle_pitch",
    "motor_right_ankle_pitch",
)
PITCH_ROLE_ACTUATORS = (
    "motor_left_hip_roll",
    "motor_right_hip_roll",
    "motor_left_ankle_roll",
    "motor_right_ankle_roll",
)


def main() -> int:
    parser = argparse.ArgumentParser(description="Render standing with axis-aware roll/pitch stabilizer mapping.")
    parser.add_argument("--model", type=Path, required=True)
    parser.add_argument("--pose-json", type=Path, required=True)
    parser.add_argument("--out-dir", type=Path, default=Path("outputs/analysis/render_axis_aware_standing"))
    parser.add_argument("--duration", type=float, default=2.0)
    parser.add_argument("--joint-kp", type=float, default=60.0)
    parser.add_argument("--joint-kd", type=float, default=4.0)
    parser.add_argument("--torque-limit", type=float, default=100.0)
    parser.add_argument("--kp-att", type=float, default=4.0)
    parser.add_argument("--kd-att", type=float, default=4.0)
    parser.add_argument("--kcom", type=float, default=0.0)
    parser.add_argument("--roll-sign", type=float, default=-1.0)
    parser.add_argument("--pitch-sign", type=float, default=-1.0)
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

    renderer = mujoco.Renderer(model, height=args.height, width=args.width)
    steps = int(args.duration / model.opt.timestep)
    render_every = max(1, int((1.0 / args.fps) / model.opt.timestep))
    frames: list[np.ndarray] = []
    timeline: list[dict[str, float]] = []
    max_abs_tau = 0.0
    max_qvel = 0.0
    max_contact = 0.0
    contact_samples = 0
    render_samples = 0

    for step in range(steps + 1):
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

        tau = args.joint_kp * (target_q - q) - args.joint_kd * qd
        for idx, name in enumerate(actuator_names):
            if name in ROLL_ROLE_ACTUATORS:
                tau[idx] += args.roll_sign * roll_term
            if name in PITCH_ROLE_ACTUATORS:
                tau[idx] += args.pitch_sign * pitch_term
        tau = np.clip(tau, -args.torque_limit, args.torque_limit)
        data.ctrl[:] = tau

        contact_force = total_contact_force(model, data)
        qvel = float(np.linalg.norm(data.qvel))
        max_abs_tau = max(max_abs_tau, float(np.max(np.abs(data.actuator_force))) if model.nu else 0.0)
        max_qvel = max(max_qvel, qvel)
        max_contact = max(max_contact, contact_force)

        if step % render_every == 0 or step == steps:
            render_samples += 1
            contact_samples += int(data.ncon > 0)
            camera = make_camera(data, args.azimuth, args.elevation, args.distance)
            renderer.update_scene(data, camera=camera)
            frame = renderer.render()
            rows = [
                f"axis-aware  t={data.time:5.3f}s  contacts={data.ncon}",
                f"z={data.qpos[2]:+.3f}m  roll={roll:+.2f}  pitch={pitch:+.2f}",
                f"|qvel|={qvel:6.1f}  contact={contact_force:7.1f}N",
                f"max |tau|={max_abs_tau:6.1f}Nm  role map",
            ]
            frames.append(overlay_text(frame, rows))
            timeline.append(
                {
                    "time": float(data.time),
                    "base_z": float(data.qpos[2]),
                    "roll": float(roll),
                    "pitch": float(pitch),
                    "yaw": float(yaw),
                    "qvel_norm": qvel,
                    "contact_force": float(contact_force),
                    "contacts": float(data.ncon),
                    "max_abs_tau_so_far": float(max_abs_tau),
                }
            )

        if step < steps:
            mujoco.mj_step(model, data)

    renderer.close()
    mp4_path = out_dir / "axis_aware_standing_render.mp4"
    gif_path = out_dir / "axis_aware_standing_render.gif"
    write_video(frames, mp4_path, gif_path, args.fps)
    if frames:
        imageio.imwrite(out_dir / "first_frame.png", frames[0])
        imageio.imwrite(out_dir / "mid_frame.png", frames[len(frames) // 2])
        imageio.imwrite(out_dir / "last_frame.png", frames[-1])

    summary = {
        "model": str(args.model.resolve()),
        "pose_json": str(args.pose_json.resolve()),
        "duration": args.duration,
        "joint_kp": args.joint_kp,
        "joint_kd": args.joint_kd,
        "torque_limit": args.torque_limit,
        "axis_aware_config": {
            "kp_att": args.kp_att,
            "kd_att": args.kd_att,
            "kcom": args.kcom,
            "roll_sign": args.roll_sign,
            "pitch_sign": args.pitch_sign,
            "roll_role_actuators": list(ROLL_ROLE_ACTUATORS),
            "pitch_role_actuators": list(PITCH_ROLE_ACTUATORS),
        },
        "frames": len(frames),
        "contact_frame_fraction": contact_samples / max(render_samples, 1),
        "max_qvel_norm": max_qvel,
        "max_contact_force": max_contact,
        "max_abs_tau": max_abs_tau,
        "final": timeline[-1] if timeline else {},
        "mp4": str(mp4_path),
        "gif": str(gif_path),
        "first_frame": str(out_dir / "first_frame.png"),
        "mid_frame": str(out_dir / "mid_frame.png"),
        "last_frame": str(out_dir / "last_frame.png"),
    }
    (out_dir / "render_summary.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(json.dumps(summary, indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
