from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import imageio.v2 as imageio
import mujoco
import numpy as np

from render_axis_aware_standing import PITCH_ROLE_ACTUATORS, ROLL_ROLE_ACTUATORS
from render_pd_standing import hinge_joint_info, load_targets, make_camera, overlay_text, quat_to_roll_pitch_yaw, write_video
from render_weight_shift import contact_stats, foot_geom_ids
from sweep_stabilized_standing import initialize, support_center, total_com


def smoothstep(value: float) -> float:
    x = min(max(value, 0.0), 1.0)
    return x * x * (3.0 - 2.0 * x)


def main() -> int:
    parser = argparse.ArgumentParser(description="Render a joint target transition between two poses.")
    parser.add_argument("--model", type=Path, required=True)
    parser.add_argument("--start-pose", type=Path, required=True)
    parser.add_argument("--end-pose", type=Path, required=True)
    parser.add_argument("--out-dir", type=Path, required=True)
    parser.add_argument("--duration", type=float, default=8.0)
    parser.add_argument("--ramp", type=float, default=4.0)
    parser.add_argument("--joint-kp", type=float, default=20.0)
    parser.add_argument("--joint-kd", type=float, default=12.0)
    parser.add_argument("--torque-limit", type=float, default=30.0)
    parser.add_argument("--kp-att", type=float, default=0.0)
    parser.add_argument("--kd-att", type=float, default=1.0)
    parser.add_argument("--kcom", type=float, default=1.0)
    parser.add_argument("--roll-sign", type=float, default=-1.0)
    parser.add_argument("--pitch-sign", type=float, default=1.0)
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
    start_base_z, start_q = load_targets(model, joints, args.start_pose.resolve())
    end_base_z, end_q = load_targets(model, joints, args.end_pose.resolve())
    initialize(model, data, joints, start_base_z, start_q)
    geom_to_side = foot_geom_ids(model)

    renderer = mujoco.Renderer(model, height=args.height, width=args.width)
    steps = int(args.duration / model.opt.timestep)
    render_every = max(1, int((1.0 / args.fps) / model.opt.timestep))
    frames: list[np.ndarray] = []
    timeline: list[dict[str, float]] = []
    max_abs_tau = 0.0
    max_qvel = 0.0
    max_contact = 0.0
    sat_samples = 0
    contact_samples = 0
    render_samples = 0
    min_left_ratio = 1.0
    max_left_ratio = 0.0

    for step in range(steps + 1):
        alpha = smoothstep(float(data.time) / max(args.ramp, 1e-9))
        target_q = (1.0 - alpha) * start_q + alpha * end_q
        target_base_z = (1.0 - alpha) * start_base_z + alpha * end_base_z
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

        stats = contact_stats(model, data, geom_to_side)
        qvel = float(np.linalg.norm(data.qvel))
        max_abs_tau = max(max_abs_tau, float(np.max(np.abs(data.actuator_force))) if model.nu else 0.0)
        max_qvel = max(max_qvel, qvel)
        max_contact = max(max_contact, stats["normal_force"])
        min_left_ratio = min(min_left_ratio, stats["left_force_ratio"])
        max_left_ratio = max(max_left_ratio, stats["left_force_ratio"])
        sat_samples += int(np.any(np.abs(tau) >= args.torque_limit * 0.999))

        if step % render_every == 0 or step == steps:
            render_samples += 1
            contact_samples += int(data.ncon > 0)
            camera = make_camera(data, args.azimuth, args.elevation, args.distance)
            renderer.update_scene(data, camera=camera)
            frame = renderer.render()
            rows = [
                f"pose transition  t={data.time:5.3f}s  alpha={alpha:.2f}",
                f"L={stats['left_force_ratio']:.2f}  R={stats['right_force_ratio']:.2f}  contacts={data.ncon}",
                f"roll={roll:+.2f}  pitch={pitch:+.2f}  |qvel|={qvel:4.2f}",
                f"target z={target_base_z:+.3f}  max|tau|={max_abs_tau:4.1f}Nm",
            ]
            frames.append(overlay_text(frame, rows))
            timeline.append(
                {
                    "time": float(data.time),
                    "alpha": float(alpha),
                    "target_base_z": float(target_base_z),
                    "base_z": float(data.qpos[2]),
                    "roll": float(roll),
                    "pitch": float(pitch),
                    "yaw": float(yaw),
                    "qvel_norm": qvel,
                    "com_err_x": float(com_err[0]),
                    "com_err_y": float(com_err[1]),
                    **{key: float(value) for key, value in stats.items()},
                    "max_abs_tau_so_far": float(max_abs_tau),
                }
            )
        if step < steps:
            mujoco.mj_step(model, data)

    renderer.close()
    mp4_path = out_dir / "pose_transition_render.mp4"
    gif_path = out_dir / "pose_transition_render.gif"
    write_video(frames, mp4_path, gif_path, args.fps)
    if frames:
        imageio.imwrite(out_dir / "first_frame.png", frames[0])
        imageio.imwrite(out_dir / "mid_frame.png", frames[len(frames) // 2])
        imageio.imwrite(out_dir / "last_frame.png", frames[-1])

    csv_path = out_dir / "pose_transition_timeline.csv"
    if timeline:
        with csv_path.open("w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=list(timeline[0].keys()))
            writer.writeheader()
            writer.writerows(timeline)

    summary = {
        "model": str(args.model.resolve()),
        "start_pose": str(args.start_pose.resolve()),
        "end_pose": str(args.end_pose.resolve()),
        "duration": args.duration,
        "ramp": args.ramp,
        "controller": {
            "joint_kp": args.joint_kp,
            "joint_kd": args.joint_kd,
            "torque_limit": args.torque_limit,
            "kp_att": args.kp_att,
            "kd_att": args.kd_att,
            "kcom": args.kcom,
            "roll_sign": args.roll_sign,
            "pitch_sign": args.pitch_sign,
        },
        "contact_frame_fraction": contact_samples / max(render_samples, 1),
        "saturation_fraction": sat_samples / max(steps + 1, 1),
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
    }
    (out_dir / "pose_transition_summary.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(json.dumps(summary, indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
