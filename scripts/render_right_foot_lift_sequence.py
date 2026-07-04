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
from render_pose_sequence import phase_target
from render_weight_shift import contact_stats, foot_geom_ids
from sweep_stabilized_standing import initialize, support_center, total_com


def right_foot_clearance(model: mujoco.MjModel, data: mujoco.MjData) -> float:
    lowest = np.inf
    for geom_id in range(model.ngeom):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, geom_id) or ""
        if not name.startswith("foot_R_v1_1_sole_pad_"):
            continue
        center = np.asarray(data.geom_xpos[geom_id], dtype=np.float64)
        xmat = np.asarray(data.geom_xmat[geom_id], dtype=np.float64).reshape(3, 3)
        half = np.asarray(model.geom_size[geom_id], dtype=np.float64)
        for sx in (-1.0, 1.0):
            for sy in (-1.0, 1.0):
                for sz in (-1.0, 1.0):
                    local = np.array([sx * half[0], sy * half[1], sz * half[2]], dtype=np.float64)
                    lowest = min(lowest, float((center + xmat @ local)[2]))
    return float(lowest)


def main() -> int:
    parser = argparse.ArgumentParser(description="Render a right foot lift pose transition with clearance metrics.")
    parser.add_argument("--model", type=Path, required=True)
    parser.add_argument("--poses", nargs="+", type=Path, required=True)
    parser.add_argument("--out-dir", type=Path, required=True)
    parser.add_argument("--target-left-ratio", type=float, default=0.78)
    parser.add_argument("--ramp", type=float, default=4.0)
    parser.add_argument("--hold", type=float, default=4.0)
    parser.add_argument("--duration", type=float, default=8.0)
    parser.add_argument("--joint-kp", type=float, default=20.0)
    parser.add_argument("--joint-kd", type=float, default=12.0)
    parser.add_argument("--torque-limit", type=float, default=30.0)
    parser.add_argument("--kp-att", type=float, default=0.0)
    parser.add_argument("--kd-att", type=float, default=1.0)
    parser.add_argument("--kcom", type=float, default=1.0)
    parser.add_argument("--roll-sign", type=float, default=-1.0)
    parser.add_argument("--pitch-sign", type=float, default=1.0)
    parser.add_argument("--kforce", type=float, default=5.0)
    parser.add_argument("--force-sign", type=float, default=1.0)
    parser.add_argument("--fps", type=int, default=15)
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
    base_zs = []
    targets = []
    for pose in args.poses:
        base_z, target_q = load_targets(model, joints, pose.resolve())
        base_zs.append(base_z)
        targets.append(target_q)
    initialize(model, data, joints, base_zs[0], targets[0])
    geom_to_side = foot_geom_ids(model)

    renderer = mujoco.Renderer(model, height=args.height, width=args.width)
    steps = int(args.duration / model.opt.timestep)
    render_every = max(1, int((1.0 / args.fps) / model.opt.timestep))
    frames: list[np.ndarray] = []
    timeline: list[dict[str, float]] = []
    max_abs_tau = 0.0
    max_qvel = 0.0
    max_contact = 0.0
    max_abs_roll = 0.0
    max_clearance = -np.inf
    min_clearance = np.inf
    sat_samples = 0
    contact_samples = 0
    render_samples = 0

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
        stats = contact_stats(model, data, geom_to_side)
        ratio_error = args.target_left_ratio - stats["left_force_ratio"]
        force_term = args.force_sign * args.kforce * ratio_error
        tau = args.joint_kp * (target_q - q) - args.joint_kd * qd
        for idx, name in enumerate(actuator_names):
            if name in ROLL_ROLE_ACTUATORS:
                tau[idx] += args.roll_sign * roll_term
            if name in PITCH_ROLE_ACTUATORS:
                tau[idx] += args.pitch_sign * pitch_term
            tau[idx] += force_term
        tau = np.clip(tau, -args.torque_limit, args.torque_limit)
        data.ctrl[:] = tau

        clearance = right_foot_clearance(model, data)
        min_clearance = min(min_clearance, clearance)
        max_clearance = max(max_clearance, clearance)
        qvel = float(np.linalg.norm(data.qvel))
        max_abs_tau = max(max_abs_tau, float(np.max(np.abs(data.actuator_force))) if model.nu else 0.0)
        max_qvel = max(max_qvel, qvel)
        max_contact = max(max_contact, stats["normal_force"])
        max_abs_roll = max(max_abs_roll, abs(float(roll)))
        sat_samples += int(np.any(np.abs(tau) >= args.torque_limit * 0.999))

        if step % render_every == 0 or step == steps:
            render_samples += 1
            contact_samples += int(data.ncon > 0)
            camera = make_camera(data, args.azimuth, args.elevation, args.distance)
            renderer.update_scene(data, camera=camera)
            frame = renderer.render()
            rows = [
                f"right lift  t={data.time:5.3f}s  phase={phase}  alpha={alpha:.2f}",
                f"clear={clearance*1000:+.1f}mm  L={stats['left_force_ratio']:.2f}/{args.target_left_ratio:.2f}",
                f"Rforce={stats['right_force']:5.1f}N  Rcontacts={stats['right_contacts']:.0f}",
                f"roll={roll:+.2f}  pitch={pitch:+.2f}  max|tau|={max_abs_tau:4.1f}Nm",
            ]
            frames.append(overlay_text(frame, rows))
            timeline.append(
                {
                    "time": float(data.time),
                    "phase": float(phase),
                    "alpha": float(alpha),
                    "right_clearance_m": float(clearance),
                    "target_left_ratio": float(args.target_left_ratio),
                    "ratio_error": float(ratio_error),
                    "force_term": float(force_term),
                    "target_base_z": float(target_base_z),
                    "base_z": float(data.qpos[2]),
                    "roll": float(roll),
                    "pitch": float(pitch),
                    "yaw": float(yaw),
                    "qvel_norm": qvel,
                    **{key: float(value) for key, value in stats.items()},
                    "max_abs_tau_so_far": float(max_abs_tau),
                }
            )
        if step < steps:
            mujoco.mj_step(model, data)

    renderer.close()
    mp4_path = out_dir / "right_foot_lift_render.mp4"
    gif_path = out_dir / "right_foot_lift_render.gif"
    write_video(frames, mp4_path, gif_path, args.fps)
    if frames:
        imageio.imwrite(out_dir / "first_frame.png", frames[0])
        imageio.imwrite(out_dir / "mid_frame.png", frames[len(frames) // 2])
        imageio.imwrite(out_dir / "last_frame.png", frames[-1])
    csv_path = out_dir / "right_foot_lift_timeline.csv"
    if timeline:
        with csv_path.open("w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=list(timeline[0].keys()))
            writer.writeheader()
            writer.writerows(timeline)
    summary = {
        "model": str(args.model.resolve()),
        "poses": [str(path.resolve()) for path in args.poses],
        "duration": args.duration,
        "ramp": args.ramp,
        "hold": args.hold,
        "target_left_ratio": args.target_left_ratio,
        "contact_frame_fraction": contact_samples / max(render_samples, 1),
        "saturation_fraction": sat_samples / max(steps + 1, 1),
        "max_qvel_norm": max_qvel,
        "max_contact_force": max_contact,
        "max_abs_tau": max_abs_tau,
        "max_abs_roll": max_abs_roll,
        "max_right_clearance_m": float(max_clearance),
        "min_right_clearance_m": float(min_clearance),
        "final": timeline[-1] if timeline else {},
        "mp4": str(mp4_path),
        "gif": str(gif_path),
        "csv": str(csv_path),
    }
    (out_dir / "right_foot_lift_summary.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(json.dumps(summary, indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
