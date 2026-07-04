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
from render_right_foot_lift_sequence import right_foot_clearance
from render_weight_shift import contact_stats, foot_geom_ids
from sweep_stabilized_standing import initialize, support_center, total_com


TASK_JOINT_LIMITS = {
    "right_hip_roll": (-0.16, 0.16),
    "right_hip_pitch": (-0.18, 0.12),
    "right_knee_pitch": (-0.26, 0.08),
    "right_ankle_pitch": (-0.10, 0.28),
    "right_ankle_roll": (-0.16, 0.16),
}


def clearance_for_qpos(model: mujoco.MjModel, qpos: np.ndarray) -> float:
    data = mujoco.MjData(model)
    data.qpos[:] = qpos
    data.qvel[:] = 0.0
    mujoco.mj_forward(model, data)
    return right_foot_clearance(model, data)


def finite_difference_clearance_jacobian(
    model: mujoco.MjModel,
    base_qpos: np.ndarray,
    task_qpos_indices: list[int],
    step: float,
) -> np.ndarray:
    current = clearance_for_qpos(model, base_qpos)
    jac = np.zeros(len(task_qpos_indices), dtype=np.float64)
    for idx, qpos_idx in enumerate(task_qpos_indices):
        qpos_plus = base_qpos.copy()
        qpos_plus[qpos_idx] += step
        jac[idx] = (clearance_for_qpos(model, qpos_plus) - current) / step
    return jac


def main() -> int:
    parser = argparse.ArgumentParser(description="Render task-space right-foot clearance feedback controller.")
    parser.add_argument("--model", type=Path, required=True)
    parser.add_argument("--start-pose", type=Path, required=True)
    parser.add_argument("--out-dir", type=Path, required=True)
    parser.add_argument("--duration", type=float, default=8.0)
    parser.add_argument("--lift-start", type=float, default=1.0)
    parser.add_argument("--lift-end", type=float, default=4.0)
    parser.add_argument("--return-end", type=float, default=6.0)
    parser.add_argument("--target-clearance", type=float, default=0.002)
    parser.add_argument("--task-gain", type=float, default=0.9)
    parser.add_argument("--task-decay", type=float, default=1.6)
    parser.add_argument("--jac-step", type=float, default=1e-4)
    parser.add_argument("--damping", type=float, default=1e-4)
    parser.add_argument("--max-dq-step", type=float, default=0.012)
    parser.add_argument("--roll-soft-limit", type=float, default=0.10)
    parser.add_argument("--roll-hard-limit", type=float, default=0.14)
    parser.add_argument("--target-left-ratio", type=float, default=0.94)
    parser.add_argument("--joint-kp", type=float, default=20.0)
    parser.add_argument("--joint-kd", type=float, default=12.0)
    parser.add_argument("--torque-limit", type=float, default=30.0)
    parser.add_argument("--kp-att", type=float, default=0.0)
    parser.add_argument("--kd-att", type=float, default=1.0)
    parser.add_argument("--kcom", type=float, default=1.0)
    parser.add_argument("--roll-sign", type=float, default=-1.0)
    parser.add_argument("--pitch-sign", type=float, default=1.0)
    parser.add_argument("--kforce", type=float, default=12.0)
    parser.add_argument("--force-sign", type=float, default=1.0)
    parser.add_argument("--force-role", choices=("roll", "pitch", "both"), default="both")
    parser.add_argument("--force-on", choices=("all", "hip", "ankle"), default="all")
    parser.add_argument("--force-side-mode", choices=("same", "opposite"), default="opposite")
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
    joint_names = [str(joint["name"]) for joint in joints]
    actuator_names = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, idx) or f"actuator_{idx}" for idx in range(model.nu)]
    base_z, start_q = load_targets(model, joints, args.start_pose.resolve())
    initialize(model, data, joints, base_z, start_q)
    geom_to_side = foot_geom_ids(model)

    task_joint_names = [name for name in TASK_JOINT_LIMITS if name in joint_names]
    task_joint_indices = [joint_names.index(name) for name in task_joint_names]
    task_qpos_indices = [int(joints[idx]["qposadr"]) for idx in task_joint_indices]
    task_offsets = np.zeros(len(task_joint_indices), dtype=np.float64)
    lower = np.array([TASK_JOINT_LIMITS[name][0] for name in task_joint_names], dtype=np.float64)
    upper = np.array([TASK_JOINT_LIMITS[name][1] for name in task_joint_names], dtype=np.float64)

    renderer = mujoco.Renderer(model, height=args.height, width=args.width)
    steps = int(args.duration / model.opt.timestep)
    render_every = max(1, int((1.0 / args.fps) / model.opt.timestep))
    dt = float(model.opt.timestep)
    frames: list[np.ndarray] = []
    timeline: list[dict[str, float]] = []
    max_abs_tau = 0.0
    max_qvel = 0.0
    max_contact = 0.0
    max_abs_roll = 0.0
    max_abs_pitch = 0.0
    max_clearance = -np.inf
    min_clearance = np.inf
    sat_samples = 0
    render_samples = 0
    contact_samples = 0

    for step_idx in range(steps + 1):
        time = float(data.time)
        q = np.array([data.qpos[int(joint["qposadr"])] for joint in joints], dtype=np.float64)
        qd = np.array([data.qvel[int(joint["dofadr"])] for joint in joints], dtype=np.float64)
        roll, pitch, yaw = quat_to_roll_pitch_yaw(data.qpos[3:7])
        clearance = right_foot_clearance(model, data)

        if args.lift_start <= time <= args.lift_end:
            error = max(0.0, args.target_clearance - clearance)
            if error > 0.0 and abs(float(roll)) <= args.roll_hard_limit:
                jac = finite_difference_clearance_jacobian(model, data.qpos.copy(), task_qpos_indices, args.jac_step)
                denom = float(np.dot(jac, jac) + args.damping)
                dq = args.task_gain * error * jac / denom
                dq = np.clip(dq, -args.max_dq_step, args.max_dq_step)
                task_offsets += dq
            if abs(float(roll)) > args.roll_soft_limit:
                scale = (abs(float(roll)) - args.roll_soft_limit) / max(args.roll_hard_limit - args.roll_soft_limit, 1e-6)
                task_offsets -= np.sign(task_offsets) * np.minimum(np.abs(task_offsets), args.task_decay * dt * scale)
        else:
            task_offsets -= np.sign(task_offsets) * np.minimum(np.abs(task_offsets), args.task_decay * dt)
        if abs(float(roll)) > args.roll_hard_limit:
            task_offsets -= np.sign(task_offsets) * np.minimum(np.abs(task_offsets), 2.0 * args.task_decay * dt)
        task_offsets = np.clip(task_offsets, lower, upper)

        target_q = start_q.copy()
        for local_idx, joint_idx in enumerate(task_joint_indices):
            target_q[joint_idx] += task_offsets[local_idx]

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
            role_match = (args.force_role in ("roll", "both") and name in ROLL_ROLE_ACTUATORS) or (
                args.force_role in ("pitch", "both") and name in PITCH_ROLE_ACTUATORS
            )
            group_match = args.force_on == "all" or (args.force_on == "hip" and "hip" in name) or (args.force_on == "ankle" and "ankle" in name)
            if role_match and group_match:
                side_scale = 1.0
                if args.force_side_mode == "opposite":
                    side_scale = 1.0 if "_left_" in name else -1.0
                tau[idx] += side_scale * force_term
        tau = np.clip(tau, -args.torque_limit, args.torque_limit)
        data.ctrl[:] = tau

        min_clearance = min(min_clearance, clearance)
        max_clearance = max(max_clearance, clearance)
        qvel = float(np.linalg.norm(data.qvel))
        max_abs_tau = max(max_abs_tau, float(np.max(np.abs(data.actuator_force))) if model.nu else 0.0)
        max_qvel = max(max_qvel, qvel)
        max_contact = max(max_contact, stats["normal_force"])
        max_abs_roll = max(max_abs_roll, abs(float(roll)))
        max_abs_pitch = max(max_abs_pitch, abs(float(pitch)))
        sat_samples += int(np.any(np.abs(tau) >= args.torque_limit * 0.999))

        if step_idx % render_every == 0 or step_idx == steps:
            render_samples += 1
            contact_samples += int(data.ncon > 0)
            camera = make_camera(data, args.azimuth, args.elevation, args.distance)
            renderer.update_scene(data, camera=camera)
            frame = renderer.render()
            rows = [
                f"task-space lift  t={time:5.3f}s  |off|={np.linalg.norm(task_offsets):.3f}",
                f"clear={clearance*1000:+.1f}mm  L={stats['left_force_ratio']:.2f}/{args.target_left_ratio:.2f}",
                f"Rforce={stats['right_force']:5.1f}N  Rcontacts={stats['right_contacts']:.0f}",
                f"roll={roll:+.2f}  pitch={pitch:+.2f}  max|tau|={max_abs_tau:4.1f}Nm",
            ]
            frames.append(overlay_text(frame, rows))
            timeline.append(
                {
                    "time": time,
                    "task_offset_norm": float(np.linalg.norm(task_offsets)),
                    **{f"offset_{name}": float(task_offsets[idx]) for idx, name in enumerate(task_joint_names)},
                    "right_clearance_m": float(clearance),
                    "target_clearance": args.target_clearance,
                    "target_left_ratio": args.target_left_ratio,
                    "ratio_error": float(ratio_error),
                    "force_term": float(force_term),
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
        if step_idx < steps:
            mujoco.mj_step(model, data)

    renderer.close()
    mp4_path = out_dir / "task_space_lift_render.mp4"
    gif_path = out_dir / "task_space_lift_render.gif"
    write_video(frames, mp4_path, gif_path, args.fps)
    if frames:
        imageio.imwrite(out_dir / "first_frame.png", frames[0])
        imageio.imwrite(out_dir / "mid_frame.png", frames[len(frames) // 2])
        imageio.imwrite(out_dir / "last_frame.png", frames[-1])
    csv_path = out_dir / "task_space_lift_timeline.csv"
    if timeline:
        with csv_path.open("w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=list(timeline[0].keys()))
            writer.writeheader()
            writer.writerows(timeline)
    summary = {
        "model": str(args.model.resolve()),
        "start_pose": str(args.start_pose.resolve()),
        "duration": args.duration,
        "target_clearance": args.target_clearance,
        "target_left_ratio": args.target_left_ratio,
        "task_joint_names": task_joint_names,
        "contact_frame_fraction": contact_samples / max(render_samples, 1),
        "saturation_fraction": sat_samples / max(steps + 1, 1),
        "max_qvel_norm": max_qvel,
        "max_contact_force": max_contact,
        "max_abs_tau": max_abs_tau,
        "max_abs_roll": max_abs_roll,
        "max_abs_pitch": max_abs_pitch,
        "max_right_clearance_m": float(max_clearance),
        "min_right_clearance_m": float(min_clearance),
        "final": timeline[-1] if timeline else {},
        "mp4": str(mp4_path),
        "gif": str(gif_path),
        "csv": str(csv_path),
    }
    (out_dir / "task_space_lift_summary.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(json.dumps(summary, indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
