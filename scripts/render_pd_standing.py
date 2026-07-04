from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import cv2
import imageio.v2 as imageio
import mujoco
import numpy as np


def quat_to_roll_pitch_yaw(q: np.ndarray) -> tuple[float, float, float]:
    w, x, y, z = [float(v) for v in q]
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = 2.0 * (w * y - z * x)
    pitch = math.copysign(math.pi / 2.0, sinp) if abs(sinp) >= 1.0 else math.asin(sinp)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def hinge_joint_info(model: mujoco.MjModel) -> list[dict[str, int | str]]:
    joints = []
    for joint_id in range(model.njnt):
        if model.jnt_type[joint_id] != mujoco.mjtJoint.mjJNT_HINGE:
            continue
        joints.append(
            {
                "id": joint_id,
                "name": mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, joint_id) or f"joint_{joint_id}",
                "qposadr": int(model.jnt_qposadr[joint_id]),
                "dofadr": int(model.jnt_dofadr[joint_id]),
            }
        )
    return joints


def load_targets(model: mujoco.MjModel, joints: list[dict[str, int | str]], pose_path: Path | None) -> tuple[float, np.ndarray]:
    base_z = float(model.qpos0[2]) if model.nq >= 3 else 0.0
    target_q = np.array([float(model.qpos0[int(joint["qposadr"])]) for joint in joints], dtype=np.float64)
    if pose_path is None:
        return base_z, target_q

    payload = json.loads(pose_path.read_text(encoding="utf-8"))
    base_z = float(payload["base_z"])
    targets = payload.get("joint_targets", {})
    for idx, joint in enumerate(joints):
        name = str(joint["name"])
        if name in targets:
            target_q[idx] = float(targets[name])
    return base_z, target_q


def init_state(model: mujoco.MjModel, data: mujoco.MjData, joints: list[dict[str, int | str]], base_z: float, target_q: np.ndarray) -> None:
    data.qpos[:] = model.qpos0
    data.qvel[:] = 0.0
    if model.nq >= 7:
        data.qpos[0:3] = np.array([0.0, 0.0, base_z])
        data.qpos[3:7] = np.array([1.0, 0.0, 0.0, 0.0])
    for value, joint in zip(target_q, joints):
        data.qpos[int(joint["qposadr"])] = float(value)
    mujoco.mj_forward(model, data)


def total_contact_force(model: mujoco.MjModel, data: mujoco.MjData) -> float:
    force = np.zeros(6, dtype=np.float64)
    total = 0.0
    for contact_id in range(data.ncon):
        mujoco.mj_contactForce(model, data, contact_id, force)
        total += max(float(force[0]), 0.0)
    return total


def overlay_text(frame: np.ndarray, rows: list[str]) -> np.ndarray:
    output = frame.copy()
    panel_h = 26 + 24 * len(rows)
    cv2.rectangle(output, (12, 12), (430, panel_h), (20, 20, 20), thickness=-1)
    cv2.rectangle(output, (12, 12), (430, panel_h), (220, 220, 220), thickness=1)
    y = 42
    for row in rows:
        cv2.putText(output, row, (24, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (245, 245, 245), 1, cv2.LINE_AA)
        y += 24
    return output


def make_camera(data: mujoco.MjData, azimuth: float, elevation: float, distance: float) -> mujoco.MjvCamera:
    camera = mujoco.MjvCamera()
    camera.type = mujoco.mjtCamera.mjCAMERA_FREE
    camera.lookat[:] = data.qpos[0:3] if data.qpos.size >= 3 else np.array([0.0, 0.0, 0.2])
    camera.lookat[2] = max(float(camera.lookat[2]), 0.18)
    camera.distance = distance
    camera.azimuth = azimuth
    camera.elevation = elevation
    return camera


def write_video(frames: list[np.ndarray], out_mp4: Path, out_gif: Path, fps: int) -> None:
    if not frames:
        return
    height, width = frames[0].shape[:2]
    writer = cv2.VideoWriter(str(out_mp4), cv2.VideoWriter_fourcc(*"mp4v"), fps, (width, height))
    for frame in frames:
        writer.write(cv2.cvtColor(frame, cv2.COLOR_RGB2BGR))
    writer.release()
    gif_stride = max(1, int(fps / 12))
    imageio.mimsave(out_gif, frames[::gif_stride], duration=gif_stride / fps)


def main() -> int:
    parser = argparse.ArgumentParser(description="Render the full robot during a joint-space PD standing probe.")
    parser.add_argument("--model", type=Path, required=True)
    parser.add_argument("--pose-json", type=Path)
    parser.add_argument("--out-dir", type=Path, default=Path("outputs/analysis/render_pd_standing"))
    parser.add_argument("--duration", type=float, default=2.0)
    parser.add_argument("--kp", type=float, default=60.0)
    parser.add_argument("--kd", type=float, default=4.0)
    parser.add_argument("--torque-limit", type=float, default=100.0)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--azimuth", type=float, default=135.0)
    parser.add_argument("--elevation", type=float, default=-18.0)
    parser.add_argument("--distance", type=float, default=0.95)
    args = parser.parse_args()

    out_dir = args.out_dir.resolve()
    out_dir.mkdir(parents=True, exist_ok=True)

    model = mujoco.MjModel.from_xml_path(str(args.model.resolve()))
    data = mujoco.MjData(model)
    joints = hinge_joint_info(model)
    actuator_names = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, idx) or f"actuator_{idx}" for idx in range(model.nu)]
    if len(joints) != model.nu:
        raise ValueError(f"Expected one actuator per hinge joint, got hinges={len(joints)} nu={model.nu}")

    base_z, target_q = load_targets(model, joints, args.pose_json.resolve() if args.pose_json else None)
    init_state(model, data, joints, base_z, target_q)

    renderer = mujoco.Renderer(model, height=args.height, width=args.width)
    steps = int(args.duration / model.opt.timestep)
    render_every = max(1, int((1.0 / args.fps) / model.opt.timestep))
    frames: list[np.ndarray] = []
    timeline: list[dict[str, float]] = []
    max_abs_tau = 0.0
    max_qvel = 0.0
    max_contact = 0.0

    for step in range(steps + 1):
        q = np.array([data.qpos[int(joint["qposadr"])] for joint in joints], dtype=np.float64)
        qd = np.array([data.qvel[int(joint["dofadr"])] for joint in joints], dtype=np.float64)
        tau = args.kp * (target_q - q) - args.kd * qd
        tau = np.clip(tau, -args.torque_limit, args.torque_limit)
        data.ctrl[:] = tau

        roll, pitch, yaw = quat_to_roll_pitch_yaw(data.qpos[3:7]) if model.nq >= 7 else (0.0, 0.0, 0.0)
        contact_force = total_contact_force(model, data)
        qvel = float(np.linalg.norm(data.qvel))
        max_abs_tau = max(max_abs_tau, float(np.max(np.abs(data.actuator_force))) if model.nu else 0.0)
        max_qvel = max(max_qvel, qvel)
        max_contact = max(max_contact, contact_force)

        if step % render_every == 0 or step == steps:
            camera = make_camera(data, args.azimuth, args.elevation, args.distance)
            renderer.update_scene(data, camera=camera)
            frame = renderer.render()
            rows = [
                f"t={data.time:5.3f}s  contacts={data.ncon}",
                f"base z={data.qpos[2]:+.3f}m  roll={roll:+.2f}  pitch={pitch:+.2f}",
                f"|qvel|={qvel:6.1f}  contact={contact_force:7.1f}N",
                f"max |tau|={max_abs_tau:6.1f}Nm",
            ]
            frames.append(overlay_text(frame, rows))
            timeline.append(
                {
                    "time": float(data.time),
                    "base_z": float(data.qpos[2]),
                    "roll": float(roll),
                    "pitch": float(pitch),
                    "qvel_norm": qvel,
                    "contact_force": float(contact_force),
                    "contacts": float(data.ncon),
                    "max_abs_tau_so_far": float(max_abs_tau),
                }
            )

        if step < steps:
            mujoco.mj_step(model, data)

    renderer.close()
    mp4_path = out_dir / "pd_standing_render.mp4"
    gif_path = out_dir / "pd_standing_render.gif"
    write_video(frames, mp4_path, gif_path, args.fps)
    if frames:
        imageio.imwrite(out_dir / "first_frame.png", frames[0])
        imageio.imwrite(out_dir / "mid_frame.png", frames[len(frames) // 2])
        imageio.imwrite(out_dir / "last_frame.png", frames[-1])

    summary = {
        "model": str(args.model.resolve()),
        "pose_json": str(args.pose_json.resolve()) if args.pose_json else None,
        "duration": args.duration,
        "kp": args.kp,
        "kd": args.kd,
        "torque_limit": args.torque_limit,
        "fps": args.fps,
        "frames": len(frames),
        "mp4": str(mp4_path),
        "gif": str(gif_path),
        "first_frame": str(out_dir / "first_frame.png"),
        "mid_frame": str(out_dir / "mid_frame.png"),
        "last_frame": str(out_dir / "last_frame.png"),
        "final": timeline[-1] if timeline else {},
        "max_qvel_norm": max_qvel,
        "max_contact_force": max_contact,
        "max_abs_tau": max_abs_tau,
        "actuators": actuator_names,
    }
    (out_dir / "render_summary.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(json.dumps(summary, indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
