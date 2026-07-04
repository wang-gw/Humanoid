from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path

import mujoco
import numpy as np


def geom_low_z(model: mujoco.MjModel, data: mujoco.MjData, geom_id: int) -> float:
    if model.geom_type[geom_id] == mujoco.mjtGeom.mjGEOM_BOX:
        geom_xmat = np.asarray(data.geom_xmat[geom_id]).reshape(3, 3)
        halfsize = np.asarray(model.geom_size[geom_id])
        z_extent = float(np.abs(geom_xmat[2, :]).dot(halfsize))
        return float(data.geom_xpos[geom_id, 2] - z_extent)
    return float(data.geom_xpos[geom_id, 2] - model.geom_rbound[geom_id])


def quat_to_roll_pitch_yaw(q: np.ndarray) -> tuple[float, float, float]:
    w, x, y, z = [float(v) for v in q]
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def infer_base_z(model: mujoco.MjModel, data: mujoco.MjData, clearance: float) -> float:
    data.qpos[:] = model.qpos0
    if model.nq >= 7:
        data.qpos[2] = 0.0
        data.qpos[3:7] = np.array([1.0, 0.0, 0.0, 0.0])
    mujoco.mj_forward(model, data)
    floor_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "floor")
    lows = []
    for geom_id in range(model.ngeom):
        if geom_id == floor_id:
            continue
        if model.geom_contype[geom_id] == 0:
            continue
        lows.append(geom_low_z(model, data, geom_id))
    lowest = min(lows) if lows else 0.0
    return max(0.0, clearance - lowest)


def hinge_joint_info(model: mujoco.MjModel) -> list[dict[str, int | str]]:
    joints: list[dict[str, int | str]] = []
    for joint_id in range(model.njnt):
        if model.jnt_type[joint_id] != mujoco.mjtJoint.mjJNT_HINGE:
            continue
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, joint_id) or f"joint_{joint_id}"
        joints.append(
            {
                "id": joint_id,
                "name": name,
                "qposadr": int(model.jnt_qposadr[joint_id]),
                "dofadr": int(model.jnt_dofadr[joint_id]),
            }
        )
    return joints


def contact_normal_force(model: mujoco.MjModel, data: mujoco.MjData) -> float:
    total = 0.0
    force = np.zeros(6, dtype=np.float64)
    for contact_id in range(data.ncon):
        mujoco.mj_contactForce(model, data, contact_id, force)
        total += max(float(force[0]), 0.0)
    return total


def main() -> int:
    parser = argparse.ArgumentParser(description="Run a neutral-pose joint PD standing probe.")
    parser.add_argument("--model", type=Path, default=Path("envs/robots/urdf_f/URDF_F_mujoco.xml"))
    parser.add_argument("--duration", type=float, default=2.0)
    parser.add_argument("--kp", type=float, default=60.0)
    parser.add_argument("--kd", type=float, default=4.0)
    parser.add_argument("--torque-limit", type=float, default=100.0)
    parser.add_argument("--base-z", type=float, default=None)
    parser.add_argument("--pose-json", type=Path, default=None)
    parser.add_argument("--clearance", type=float, default=0.005)
    parser.add_argument("--log-every", type=int, default=5)
    parser.add_argument("--out-dir", type=Path, default=Path("outputs/analysis/pd_standing"))
    parser.add_argument("--doc-dir", type=Path, default=Path("docs/hardware_validation"))
    parser.add_argument("--doc-name", default="04_neutral_pd_standing_probe.md")
    args = parser.parse_args()

    model_path = args.model.resolve()
    out_dir = args.out_dir.resolve()
    doc_dir = args.doc_dir.resolve()
    out_dir.mkdir(parents=True, exist_ok=True)
    doc_dir.mkdir(parents=True, exist_ok=True)

    model = mujoco.MjModel.from_xml_path(str(model_path))
    data = mujoco.MjData(model)
    joints = hinge_joint_info(model)
    if len(joints) != model.nu:
        raise ValueError(f"Expected one actuator per hinge joint, got hinges={len(joints)} nu={model.nu}")

    pose_payload = None
    if args.pose_json is not None:
        pose_payload = json.loads(args.pose_json.resolve().read_text(encoding="utf-8"))

    base_z = float(args.base_z) if args.base_z is not None else infer_base_z(model, data, args.clearance)
    if pose_payload is not None and args.base_z is None:
        base_z = float(pose_payload["base_z"])
    target_q = np.array([float(model.qpos0[int(joint["qposadr"])]) for joint in joints], dtype=np.float64)
    if pose_payload is not None:
        targets = pose_payload["joint_targets"]
        for idx, joint in enumerate(joints):
            name = str(joint["name"])
            if name in targets:
                target_q[idx] = float(targets[name])

    data.qpos[:] = model.qpos0
    data.qvel[:] = 0.0
    if model.nq >= 7:
        data.qpos[0:3] = np.array([0.0, 0.0, base_z])
        data.qpos[3:7] = np.array([1.0, 0.0, 0.0, 0.0])
    for value, joint in zip(target_q, joints):
        data.qpos[int(joint["qposadr"])] = float(value)
    mujoco.mj_forward(model, data)
    actuator_names = [
        mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, idx) or f"actuator_{idx}"
        for idx in range(model.nu)
    ]

    steps = int(max(args.duration, 0.0) / model.opt.timestep)
    rows: list[dict[str, float]] = []
    max_abs_tau = np.zeros(model.nu, dtype=np.float64)
    sum_tau_sq = np.zeros(model.nu, dtype=np.float64)
    samples = 0

    for step in range(steps + 1):
        q = np.array([data.qpos[int(joint["qposadr"])] for joint in joints], dtype=np.float64)
        qd = np.array([data.qvel[int(joint["dofadr"])] for joint in joints], dtype=np.float64)
        tau = args.kp * (target_q - q) - args.kd * qd
        tau = np.clip(tau, -args.torque_limit, args.torque_limit)
        data.ctrl[:] = tau

        if step % max(args.log_every, 1) == 0 or step == steps:
            roll, pitch, yaw = quat_to_roll_pitch_yaw(data.qpos[3:7]) if model.nq >= 7 else (0.0, 0.0, 0.0)
            actuator_force = np.array(data.actuator_force, dtype=np.float64) if model.nu else np.zeros(0)
            max_abs_tau = np.maximum(max_abs_tau, np.abs(actuator_force))
            sum_tau_sq += actuator_force * actuator_force
            samples += 1
            row: dict[str, float] = {
                "step": float(step),
                "time": float(data.time),
                "base_x": float(data.qpos[0]) if model.nq >= 1 else 0.0,
                "base_y": float(data.qpos[1]) if model.nq >= 2 else 0.0,
                "base_z": float(data.qpos[2]) if model.nq >= 3 else 0.0,
                "base_roll": roll,
                "base_pitch": pitch,
                "base_yaw": yaw,
                "contacts": float(data.ncon),
                "contact_normal_force": contact_normal_force(model, data),
                "qvel_norm": float(np.linalg.norm(data.qvel)),
                "ctrl_norm": float(np.linalg.norm(data.ctrl)),
                "max_abs_ctrl": float(np.max(np.abs(data.ctrl))) if model.nu else 0.0,
            }
            for actuator_id in range(model.nu):
                row[f"tau_{actuator_names[actuator_id]}"] = float(actuator_force[actuator_id])
            rows.append(row)

        if step < steps:
            mujoco.mj_step(model, data)

    csv_path = out_dir / "neutral_pd_standing.csv"
    with csv_path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)

    rms_tau = np.sqrt(sum_tau_sq / max(samples, 1))
    final = rows[-1]
    summary = {
        "model_path": str(model_path),
        "duration": args.duration,
        "kp": args.kp,
        "kd": args.kd,
        "torque_limit": args.torque_limit,
        "base_z": base_z,
        "steps": steps,
        "final_base_z": final["base_z"],
        "final_roll_rad": final["base_roll"],
        "final_pitch_rad": final["base_pitch"],
        "max_qvel_norm": max(row["qvel_norm"] for row in rows),
        "max_contact_normal_force": max(row["contact_normal_force"] for row in rows),
        "max_abs_tau_by_actuator": dict(zip(actuator_names, [float(v) for v in max_abs_tau])),
        "rms_tau_by_actuator": dict(zip(actuator_names, [float(v) for v in rms_tau])),
        "csv_path": str(csv_path),
    }
    json_path = out_dir / "neutral_pd_standing_summary.json"
    json_path.write_text(json.dumps(summary, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")

    max_tau = max(summary["max_abs_tau_by_actuator"].values()) if actuator_names else 0.0
    md_path = doc_dir / args.doc_name
    md_path.write_text(
        "\n".join(
            [
                "# Neutral PD Standing Probe",
                "",
                "## Purpose",
                "",
                "RL 학습 전, 현재 floating-base MJCF가 neutral joint pose에서 단순 joint-space PD로 버틸 수 있는지 확인한다. 이 단계는 최종 보행 가능성 판정이 아니라 standing/squat/weight-shift로 넘어가기 위한 sanity check다.",
                "",
                "## Command",
                "",
                "```bash",
                "python3 scripts/probe_pd_standing.py",
                "```",
                "",
                "## Controller",
                "",
                f"- Target joint pose: `model.qpos0` hinge values",
                f"- Kp: `{args.kp}`",
                f"- Kd: `{args.kd}`",
                f"- Torque limit: `+/-{args.torque_limit} Nm`",
                f"- Initial base z: `{base_z:.6f}`",
                "",
                "## Summary",
                "",
                f"- Duration: `{args.duration}` sec",
                f"- Final base z: `{summary['final_base_z']:.6f}`",
                f"- Final roll: `{summary['final_roll_rad']:.6f}` rad",
                f"- Final pitch: `{summary['final_pitch_rad']:.6f}` rad",
                f"- Max qvel norm: `{summary['max_qvel_norm']:.6f}`",
                f"- Max contact normal force: `{summary['max_contact_normal_force']:.6f}`",
                f"- Max actuator torque observed: `{max_tau:.6f}` Nm",
                "",
                "## Outputs",
                "",
                f"- CSV: `{csv_path}`",
                f"- JSON: `{json_path}`",
                "",
                "## Interpretation",
                "",
                "Neutral pose PD가 안정적이지 않으면 바로 RL로 넘어가지 않는다. 먼저 neutral standing pose, foot collision, base height, joint axis, mass distribution을 점검해야 한다.",
                "",
            ]
        ),
        encoding="utf-8",
    )

    print(f"Wrote {csv_path}")
    print(f"Wrote {json_path}")
    print(f"Wrote {md_path}")
    print(json.dumps({k: v for k, v in summary.items() if k not in {'max_abs_tau_by_actuator', 'rms_tau_by_actuator'}}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
