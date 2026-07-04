from __future__ import annotations

import argparse
import csv
import itertools
import json
import math
from pathlib import Path

import mujoco
import numpy as np


FOOT_GEOMS = ("foot_L_1_sole_collision", "foot_R_v1_1_sole_collision")
FOOT_GEOM_PREFIXES = ("foot_L_1_sole_pad_", "foot_R_v1_1_sole_pad_")


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


def hinge_joints(model: mujoco.MjModel) -> list[dict[str, int | str]]:
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


def total_com(model: mujoco.MjModel, data: mujoco.MjData) -> np.ndarray:
    mass = np.asarray(model.body_mass)
    return (np.asarray(data.xipos) * mass[:, None]).sum(axis=0) / mass.sum()


def support_center(model: mujoco.MjModel, data: mujoco.MjData) -> tuple[np.ndarray, np.ndarray]:
    xs: list[float] = []
    ys: list[float] = []
    geom_ids: list[int] = []
    for geom_id in range(model.ngeom):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, geom_id) or ""
        if name in FOOT_GEOMS or name.startswith(FOOT_GEOM_PREFIXES):
            geom_ids.append(geom_id)
    if not geom_ids:
        raise ValueError("No sole contact geoms found for support polygon calculation")
    for geom_id in geom_ids:
        center = np.asarray(data.geom_xpos[geom_id])
        xmat = np.asarray(data.geom_xmat[geom_id]).reshape(3, 3)
        half = np.asarray(model.geom_size[geom_id])
        extent = np.abs(xmat) @ half
        xs.extend([float(center[0] - extent[0]), float(center[0] + extent[0])])
        ys.extend([float(center[1] - extent[1]), float(center[1] + extent[1])])
    center = np.array([(min(xs) + max(xs)) * 0.5, (min(ys) + max(ys)) * 0.5], dtype=np.float64)
    half = np.array([(max(xs) - min(xs)) * 0.5, (max(ys) - min(ys)) * 0.5], dtype=np.float64)
    return center, half


def contact_normal_force(model: mujoco.MjModel, data: mujoco.MjData) -> float:
    total = 0.0
    force = np.zeros(6, dtype=np.float64)
    for contact_id in range(data.ncon):
        mujoco.mj_contactForce(model, data, contact_id, force)
        total += max(float(force[0]), 0.0)
    return total


def load_pose(model: mujoco.MjModel, joints: list[dict[str, int | str]], pose_path: Path | None) -> tuple[float, np.ndarray]:
    target_q = np.array([float(model.qpos0[int(joint["qposadr"])]) for joint in joints], dtype=np.float64)
    base_z = 0.005
    if pose_path is None:
        return base_z, target_q
    payload = json.loads(pose_path.resolve().read_text(encoding="utf-8"))
    base_z = float(payload["base_z"])
    targets = payload["joint_targets"]
    for idx, joint in enumerate(joints):
        name = str(joint["name"])
        if name in targets:
            target_q[idx] = float(targets[name])
    return base_z, target_q


def initialize(model: mujoco.MjModel, data: mujoco.MjData, joints: list[dict[str, int | str]], base_z: float, target_q: np.ndarray) -> None:
    data.qpos[:] = model.qpos0
    data.qvel[:] = 0.0
    data.qpos[0:3] = np.array([0.0, 0.0, base_z])
    data.qpos[3:7] = np.array([1.0, 0.0, 0.0, 0.0])
    for value, joint in zip(target_q, joints):
        data.qpos[int(joint["qposadr"])] = float(value)
    mujoco.mj_forward(model, data)


def stabilizer_torque(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    actuator_names: list[str],
    roll: float,
    pitch: float,
    cfg: dict[str, float],
) -> np.ndarray:
    tau = np.zeros(model.nu, dtype=np.float64)
    com = total_com(model, data)
    center, half = support_center(model, data)
    com_err = (com[:2] - center) / np.maximum(half, 1e-6)
    wx = float(data.qvel[3]) if model.nv >= 6 else 0.0
    wy = float(data.qvel[4]) if model.nv >= 6 else 0.0

    roll_term = cfg["kp_att"] * roll + cfg["kd_att"] * wx + cfg["kcom"] * float(com_err[1])
    pitch_term = cfg["kp_att"] * pitch + cfg["kd_att"] * wy + cfg["kcom"] * float(com_err[0])

    additions = {
        "motor_left_hip_roll": cfg["hip_roll_sign"] * roll_term,
        "motor_right_hip_roll": cfg["hip_roll_sign"] * roll_term,
        "motor_left_ankle_roll": cfg["ankle_roll_sign"] * roll_term,
        "motor_right_ankle_roll": cfg["ankle_roll_sign"] * roll_term,
        "motor_left_hip_pitch": cfg["hip_pitch_sign"] * pitch_term,
        "motor_right_hip_pitch": cfg["hip_pitch_sign"] * pitch_term,
        "motor_left_ankle_pitch": cfg["ankle_pitch_sign"] * pitch_term,
        "motor_right_ankle_pitch": cfg["ankle_pitch_sign"] * pitch_term,
    }
    for idx, name in enumerate(actuator_names):
        tau[idx] += additions.get(name, 0.0)
    return tau


def simulate(
    model: mujoco.MjModel,
    joints: list[dict[str, int | str]],
    actuator_names: list[str],
    base_z: float,
    target_q: np.ndarray,
    cfg: dict[str, float],
    duration: float,
    joint_kp: float,
    joint_kd: float,
    torque_limit: float,
    log_every: int,
    keep_rows: bool,
) -> tuple[dict[str, object], list[dict[str, float]]]:
    data = mujoco.MjData(model)
    initialize(model, data, joints, base_z, target_q)
    steps = int(duration / model.opt.timestep)
    rows: list[dict[str, float]] = []
    max_qvel = 0.0
    max_force = 0.0
    max_abs_roll = 0.0
    max_abs_pitch = 0.0
    sat_samples = 0
    samples = 0

    for step in range(steps + 1):
        q = np.array([data.qpos[int(joint["qposadr"])] for joint in joints], dtype=np.float64)
        qd = np.array([data.qvel[int(joint["dofadr"])] for joint in joints], dtype=np.float64)
        roll, pitch, yaw = quat_to_roll_pitch_yaw(data.qpos[3:7])
        tau = joint_kp * (target_q - q) - joint_kd * qd
        tau += stabilizer_torque(model, data, actuator_names, roll, pitch, cfg)
        tau = np.clip(tau, -torque_limit, torque_limit)
        data.ctrl[:] = tau

        if step % max(log_every, 1) == 0 or step == steps:
            force = contact_normal_force(model, data)
            qvel = float(np.linalg.norm(data.qvel))
            max_qvel = max(max_qvel, qvel)
            max_force = max(max_force, force)
            max_abs_roll = max(max_abs_roll, abs(roll))
            max_abs_pitch = max(max_abs_pitch, abs(pitch))
            sat_samples += int(np.any(np.abs(tau) >= torque_limit * 0.999))
            samples += 1
            if keep_rows:
                row = {
                    "step": float(step),
                    "time": float(data.time),
                    "base_x": float(data.qpos[0]),
                    "base_y": float(data.qpos[1]),
                    "base_z": float(data.qpos[2]),
                    "base_roll": roll,
                    "base_pitch": pitch,
                    "base_yaw": yaw,
                    "contacts": float(data.ncon),
                    "contact_normal_force": force,
                    "qvel_norm": qvel,
                    "ctrl_norm": float(np.linalg.norm(tau)),
                    "max_abs_ctrl": float(np.max(np.abs(tau))),
                }
                for actuator_id, name in enumerate(actuator_names):
                    row[f"tau_{name}"] = float(data.actuator_force[actuator_id])
                rows.append(row)

        if step < steps:
            mujoco.mj_step(model, data)

    final_roll, final_pitch, final_yaw = quat_to_roll_pitch_yaw(data.qpos[3:7])
    score = (
        8.0 * max_abs_roll
        + 8.0 * max_abs_pitch
        + 0.015 * max_qvel
        + 0.0008 * max_force
        + 0.5 * (sat_samples / max(samples, 1))
        + 4.0 * max(0.0, abs(float(data.qpos[2])) - 0.35)
    )
    summary = {
        **cfg,
        "score": float(score),
        "final_base_z": float(data.qpos[2]),
        "final_roll_rad": float(final_roll),
        "final_pitch_rad": float(final_pitch),
        "final_yaw_rad": float(final_yaw),
        "max_abs_roll_rad": float(max_abs_roll),
        "max_abs_pitch_rad": float(max_abs_pitch),
        "max_qvel_norm": float(max_qvel),
        "max_contact_normal_force": float(max_force),
        "saturation_fraction": float(sat_samples / max(samples, 1)),
    }
    return summary, rows


def main() -> int:
    parser = argparse.ArgumentParser(description="Sweep simple roll/pitch/COM stabilizer gains and signs.")
    parser.add_argument("--model", type=Path, default=Path("envs/robots/urdf_f_link/URDF_F_link_user_size_mass_inertia_direct_nobase_contact.xml"))
    parser.add_argument("--pose-json", type=Path, default=Path("configs/quasistatic_standing_pose_inertia_direct_nobase.json"))
    parser.add_argument("--duration", type=float, default=2.0)
    parser.add_argument("--joint-kp", type=float, default=60.0)
    parser.add_argument("--joint-kd", type=float, default=4.0)
    parser.add_argument("--torque-limit", type=float, default=100.0)
    parser.add_argument("--log-every", type=int, default=5)
    parser.add_argument("--out-dir", type=Path, default=Path("outputs/analysis/stabilized_standing_sweep"))
    parser.add_argument("--doc-dir", type=Path, default=Path("docs/hardware_validation"))
    args = parser.parse_args()

    model_path = args.model.resolve()
    pose_path = args.pose_json.resolve()
    out_dir = args.out_dir.resolve()
    doc_dir = args.doc_dir.resolve()
    out_dir.mkdir(parents=True, exist_ok=True)
    doc_dir.mkdir(parents=True, exist_ok=True)

    model = mujoco.MjModel.from_xml_path(str(model_path))
    joints = hinge_joints(model)
    actuator_names = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, idx) or f"actuator_{idx}" for idx in range(model.nu)]
    base_z, target_q = load_pose(model, joints, pose_path)

    configs = []
    for kp_att, kd_att, kcom in itertools.product((8.0, 16.0, 32.0), (0.8, 2.0, 4.0), (0.0, 4.0, 8.0)):
        for hip_roll_sign, ankle_roll_sign, hip_pitch_sign, ankle_pitch_sign in itertools.product((-1.0, 1.0), repeat=4):
            configs.append(
                {
                    "kp_att": kp_att,
                    "kd_att": kd_att,
                    "kcom": kcom,
                    "hip_roll_sign": hip_roll_sign,
                    "ankle_roll_sign": ankle_roll_sign,
                    "hip_pitch_sign": hip_pitch_sign,
                    "ankle_pitch_sign": ankle_pitch_sign,
                }
            )

    summaries = []
    best_summary: dict[str, object] | None = None
    for cfg in configs:
        summary, _ = simulate(
            model,
            joints,
            actuator_names,
            base_z,
            target_q,
            cfg,
            args.duration,
            args.joint_kp,
            args.joint_kd,
            args.torque_limit,
            args.log_every,
            keep_rows=False,
        )
        summaries.append(summary)
        if best_summary is None or float(summary["score"]) < float(best_summary["score"]):
            best_summary = summary

    if best_summary is None:
        raise RuntimeError("No stabilizer configs evaluated.")

    best_cfg = {key: float(best_summary[key]) for key in ("kp_att", "kd_att", "kcom", "hip_roll_sign", "ankle_roll_sign", "hip_pitch_sign", "ankle_pitch_sign")}
    best_summary, best_rows = simulate(
        model,
        joints,
        actuator_names,
        base_z,
        target_q,
        best_cfg,
        args.duration,
        args.joint_kp,
        args.joint_kd,
        args.torque_limit,
        args.log_every,
        keep_rows=True,
    )

    summary_csv = out_dir / "stabilizer_sweep_summary.csv"
    with summary_csv.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(summaries[0].keys()))
        writer.writeheader()
        writer.writerows(sorted(summaries, key=lambda item: float(item["score"])))

    best_csv = out_dir / "best_stabilized_standing.csv"
    with best_csv.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(best_rows[0].keys()))
        writer.writeheader()
        writer.writerows(best_rows)

    payload = {
        "model_path": str(model_path),
        "pose_path": str(pose_path),
        "duration": args.duration,
        "joint_kp": args.joint_kp,
        "joint_kd": args.joint_kd,
        "torque_limit": args.torque_limit,
        "evaluated_configs": len(configs),
        "best_summary": best_summary,
        "summary_csv": str(summary_csv),
        "best_csv": str(best_csv),
    }
    json_path = out_dir / "stabilizer_sweep_summary.json"
    json_path.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")

    md_path = doc_dir / "55_stabilized_standing_sweep.md"
    top = sorted(summaries, key=lambda item: float(item["score"]))[:8]
    lines = [
        "# Stabilized Standing Sweep",
        "",
        "## 목적",
        "",
        "단순 joint-space PD에 base roll/pitch와 COM 오차 feedback을 추가했을 때 standing 실패가 줄어드는지 확인한다.",
        "",
        "## 실행 명령",
        "",
        "```bash",
        "python3 scripts/sweep_stabilized_standing.py",
        "```",
        "",
        "## 입력",
        "",
        f"- 모델: `{model_path}`",
        f"- pose: `{pose_path}`",
        f"- 평가 config 수: `{len(configs)}`",
        "",
        "## Best Result",
        "",
        f"- score: `{float(best_summary['score']):.6f}`",
        f"- final base z: `{float(best_summary['final_base_z']):.6f}`",
        f"- final roll: `{float(best_summary['final_roll_rad']):.6f}` rad",
        f"- final pitch: `{float(best_summary['final_pitch_rad']):.6f}` rad",
        f"- max |roll|: `{float(best_summary['max_abs_roll_rad']):.6f}` rad",
        f"- max |pitch|: `{float(best_summary['max_abs_pitch_rad']):.6f}` rad",
        f"- max qvel norm: `{float(best_summary['max_qvel_norm']):.6f}`",
        f"- max contact force: `{float(best_summary['max_contact_normal_force']):.6f}`",
        f"- saturation fraction: `{float(best_summary['saturation_fraction']):.6f}`",
        "",
        "## Best Config",
        "",
    ]
    for key in ("kp_att", "kd_att", "kcom", "hip_roll_sign", "ankle_roll_sign", "hip_pitch_sign", "ankle_pitch_sign"):
        lines.append(f"- `{key}`: `{float(best_summary[key]):.6f}`")
    lines.extend(
        [
            "",
            "## Top Configs",
            "",
            "| rank | score | final roll | final pitch | max qvel | max force | kp_att | kd_att | kcom | signs HR/AR/HP/AP |",
            "| ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- |",
        ]
    )
    for rank, item in enumerate(top, start=1):
        signs = f"{item['hip_roll_sign']:+.0f}/{item['ankle_roll_sign']:+.0f}/{item['hip_pitch_sign']:+.0f}/{item['ankle_pitch_sign']:+.0f}"
        lines.append(
            f"| {rank} | {float(item['score']):.6f} | {float(item['final_roll_rad']):.6f} | {float(item['final_pitch_rad']):.6f} | {float(item['max_qvel_norm']):.6f} | {float(item['max_contact_normal_force']):.6f} | {float(item['kp_att']):.1f} | {float(item['kd_att']):.1f} | {float(item['kcom']):.1f} | `{signs}` |"
        )
    lines.extend(
        [
            "",
            "## 산출물",
            "",
            f"- sweep summary CSV: `{summary_csv}`",
            f"- sweep summary JSON: `{json_path}`",
            f"- best run CSV: `{best_csv}`",
            "",
            "## 판단",
            "",
            "이 stabilizer는 최종 제어기가 아니라 원인 분리용 heuristic이다. 결과가 좋아지면 제어기 부재가 큰 원인이고, 여전히 크게 무너지면 contact/inertia/joint axis 문제 가능성이 남는다.",
            "",
        ]
    )
    md_path.write_text("\n".join(lines), encoding="utf-8")

    print(f"Wrote {summary_csv}")
    print(f"Wrote {best_csv}")
    print(f"Wrote {json_path}")
    print(f"Wrote {md_path}")
    print(json.dumps({"evaluated_configs": len(configs), "best_score": best_summary["score"], "best_final_roll": best_summary["final_roll_rad"], "best_final_pitch": best_summary["final_pitch_rad"]}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
