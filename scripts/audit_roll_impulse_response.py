from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path

import mujoco
import numpy as np


ROLL_ACTUATORS = (
    "motor_left_hip_roll",
    "motor_left_ankle_roll",
    "motor_right_hip_roll",
    "motor_right_ankle_roll",
)
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
                "name": mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, joint_id) or f"joint_{joint_id}",
                "qposadr": int(model.jnt_qposadr[joint_id]),
                "dofadr": int(model.jnt_dofadr[joint_id]),
            }
        )
    return joints


def load_pose(model: mujoco.MjModel, joints: list[dict[str, int | str]], pose_path: Path) -> tuple[float, np.ndarray]:
    payload = json.loads(pose_path.resolve().read_text(encoding="utf-8"))
    target_q = np.array([float(model.qpos0[int(joint["qposadr"])]) for joint in joints], dtype=np.float64)
    targets = payload["joint_targets"]
    for idx, joint in enumerate(joints):
        name = str(joint["name"])
        if name in targets:
            target_q[idx] = float(targets[name])
    return float(payload["base_z"]), target_q


def initialize(model: mujoco.MjModel, data: mujoco.MjData, joints: list[dict[str, int | str]], base_z: float, target_q: np.ndarray) -> None:
    data.qpos[:] = model.qpos0
    data.qvel[:] = 0.0
    data.qpos[0:3] = np.array([0.0, 0.0, base_z])
    data.qpos[3:7] = np.array([1.0, 0.0, 0.0, 0.0])
    for value, joint in zip(target_q, joints):
        data.qpos[int(joint["qposadr"])] = float(value)
    mujoco.mj_forward(model, data)


def contact_stats(model: mujoco.MjModel, data: mujoco.MjData) -> dict[str, float]:
    geom_to_side = foot_geom_ids(model)

    stats = {
        "contact_normal_force": 0.0,
        "left_contact_force": 0.0,
        "right_contact_force": 0.0,
        "left_contact_count": 0.0,
        "right_contact_count": 0.0,
        "left_cop_x": 0.0,
        "left_cop_y": 0.0,
        "right_cop_x": 0.0,
        "right_cop_y": 0.0,
    }
    weighted_pos = {"left": np.zeros(3), "right": np.zeros(3)}
    force_sum = {"left": 0.0, "right": 0.0}
    force = np.zeros(6, dtype=np.float64)
    for contact_id in range(data.ncon):
        contact = data.contact[contact_id]
        side = geom_to_side.get(int(contact.geom1)) or geom_to_side.get(int(contact.geom2))
        if side is None:
            continue
        mujoco.mj_contactForce(model, data, contact_id, force)
        normal_force = max(float(force[0]), 0.0)
        stats["contact_normal_force"] += normal_force
        stats[f"{side}_contact_force"] += normal_force
        stats[f"{side}_contact_count"] += 1.0
        weighted_pos[side] += normal_force * np.asarray(contact.pos)
        force_sum[side] += normal_force

    for side in ("left", "right"):
        if force_sum[side] > 1e-9:
            cop = weighted_pos[side] / force_sum[side]
            stats[f"{side}_cop_x"] = float(cop[0])
            stats[f"{side}_cop_y"] = float(cop[1])
    return stats


def simulate_case(
    model: mujoco.MjModel,
    joints: list[dict[str, int | str]],
    actuator_names: list[str],
    base_z: float,
    target_q: np.ndarray,
    actuator_name: str,
    impulse_sign: float,
    args: argparse.Namespace,
) -> tuple[dict[str, object], list[dict[str, float]]]:
    data = mujoco.MjData(model)
    initialize(model, data, joints, base_z, target_q)
    actuator_id = actuator_names.index(actuator_name)
    steps = int(args.duration / model.opt.timestep)
    impulse_start_step = int(args.impulse_start / model.opt.timestep)
    impulse_end_step = int((args.impulse_start + args.impulse_duration) / model.opt.timestep)
    rows = []
    max_abs_roll = 0.0
    max_abs_pitch = 0.0
    max_qvel = 0.0
    max_contact_force = 0.0
    roll_before_impulse = None

    for step in range(steps + 1):
        q = np.array([data.qpos[int(joint["qposadr"])] for joint in joints], dtype=np.float64)
        qd = np.array([data.qvel[int(joint["dofadr"])] for joint in joints], dtype=np.float64)
        roll, pitch, yaw = quat_to_roll_pitch_yaw(data.qpos[3:7])
        if step == impulse_start_step:
            roll_before_impulse = roll

        tau = args.joint_kp * (target_q - q) - args.joint_kd * qd
        if impulse_start_step <= step < impulse_end_step:
            tau[actuator_id] += impulse_sign * args.impulse_torque
        tau = np.clip(tau, -args.torque_limit, args.torque_limit)
        data.ctrl[:] = tau

        stats = contact_stats(model, data)
        qvel = float(np.linalg.norm(data.qvel))
        max_abs_roll = max(max_abs_roll, abs(roll))
        max_abs_pitch = max(max_abs_pitch, abs(pitch))
        max_qvel = max(max_qvel, qvel)
        max_contact_force = max(max_contact_force, float(stats["contact_normal_force"]))

        if step % max(args.log_every, 1) == 0 or step == steps:
            row = {
                "step": float(step),
                "time": float(data.time),
                "case": float(0.0),
                "base_x": float(data.qpos[0]),
                "base_y": float(data.qpos[1]),
                "base_z": float(data.qpos[2]),
                "base_roll": roll,
                "base_pitch": pitch,
                "base_yaw": yaw,
                "qvel_norm": qvel,
                "ctrl_norm": float(np.linalg.norm(tau)),
                "max_abs_ctrl": float(np.max(np.abs(tau))),
                **stats,
            }
            for idx, name in enumerate(actuator_names):
                row[f"tau_{name}"] = float(data.actuator_force[idx])
            rows.append(row)

        if step < steps:
            mujoco.mj_step(model, data)

    final_roll, final_pitch, final_yaw = quat_to_roll_pitch_yaw(data.qpos[3:7])
    initial_roll = rows[0]["base_roll"] if rows else 0.0
    summary = {
        "actuator": actuator_name,
        "impulse_sign": impulse_sign,
        "impulse_torque": args.impulse_torque,
        "impulse_duration": args.impulse_duration,
        "initial_roll_rad": float(initial_roll),
        "roll_before_impulse_rad": float(roll_before_impulse if roll_before_impulse is not None else initial_roll),
        "final_roll_rad": float(final_roll),
        "final_pitch_rad": float(final_pitch),
        "delta_roll_final_from_initial": float(final_roll - initial_roll),
        "delta_roll_final_from_impulse_start": float(final_roll - (roll_before_impulse if roll_before_impulse is not None else initial_roll)),
        "max_abs_roll_rad": float(max_abs_roll),
        "max_abs_pitch_rad": float(max_abs_pitch),
        "max_qvel_norm": float(max_qvel),
        "max_contact_normal_force": float(max_contact_force),
        "final_left_contact_force": float(rows[-1]["left_contact_force"]) if rows else 0.0,
        "final_right_contact_force": float(rows[-1]["right_contact_force"]) if rows else 0.0,
    }
    return summary, rows


def main() -> int:
    parser = argparse.ArgumentParser(description="Apply short torque impulses on roll actuators and record base/contact response.")
    parser.add_argument("--model", type=Path, default=Path("envs/robots/urdf_f_link/URDF_F_link_user_size_mass_inertia_direct_nobase_contact.xml"))
    parser.add_argument("--pose-json", type=Path, default=Path("configs/quasistatic_standing_pose_inertia_direct_nobase.json"))
    parser.add_argument("--base-z-offset", type=float, default=0.0)
    parser.add_argument("--duration", type=float, default=0.45)
    parser.add_argument("--impulse-start", type=float, default=0.08)
    parser.add_argument("--impulse-duration", type=float, default=0.06)
    parser.add_argument("--impulse-torque", type=float, default=35.0)
    parser.add_argument("--joint-kp", type=float, default=60.0)
    parser.add_argument("--joint-kd", type=float, default=4.0)
    parser.add_argument("--torque-limit", type=float, default=100.0)
    parser.add_argument("--log-every", type=int, default=2)
    parser.add_argument("--out-dir", type=Path, default=Path("outputs/analysis/roll_impulse_response"))
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
    base_z += args.base_z_offset

    summaries = []
    all_rows = []
    for actuator_name in ROLL_ACTUATORS:
        for sign in (1.0, -1.0):
            summary, rows = simulate_case(model, joints, actuator_names, base_z, target_q, actuator_name, sign, args)
            case_id = len(summaries)
            for row in rows:
                row["case"] = float(case_id)
                row["impulse_sign"] = sign
                row["impulse_actuator_id"] = float(actuator_names.index(actuator_name))
            summaries.append(summary)
            all_rows.extend(rows)

    csv_path = out_dir / "roll_impulse_response.csv"
    with csv_path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(all_rows[0].keys()))
        writer.writeheader()
        writer.writerows(all_rows)

    summary_csv = out_dir / "roll_impulse_response_summary.csv"
    with summary_csv.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(summaries[0].keys()))
        writer.writeheader()
        writer.writerows(summaries)

    payload = {
        "model_path": str(model_path),
        "pose_path": str(pose_path),
        "base_z": base_z,
        "base_z_offset": args.base_z_offset,
        "duration": args.duration,
        "impulse_start": args.impulse_start,
        "impulse_duration": args.impulse_duration,
        "impulse_torque": args.impulse_torque,
        "joint_kp": args.joint_kp,
        "joint_kd": args.joint_kd,
        "torque_limit": args.torque_limit,
        "summaries": summaries,
        "csv_path": str(csv_path),
        "summary_csv": str(summary_csv),
    }
    json_path = out_dir / "roll_impulse_response_summary.json"
    json_path.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")

    md_path = doc_dir / "57_roll_impulse_response.md"
    lines = [
        "# Roll Impulse Response Audit",
        "",
        "## 목적",
        "",
        "standing 실패가 roll 방향에서 지속되므로, 각 roll actuator에 짧은 torque impulse를 넣어 base roll과 좌우 foot contact force가 어떤 방향으로 반응하는지 확인한다.",
        "",
        "## 실행 명령",
        "",
        "```bash",
        "python3 scripts/audit_roll_impulse_response.py",
        "```",
        "",
        "## 입력",
        "",
        f"- 모델: `{model_path}`",
        f"- pose: `{pose_path}`",
        f"- impulse torque: `{args.impulse_torque} Nm`",
        f"- impulse duration: `{args.impulse_duration} s`",
        f"- torque limit: `{args.torque_limit} Nm`",
        f"- base z offset: `{args.base_z_offset} m`",
        "",
        "## 결과",
        "",
        "| actuator | sign | final roll | delta roll from impulse start | final pitch | max qvel | max contact force | final L/R contact force |",
        "| --- | ---: | ---: | ---: | ---: | ---: | ---: | --- |",
    ]
    for item in summaries:
        lines.append(
            f"| `{item['actuator']}` | {item['impulse_sign']:+.0f} | {item['final_roll_rad']:.6f} | {item['delta_roll_final_from_impulse_start']:.6f} | {item['final_pitch_rad']:.6f} | {item['max_qvel_norm']:.6f} | {item['max_contact_normal_force']:.6f} | `{item['final_left_contact_force']:.3f} / {item['final_right_contact_force']:.3f}` |"
        )
    lines.extend(
        [
            "",
            "## 산출물",
            "",
            f"- time-series CSV: `{csv_path}`",
            f"- summary CSV: `{summary_csv}`",
            f"- summary JSON: `{json_path}`",
            "",
            "## 판단 기준",
            "",
            "같은 actuator에서 `+`와 `-` impulse가 base roll을 명확히 반대 방향으로 움직이면 torque sign 관측은 정상이다. 두 방향이 모두 같은 쪽으로 무너지거나 contact force가 한쪽으로 급격히 사라지면, roll contact support 또는 joint axis/sign 모델을 추가로 확인해야 한다.",
            "",
        ]
    )
    md_path.write_text("\n".join(lines), encoding="utf-8")

    print(f"Wrote {csv_path}")
    print(f"Wrote {summary_csv}")
    print(f"Wrote {json_path}")
    print(f"Wrote {md_path}")
    print(json.dumps({"cases": len(summaries), "csv_path": str(csv_path)}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
