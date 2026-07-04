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

OBSERVED_BODIES = ("base_link", "foot_L_1", "foot_R_v1_1", "thighJ_L_1", "thighJ_R_1")


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


def load_pose(model: mujoco.MjModel, joints: list[dict[str, int | str]], pose_path: Path) -> np.ndarray:
    payload = json.loads(pose_path.resolve().read_text(encoding="utf-8"))
    target_q = np.array([float(model.qpos0[int(joint["qposadr"])]) for joint in joints], dtype=np.float64)
    targets = payload["joint_targets"]
    for idx, joint in enumerate(joints):
        name = str(joint["name"])
        if name in targets:
            target_q[idx] = float(targets[name])
    return target_q


def initialize(model: mujoco.MjModel, data: mujoco.MjData, joints: list[dict[str, int | str]], base_z: float, target_q: np.ndarray) -> None:
    data.qpos[:] = model.qpos0
    data.qvel[:] = 0.0
    data.qpos[0:3] = np.array([0.0, 0.0, base_z])
    data.qpos[3:7] = np.array([1.0, 0.0, 0.0, 0.0])
    for value, joint in zip(target_q, joints):
        data.qpos[int(joint["qposadr"])] = float(value)
    mujoco.mj_forward(model, data)


def body_rpy(model: mujoco.MjModel, data: mujoco.MjData, body_name: str) -> tuple[float, float, float]:
    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body_name)
    return quat_to_roll_pitch_yaw(np.asarray(data.xquat[body_id]))


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
    joint_id = int(model.actuator_trnid[actuator_id, 0])
    joint_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, joint_id) or f"joint_{joint_id}"
    qposadr = int(model.jnt_qposadr[joint_id])
    steps = int(args.duration / model.opt.timestep)
    impulse_start_step = int(args.impulse_start / model.opt.timestep)
    impulse_end_step = int((args.impulse_start + args.impulse_duration) / model.opt.timestep)

    initial_base_rpy = body_rpy(model, data, "base_link")
    initial_body_rpy = {name: body_rpy(model, data, name) for name in OBSERVED_BODIES}
    initial_joint_q = float(data.qpos[qposadr])
    rows: list[dict[str, float]] = []

    for step in range(steps + 1):
        tau = np.zeros(model.nu, dtype=np.float64)
        if impulse_start_step <= step < impulse_end_step:
            tau[actuator_id] = impulse_sign * args.impulse_torque
        tau = np.clip(tau, -args.torque_limit, args.torque_limit)
        data.ctrl[:] = tau

        if step % max(args.log_every, 1) == 0 or step == steps:
            base_roll, base_pitch, base_yaw = quat_to_roll_pitch_yaw(data.qpos[3:7])
            row = {
                "step": float(step),
                "time": float(data.time),
                "base_x": float(data.qpos[0]),
                "base_y": float(data.qpos[1]),
                "base_z": float(data.qpos[2]),
                "base_roll": base_roll,
                "base_pitch": base_pitch,
                "base_yaw": base_yaw,
                "qvel_norm": float(np.linalg.norm(data.qvel)),
                "impulse_actuator_id": float(actuator_id),
                "impulse_sign": impulse_sign,
                "observed_joint_q": float(data.qpos[qposadr]),
                "observed_joint_qd": float(data.qvel[int(model.jnt_dofadr[joint_id])]),
            }
            for body_name in OBSERVED_BODIES:
                roll, pitch, yaw = body_rpy(model, data, body_name)
                row[f"{body_name}_roll"] = roll
                row[f"{body_name}_pitch"] = pitch
                row[f"{body_name}_yaw"] = yaw
            for idx, name in enumerate(actuator_names):
                row[f"tau_{name}"] = float(data.actuator_force[idx])
            rows.append(row)

        if step < steps:
            mujoco.mj_step(model, data)

    final_body_rpy = {name: body_rpy(model, data, name) for name in OBSERVED_BODIES}
    final_base_rpy = body_rpy(model, data, "base_link")
    summary = {
        "actuator": actuator_name,
        "joint": joint_name,
        "impulse_sign": impulse_sign,
        "impulse_torque": args.impulse_torque,
        "impulse_duration": args.impulse_duration,
        "initial_joint_q": initial_joint_q,
        "final_joint_q": float(data.qpos[qposadr]),
        "delta_joint_q": float(data.qpos[qposadr] - initial_joint_q),
        "initial_base_roll": float(initial_base_rpy[0]),
        "final_base_roll": float(final_base_rpy[0]),
        "delta_base_roll": float(final_base_rpy[0] - initial_base_rpy[0]),
        "final_base_pitch": float(final_base_rpy[1]),
        "delta_base_pitch": float(final_base_rpy[1] - initial_base_rpy[1]),
        "max_qvel_norm": max(float(row["qvel_norm"]) for row in rows),
        "body_delta_roll": {
            name: float(final_body_rpy[name][0] - initial_body_rpy[name][0]) for name in OBSERVED_BODIES
        },
        "body_delta_pitch": {
            name: float(final_body_rpy[name][1] - initial_body_rpy[name][1]) for name in OBSERVED_BODIES
        },
    }
    return summary, rows


def main() -> int:
    parser = argparse.ArgumentParser(description="Audit roll actuator impulse response without gravity/contact.")
    parser.add_argument("--model", type=Path, default=Path("envs/robots/urdf_f_link/URDF_F_link_user_size_mass_inertia_direct_nobase_contact.xml"))
    parser.add_argument("--pose-json", type=Path, default=Path("configs/quasistatic_standing_pose_inertia_direct_nobase.json"))
    parser.add_argument("--base-z", type=float, default=1.0)
    parser.add_argument("--duration", type=float, default=0.25)
    parser.add_argument("--impulse-start", type=float, default=0.02)
    parser.add_argument("--impulse-duration", type=float, default=0.08)
    parser.add_argument("--impulse-torque", type=float, default=25.0)
    parser.add_argument("--torque-limit", type=float, default=100.0)
    parser.add_argument("--log-every", type=int, default=2)
    parser.add_argument("--out-dir", type=Path, default=Path("outputs/analysis/roll_free_impulse_response"))
    parser.add_argument("--doc-dir", type=Path, default=Path("docs/hardware_validation"))
    args = parser.parse_args()

    model_path = args.model.resolve()
    pose_path = args.pose_json.resolve()
    out_dir = args.out_dir.resolve()
    doc_dir = args.doc_dir.resolve()
    out_dir.mkdir(parents=True, exist_ok=True)
    doc_dir.mkdir(parents=True, exist_ok=True)

    model = mujoco.MjModel.from_xml_path(str(model_path))
    model.opt.gravity[:] = 0.0
    joints = hinge_joints(model)
    actuator_names = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, idx) or f"actuator_{idx}" for idx in range(model.nu)]
    target_q = load_pose(model, joints, pose_path)

    summaries = []
    all_rows = []
    for actuator_name in ROLL_ACTUATORS:
        for sign in (1.0, -1.0):
            summary, rows = simulate_case(model, joints, actuator_names, args.base_z, target_q, actuator_name, sign, args)
            case_id = len(summaries)
            for row in rows:
                row["case"] = float(case_id)
            summaries.append(summary)
            all_rows.extend(rows)

    csv_path = out_dir / "roll_free_impulse_response.csv"
    with csv_path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(all_rows[0].keys()))
        writer.writeheader()
        writer.writerows(all_rows)

    summary_csv = out_dir / "roll_free_impulse_response_summary.csv"
    flat_summaries = []
    for item in summaries:
        row = {key: value for key, value in item.items() if key not in {"body_delta_roll", "body_delta_pitch"}}
        for body_name, value in item["body_delta_roll"].items():
            row[f"{body_name}_delta_roll"] = value
        for body_name, value in item["body_delta_pitch"].items():
            row[f"{body_name}_delta_pitch"] = value
        flat_summaries.append(row)
    with summary_csv.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(flat_summaries[0].keys()))
        writer.writeheader()
        writer.writerows(flat_summaries)

    payload = {
        "model_path": str(model_path),
        "pose_path": str(pose_path),
        "gravity": [float(v) for v in model.opt.gravity],
        "base_z": args.base_z,
        "duration": args.duration,
        "impulse_start": args.impulse_start,
        "impulse_duration": args.impulse_duration,
        "impulse_torque": args.impulse_torque,
        "summaries": summaries,
        "csv_path": str(csv_path),
        "summary_csv": str(summary_csv),
    }
    json_path = out_dir / "roll_free_impulse_response_summary.json"
    json_path.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")

    md_path = doc_dir / "60_roll_free_impulse_response.md"
    lines = [
        "# Roll Free Impulse Response",
        "",
        "## 목적",
        "",
        "contact와 중력 영향을 제거한 상태에서 roll actuator torque sign과 link 회전 방향을 확인한다. 이 실험은 roll contact 지지 문제와 joint axis/sign 문제를 분리하기 위한 것이다.",
        "",
        "## 실행 명령",
        "",
        "```bash",
        "python3 scripts/audit_roll_free_impulse_response.py",
        "```",
        "",
        "## 조건",
        "",
        f"- 모델: `{model_path}`",
        f"- pose: `{pose_path}`",
        "- gravity: `[0, 0, 0]`",
        f"- base z: `{args.base_z}`",
        f"- impulse torque: `{args.impulse_torque} Nm`",
        f"- impulse duration: `{args.impulse_duration} s`",
        "",
        "## 결과",
        "",
        "| actuator | joint | sign | delta joint q | delta base roll | L foot delta roll | R foot delta roll | max qvel |",
        "| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: |",
    ]
    for item in summaries:
        lines.append(
            f"| `{item['actuator']}` | `{item['joint']}` | {item['impulse_sign']:+.0f} | {item['delta_joint_q']:.6f} | {item['delta_base_roll']:.6f} | {item['body_delta_roll']['foot_L_1']:.6f} | {item['body_delta_roll']['foot_R_v1_1']:.6f} | {item['max_qvel_norm']:.6f} |"
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
            "같은 actuator에서 `+`와 `-` impulse의 `delta_joint_q`가 명확히 반대 부호면 actuator-to-joint sign은 MuJoCo 내부에서 정상적으로 작동한다. 이때 contact 실험에서 반대 방향 roll이 나오지 않는다면, 우선 원인은 contact/초기자세/관성 모델 쪽으로 본다.",
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
