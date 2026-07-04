from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import mujoco
import numpy as np


FOOT_GEOMS = ("foot_L_1_sole_collision", "foot_R_v1_1_sole_collision")
FOOT_BODIES = ("foot_L_1", "foot_R_v1_1")


def geom_low_z(model: mujoco.MjModel, data: mujoco.MjData, geom_id: int) -> float:
    if model.geom_type[geom_id] == mujoco.mjtGeom.mjGEOM_BOX:
        geom_xmat = np.asarray(data.geom_xmat[geom_id]).reshape(3, 3)
        halfsize = np.asarray(model.geom_size[geom_id])
        z_extent = float(np.abs(geom_xmat[2, :]).dot(halfsize))
        return float(data.geom_xpos[geom_id, 2] - z_extent)
    return float(data.geom_xpos[geom_id, 2] - model.geom_rbound[geom_id])


def hinge_joints(model: mujoco.MjModel) -> list[dict[str, object]]:
    joints = []
    for joint_id in range(model.njnt):
        if model.jnt_type[joint_id] != mujoco.mjtJoint.mjJNT_HINGE:
            continue
        joints.append(
            {
                "id": joint_id,
                "name": mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, joint_id) or f"joint_{joint_id}",
                "axis": [float(v) for v in model.jnt_axis[joint_id]],
                "qposadr": int(model.jnt_qposadr[joint_id]),
                "dofadr": int(model.jnt_dofadr[joint_id]),
            }
        )
    return joints


def infer_base_z(model: mujoco.MjModel, data: mujoco.MjData, clearance: float) -> float:
    data.qpos[:] = model.qpos0
    data.qvel[:] = 0.0
    data.qpos[0:3] = np.array([0.0, 0.0, 0.0])
    data.qpos[3:7] = np.array([1.0, 0.0, 0.0, 0.0])
    mujoco.mj_forward(model, data)
    lows = []
    for geom_id in range(model.ngeom):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, geom_id)
        if name == "floor" or model.geom_contype[geom_id] == 0:
            continue
        lows.append(geom_low_z(model, data, geom_id))
    return clearance - min(lows)


def set_pose(model: mujoco.MjModel, data: mujoco.MjData, base_z: float, joint_values: dict[str, float]) -> None:
    data.qpos[:] = model.qpos0
    data.qvel[:] = 0.0
    data.qpos[0:3] = np.array([0.0, 0.0, base_z])
    data.qpos[3:7] = np.array([1.0, 0.0, 0.0, 0.0])
    for joint in hinge_joints(model):
        data.qpos[int(joint["qposadr"])] = float(joint_values.get(str(joint["name"]), 0.0))
    mujoco.mj_forward(model, data)


def total_com(model: mujoco.MjModel, data: mujoco.MjData) -> np.ndarray:
    mass = np.asarray(model.body_mass)
    return (np.asarray(data.xipos) * mass[:, None]).sum(axis=0) / mass.sum()


def support_aabb(model: mujoco.MjModel, data: mujoco.MjData) -> dict[str, float]:
    xs: list[float] = []
    ys: list[float] = []
    for geom_name in FOOT_GEOMS:
        geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, geom_name)
        center = np.asarray(data.geom_xpos[geom_id])
        size = np.asarray(model.geom_size[geom_id])
        xs.extend([float(center[0] - size[0]), float(center[0] + size[0])])
        ys.extend([float(center[1] - size[1]), float(center[1] + size[1])])
    return {"x_min": min(xs), "x_max": max(xs), "y_min": min(ys), "y_max": max(ys)}


def support_margin(point: np.ndarray, aabb: dict[str, float]) -> dict[str, float | bool]:
    x, y = float(point[0]), float(point[1])
    return {
        "inside_x": aabb["x_min"] <= x <= aabb["x_max"],
        "inside_y": aabb["y_min"] <= y <= aabb["y_max"],
        "x_min_margin": x - aabb["x_min"],
        "x_max_margin": aabb["x_max"] - x,
        "y_min_margin": y - aabb["y_min"],
        "y_max_margin": aabb["y_max"] - y,
    }


def snapshot(model: mujoco.MjModel, data: mujoco.MjData) -> dict[str, object]:
    foot_geom_pos = {}
    for geom_name in FOOT_GEOMS:
        geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, geom_name)
        foot_geom_pos[geom_name] = np.asarray(data.geom_xpos[geom_id]).copy()
    foot_body_pos = {}
    for body_name in FOOT_BODIES:
        body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body_name)
        foot_body_pos[body_name] = np.asarray(data.xpos[body_id]).copy()
    com = total_com(model, data)
    aabb = support_aabb(model, data)
    margin = support_margin(com, aabb)
    return {
        "com": com,
        "support_aabb": aabb,
        "support_margin": margin,
        "foot_geom_pos": foot_geom_pos,
        "foot_body_pos": foot_body_pos,
    }


def vec(values: np.ndarray) -> list[float]:
    return [float(v) for v in values]


def side_for_joint(name: str) -> str:
    if name.startswith("left_"):
        return "left"
    if name.startswith("right_"):
        return "right"
    return "unknown"


def main() -> int:
    parser = argparse.ArgumentParser(description="Audit +/- joint sign response around zero standing pose.")
    parser.add_argument("--model", type=Path, default=Path("envs/robots/urdf_f_link/URDF_F_link_user_size_mass_contact.xml"))
    parser.add_argument("--delta", type=float, default=0.05)
    parser.add_argument("--base-z", type=float, default=None)
    parser.add_argument("--out-dir", type=Path, default=Path("outputs/analysis/joint_sign_response_user_size_mass_contact"))
    parser.add_argument("--doc-dir", type=Path, default=Path("docs/hardware_validation"))
    args = parser.parse_args()

    model_path = args.model.resolve()
    out_dir = args.out_dir.resolve()
    doc_dir = args.doc_dir.resolve()
    out_dir.mkdir(parents=True, exist_ok=True)
    doc_dir.mkdir(parents=True, exist_ok=True)

    model = mujoco.MjModel.from_xml_path(str(model_path))
    data = mujoco.MjData(model)
    base_z = float(args.base_z) if args.base_z is not None else infer_base_z(model, data, 0.005)
    zero_values = {str(joint["name"]): 0.0 for joint in hinge_joints(model)}
    set_pose(model, data, base_z, zero_values)
    baseline = snapshot(model, data)
    joints = hinge_joints(model)

    rows: list[dict[str, object]] = []
    summary: list[dict[str, object]] = []
    for joint in joints:
        joint_name = str(joint["name"])
        side = side_for_joint(joint_name)
        for sign, label in ((1.0, "plus"), (-1.0, "minus")):
            values = dict(zero_values)
            values[joint_name] = sign * args.delta
            set_pose(model, data, base_z, values)
            snap = snapshot(model, data)
            com_delta = snap["com"] - baseline["com"]
            own_geom = "foot_L_1_sole_collision" if side == "left" else "foot_R_v1_1_sole_collision"
            other_geom = "foot_R_v1_1_sole_collision" if side == "left" else "foot_L_1_sole_collision"
            own_delta = snap["foot_geom_pos"][own_geom] - baseline["foot_geom_pos"][own_geom]
            other_delta = snap["foot_geom_pos"][other_geom] - baseline["foot_geom_pos"][other_geom]
            row = {
                "joint": joint_name,
                "side": side,
                "axis": " ".join(f"{float(v):g}" for v in joint["axis"]),
                "perturbation": label,
                "angle_rad": sign * args.delta,
                "com_dx": float(com_delta[0]),
                "com_dy": float(com_delta[1]),
                "com_dz": float(com_delta[2]),
                "own_foot_dx": float(own_delta[0]),
                "own_foot_dy": float(own_delta[1]),
                "own_foot_dz": float(own_delta[2]),
                "other_foot_dx": float(other_delta[0]),
                "other_foot_dy": float(other_delta[1]),
                "other_foot_dz": float(other_delta[2]),
                "inside_x": bool(snap["support_margin"]["inside_x"]),
                "inside_y": bool(snap["support_margin"]["inside_y"]),
                "min_support_margin": float(
                    min(
                        snap["support_margin"]["x_min_margin"],
                        snap["support_margin"]["x_max_margin"],
                        snap["support_margin"]["y_min_margin"],
                        snap["support_margin"]["y_max_margin"],
                    )
                ),
            }
            rows.append(row)

        plus = next(row for row in rows if row["joint"] == joint_name and row["perturbation"] == "plus")
        minus = next(row for row in rows if row["joint"] == joint_name and row["perturbation"] == "minus")
        own_central = np.array(
            [
                float(plus["own_foot_dx"]) - float(minus["own_foot_dx"]),
                float(plus["own_foot_dy"]) - float(minus["own_foot_dy"]),
                float(plus["own_foot_dz"]) - float(minus["own_foot_dz"]),
            ]
        ) / (2.0 * args.delta)
        com_central = np.array(
            [
                float(plus["com_dx"]) - float(minus["com_dx"]),
                float(plus["com_dy"]) - float(minus["com_dy"]),
                float(plus["com_dz"]) - float(minus["com_dz"]),
            ]
        ) / (2.0 * args.delta)
        summary.append(
            {
                "joint": joint_name,
                "side": side,
                "axis": joint["axis"],
                "own_foot_sensitivity_per_rad": vec(own_central),
                "own_foot_sensitivity_norm": float(np.linalg.norm(own_central)),
                "com_sensitivity_per_rad": vec(com_central),
                "com_sensitivity_norm": float(np.linalg.norm(com_central)),
            }
        )

    csv_path = out_dir / "joint_sign_response.csv"
    with csv_path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)

    json_path = out_dir / "joint_sign_response_summary.json"
    payload = {
        "model_path": str(model_path),
        "base_z": base_z,
        "delta_rad": args.delta,
        "baseline": {
            "com": vec(baseline["com"]),
            "support_aabb": baseline["support_aabb"],
            "support_margin": baseline["support_margin"],
            "foot_geom_pos": {key: vec(value) for key, value in baseline["foot_geom_pos"].items()},
        },
        "rows": rows,
        "summary": summary,
    }
    json_path.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")

    md_path = doc_dir / "45_joint_sign_response_audit.md"
    lines = [
        "# Joint Sign Response 감사",
        "",
        "## 목적",
        "",
        "0 rad standing pose 주변에서 각 joint를 `+/-delta`만큼 움직였을 때 COM과 좌우 foot contact가 어떻게 변하는지 기록한다. 목적은 action order, joint side, axis/sign 문제를 동역학 RL 전에 분리하는 것이다.",
        "",
        "## 실행 명령",
        "",
        "```bash",
        "python3 scripts/audit_joint_sign_response.py --model envs/robots/urdf_f_link/URDF_F_link_user_size_mass_contact.xml",
        "```",
        "",
        "## 기준 상태",
        "",
        f"- 모델: `{model_path}`",
        f"- base z: `{base_z:.6f}`",
        f"- delta: `+/-{args.delta}` rad",
        f"- baseline COM: `{vec(baseline['com'])}`",
        f"- baseline support margin: `{baseline['support_margin']}`",
        "",
        "## 요약",
        "",
        "| joint | side | axis | own foot sensitivity / rad | own norm | COM sensitivity / rad | COM norm |",
        "| --- | --- | --- | --- | ---: | --- | ---: |",
    ]
    for item in summary:
        axis = " ".join(f"{float(v):g}" for v in item["axis"])
        lines.append(
            f"| `{item['joint']}` | `{item['side']}` | `{axis}` | `{item['own_foot_sensitivity_per_rad']}` | {item['own_foot_sensitivity_norm']:.6f} | `{item['com_sensitivity_per_rad']}` | {item['com_sensitivity_norm']:.6f} |"
        )
    lines.extend(
        [
            "",
            "## 산출물",
            "",
            f"- CSV: `{csv_path}`",
            f"- JSON: `{json_path}`",
            "",
            "## 해석",
            "",
            "각 joint가 자기 쪽 foot contact에 가장 크게 반응하면 side/action order는 큰 틀에서 맞는 것으로 본다. 반대로 반대쪽 foot만 크게 움직이거나, 예상 역할과 전혀 다른 축 방향 민감도가 나오면 joint mapping 또는 sign 확인이 필요하다.",
            "",
            "이 감사는 기구학적 sign response만 본다. 동역학 standing 실패 여부는 별도 PD probe 결과와 함께 판단해야 한다.",
        ]
    )
    md_path.write_text("\n".join(lines), encoding="utf-8")

    print(f"Wrote {csv_path}")
    print(f"Wrote {json_path}")
    print(f"Wrote {md_path}")
    print(json.dumps({"base_z": base_z, "summary": summary}, indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
