from __future__ import annotations

import argparse
import json
from pathlib import Path

import mujoco
import numpy as np


FOOT_GEOMS = ("foot_L_1_sole_collision", "foot_R_v1_1_sole_collision")


def hinge_joints(model: mujoco.MjModel) -> list[dict[str, object]]:
    joints = []
    for joint_id in range(model.njnt):
        if model.jnt_type[joint_id] != mujoco.mjtJoint.mjJNT_HINGE:
            continue
        joints.append(
            {
                "name": mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, joint_id) or f"joint_{joint_id}",
                "qposadr": int(model.jnt_qposadr[joint_id]),
                "range": np.asarray(model.jnt_range[joint_id], dtype=np.float64),
            }
        )
    return joints


def geom_extents(model: mujoco.MjModel, data: mujoco.MjData, geom_id: int) -> np.ndarray:
    xmat = np.asarray(data.geom_xmat[geom_id]).reshape(3, 3)
    halfsize = np.asarray(model.geom_size[geom_id])
    return np.abs(xmat) @ halfsize


def total_com(model: mujoco.MjModel, data: mujoco.MjData) -> np.ndarray:
    mass = np.asarray(model.body_mass)
    return (np.asarray(data.xipos) * mass[:, None]).sum(axis=0) / mass.sum()


def set_pose(model: mujoco.MjModel, data: mujoco.MjData, joints: list[dict[str, object]], q: np.ndarray, clearance: float) -> float:
    data.qpos[:] = model.qpos0
    data.qvel[:] = 0.0
    data.qpos[0:3] = np.array([0.0, 0.0, 0.0])
    data.qpos[3:7] = np.array([1.0, 0.0, 0.0, 0.0])
    for value, joint in zip(q, joints):
        data.qpos[int(joint["qposadr"])] = float(value)
    mujoco.mj_forward(model, data)

    lows = []
    for name in FOOT_GEOMS:
        geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name)
        extents = geom_extents(model, data, geom_id)
        lows.append(float(data.geom_xpos[geom_id, 2] - extents[2]))
    base_z = clearance - min(lows)
    data.qpos[2] = base_z
    mujoco.mj_forward(model, data)
    return float(base_z)


def evaluate(model: mujoco.MjModel, data: mujoco.MjData, joints: list[dict[str, object]], q: np.ndarray, clearance: float) -> dict[str, object]:
    base_z = set_pose(model, data, joints, q, clearance)
    com = total_com(model, data)

    xs: list[float] = []
    ys: list[float] = []
    lows: list[float] = []
    normal_penalty = 0.0
    centers = []
    for name in FOOT_GEOMS:
        geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name)
        center = np.asarray(data.geom_xpos[geom_id])
        extents = geom_extents(model, data, geom_id)
        xmat = np.asarray(data.geom_xmat[geom_id]).reshape(3, 3)
        foot_z_axis = xmat[:, 2]
        normal_penalty += float((1.0 - abs(foot_z_axis[2])) ** 2)
        xs.extend([float(center[0] - extents[0]), float(center[0] + extents[0])])
        ys.extend([float(center[1] - extents[1]), float(center[1] + extents[1])])
        lows.append(float(center[2] - extents[2]))
        centers.append(center)

    aabb = {"x_min": min(xs), "x_max": max(xs), "y_min": min(ys), "y_max": max(ys)}
    margins = np.array([com[0] - aabb["x_min"], aabb["x_max"] - com[0], com[1] - aabb["y_min"], aabb["y_max"] - com[1]], dtype=np.float64)
    support_center = np.array([(aabb["x_min"] + aabb["x_max"]) * 0.5, (aabb["y_min"] + aabb["y_max"]) * 0.5], dtype=np.float64)
    support_half = np.array([(aabb["x_max"] - aabb["x_min"]) * 0.5, (aabb["y_max"] - aabb["y_min"]) * 0.5], dtype=np.float64)
    normalized_com_error = (com[:2] - support_center) / np.maximum(support_half, 1e-6)

    foot_height_diff = abs(lows[0] - lows[1])
    foot_y_symmetry = abs(float(centers[0][1] - centers[1][1]))
    outside_penalty = float(np.square(np.minimum(margins, 0.0)).sum())
    center_penalty = float(normalized_com_error.dot(normalized_com_error))
    joint_penalty = float(q.dot(q))
    base_penalty = 0.0 if 0.0 <= base_z <= 0.8 else float((base_z - np.clip(base_z, 0.0, 0.8)) ** 2)

    cost = (
        10000.0 * outside_penalty
        + 25.0 * center_penalty
        + 5000.0 * foot_height_diff * foot_height_diff
        + 100.0 * normal_penalty
        + 0.25 * joint_penalty
        + 200.0 * foot_y_symmetry * foot_y_symmetry
        + 1000.0 * base_penalty
    )
    return {
        "cost": float(cost),
        "base_z": base_z,
        "com_world": [float(v) for v in com],
        "support_aabb": aabb,
        "support_margins": [float(v) for v in margins],
        "foot_low_z": [float(v) for v in lows],
        "foot_height_diff": float(foot_height_diff),
        "normal_penalty": float(normal_penalty),
        "center_penalty": float(center_penalty),
        "joint_norm": float(np.linalg.norm(q)),
    }


def sample_candidates(rng: np.random.Generator, mean: np.ndarray, sigma: np.ndarray, count: int, low: np.ndarray, high: np.ndarray) -> np.ndarray:
    q = rng.normal(mean, sigma, size=(count, mean.size))
    return np.clip(q, low, high)


def main() -> int:
    parser = argparse.ArgumentParser(description="Search a quasi-static standing pose candidate using foot and COM objectives.")
    parser.add_argument("--model", type=Path, default=Path("envs/robots/urdf_f_link/URDF_F_link_user_size_mass_inertia_direct_nobase_contact.xml"))
    parser.add_argument("--samples", type=int, default=40000)
    parser.add_argument("--elite", type=int, default=256)
    parser.add_argument("--iterations", type=int, default=8)
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--limit", type=float, default=0.8)
    parser.add_argument("--clearance", type=float, default=0.005)
    parser.add_argument("--out", type=Path, default=Path("configs/quasistatic_standing_pose_inertia_direct_nobase.json"))
    parser.add_argument("--doc-dir", type=Path, default=Path("docs/hardware_validation"))
    args = parser.parse_args()

    model_path = args.model.resolve()
    model = mujoco.MjModel.from_xml_path(str(model_path))
    data = mujoco.MjData(model)
    joints = hinge_joints(model)
    low = np.array([max(float(joint["range"][0]), -args.limit) for joint in joints], dtype=np.float64)
    high = np.array([min(float(joint["range"][1]), args.limit) for joint in joints], dtype=np.float64)

    rng = np.random.default_rng(args.seed)
    mean = np.zeros(len(joints), dtype=np.float64)
    sigma = np.full(len(joints), args.limit * 0.5, dtype=np.float64)
    best_result: dict[str, object] | None = None
    best_q: np.ndarray | None = None

    history = []
    per_iter = max(args.samples // max(args.iterations, 1), args.elite)
    for iteration in range(args.iterations):
        candidates = sample_candidates(rng, mean, sigma, per_iter, low, high)
        if iteration == 0:
            candidates[0, :] = 0.0
        scored = []
        for q in candidates:
            result = evaluate(model, data, joints, q, args.clearance)
            scored.append((float(result["cost"]), q.copy(), result))
            if best_result is None or float(result["cost"]) < float(best_result["cost"]):
                best_result = result
                best_q = q.copy()
        scored.sort(key=lambda item: item[0])
        elites = np.array([item[1] for item in scored[: args.elite]], dtype=np.float64)
        mean = elites.mean(axis=0)
        sigma = np.maximum(elites.std(axis=0), 0.03)
        history.append({"iteration": iteration, "best_cost": scored[0][0], "global_best_cost": float(best_result["cost"])})

    if best_result is None or best_q is None:
        raise RuntimeError("No candidate evaluated.")

    joint_targets = {str(joint["name"]): float(value) for joint, value in zip(joints, best_q)}
    payload = {
        "model_path": str(model_path),
        "search": {
            "samples": args.samples,
            "per_iteration": per_iter,
            "elite": args.elite,
            "iterations": args.iterations,
            "seed": args.seed,
            "joint_abs_limit": args.limit,
            "clearance": args.clearance,
            "history": history,
        },
        "base_z": best_result["base_z"],
        "cost": best_result["cost"],
        "joint_targets": joint_targets,
        "com_world": best_result["com_world"],
        "support_aabb": best_result["support_aabb"],
        "support_margins": best_result["support_margins"],
        "foot_low_z": best_result["foot_low_z"],
        "foot_height_diff": best_result["foot_height_diff"],
        "normal_penalty": best_result["normal_penalty"],
        "center_penalty": best_result["center_penalty"],
        "joint_norm": best_result["joint_norm"],
    }

    out = args.out.resolve()
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")

    doc_dir = args.doc_dir.resolve()
    doc_dir.mkdir(parents=True, exist_ok=True)
    md_path = doc_dir / "53_quasistatic_standing_pose_search.md"
    lines = [
        "# Quasi-static Standing Pose Search",
        "",
        "## 목적",
        "",
        "neutral `0 rad` pose에서 standing이 실패했기 때문에, COM이 support 중심에 가깝고 양발 높이/수평성이 나은 준정적 standing 후보를 탐색한다.",
        "",
        "## 실행 명령",
        "",
        "```bash",
        "python3 scripts/search_quasistatic_standing_pose.py",
        "```",
        "",
        "## 결과",
        "",
        f"- 모델: `{model_path}`",
        f"- 후보 pose: `{out}`",
        f"- cost: `{float(best_result['cost']):.6f}`",
        f"- base z: `{float(best_result['base_z']):.6f}`",
        f"- COM world: `{best_result['com_world']}`",
        f"- support margins: `{best_result['support_margins']}`",
        f"- foot low z: `{best_result['foot_low_z']}`",
        f"- foot height diff: `{float(best_result['foot_height_diff']):.9f}`",
        f"- foot normal penalty: `{float(best_result['normal_penalty']):.9f}`",
        f"- COM center penalty: `{float(best_result['center_penalty']):.9f}`",
        "",
        "## Joint Targets",
        "",
    ]
    lines.extend(f"- `{name}`: `{value:.6f}` rad" for name, value in joint_targets.items())
    lines.extend(
        [
            "",
            "## 판단",
            "",
            "이 pose는 동역학적으로 검증된 standing이 아니라, 다음 PD probe에 넣을 준정적 후보이다. 이 후보에서도 실패하면 단순 pose 문제가 아니라 controller/contact/inertial frame 문제가 남아 있다고 본다.",
            "",
        ]
    )
    md_path.write_text("\n".join(lines), encoding="utf-8")

    print(f"Wrote {out}")
    print(f"Wrote {md_path}")
    print(json.dumps({"cost": best_result["cost"], "base_z": best_result["base_z"], "joint_norm": best_result["joint_norm"]}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
