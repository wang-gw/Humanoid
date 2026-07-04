from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import mujoco
import numpy as np


FOOT_GEOMS = ("foot_L_1_sole_collision", "foot_R_v1_1_sole_collision")


def load_pose(model: mujoco.MjModel, data: mujoco.MjData, pose_path: Path | None) -> None:
    data.qpos[:] = model.qpos0
    data.qvel[:] = 0.0
    if pose_path is None:
        mujoco.mj_forward(model, data)
        return

    payload = json.loads(pose_path.read_text(encoding="utf-8"))
    if "base_z" in payload:
        data.qpos[2] = float(payload["base_z"])
    targets = payload.get("joint_targets", {})
    for joint_name, value in targets.items():
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
        if joint_id < 0:
            continue
        qposadr = int(model.jnt_qposadr[joint_id])
        data.qpos[qposadr] = float(value)
    mujoco.mj_forward(model, data)


def total_com(model: mujoco.MjModel, data: mujoco.MjData) -> np.ndarray:
    masses = model.body_mass[:, None]
    return np.sum(masses * data.xipos, axis=0) / np.sum(model.body_mass)


def box_corners_2d(center: np.ndarray, xmat: np.ndarray, half: np.ndarray, axes: tuple[int, int]) -> np.ndarray:
    world_axes = [xmat[:, axes[0]], xmat[:, axes[1]]]
    corners = []
    for sx in (-1.0, 1.0):
        for sy in (-1.0, 1.0):
            point = center + sx * half[axes[0]] * world_axes[0] + sy * half[axes[1]] * world_axes[1]
            corners.append(point)
    corners = np.array(corners)
    centroid = corners[:, axes].mean(axis=0)
    angles = np.arctan2(corners[:, axes[1]] - centroid[1], corners[:, axes[0]] - centroid[0])
    return corners[np.argsort(angles)]


def convex_hull(points: np.ndarray) -> np.ndarray:
    pts = sorted({(float(point[0]), float(point[1])) for point in points})
    if len(pts) <= 1:
        return np.array(pts, dtype=float)

    def cross(o: tuple[float, float], a: tuple[float, float], b: tuple[float, float]) -> float:
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    lower: list[tuple[float, float]] = []
    for point in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], point) <= 0:
            lower.pop()
        lower.append(point)

    upper: list[tuple[float, float]] = []
    for point in reversed(pts):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], point) <= 0:
            upper.pop()
        upper.append(point)

    return np.array(lower[:-1] + upper[:-1], dtype=float)


def foot_records(model: mujoco.MjModel, data: mujoco.MjData) -> list[dict[str, object]]:
    records = []
    for geom_name in FOOT_GEOMS:
        geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, geom_name)
        if geom_id < 0:
            continue
        records.append(
            {
                "name": geom_name,
                "center": np.array(data.geom_xpos[geom_id], dtype=float),
                "xmat": np.array(data.geom_xmat[geom_id], dtype=float).reshape(3, 3),
                "half": np.array(model.geom_size[geom_id], dtype=float),
            }
        )
    return records


def set_equal_2d(ax: plt.Axes, xs: list[float], ys: list[float], pad: float = 0.04) -> None:
    xmin, xmax = min(xs), max(xs)
    ymin, ymax = min(ys), max(ys)
    span = max(xmax - xmin, ymax - ymin, 1e-3)
    cx, cy = (xmin + xmax) * 0.5, (ymin + ymax) * 0.5
    ax.set_xlim(cx - span * 0.5 - pad, cx + span * 0.5 + pad)
    ax.set_ylim(cy - span * 0.5 - pad, cy + span * 0.5 + pad)
    ax.set_aspect("equal", adjustable="box")


def plot_projection(
    records: list[dict[str, object]],
    com: np.ndarray,
    axes: tuple[int, int],
    labels: tuple[str, str],
    title: str,
    out: Path,
) -> None:
    fig, ax = plt.subplots(figsize=(8, 7), dpi=160)
    colors = {"foot_L_1_sole_collision": "#277da1", "foot_R_v1_1_sole_collision": "#f3722c"}
    xs: list[float] = [float(com[axes[0]])]
    ys: list[float] = [float(com[axes[1]])]
    all_projected_corners: list[np.ndarray] = []

    for record in records:
        name = str(record["name"])
        center = record["center"]
        xmat = record["xmat"]
        half = record["half"]
        corners = box_corners_2d(center, xmat, half, axes)
        closed = np.vstack([corners, corners[0]])
        ax.fill(
            closed[:, axes[0]],
            closed[:, axes[1]],
            color=colors.get(name, "#43aa8b"),
            alpha=0.28,
            edgecolor=colors.get(name, "#43aa8b"),
            linewidth=2.0,
            label=name,
        )
        ax.scatter([center[axes[0]]], [center[axes[1]]], color=colors.get(name, "#43aa8b"), s=28)
        ax.text(center[axes[0]], center[axes[1]], name.replace("_sole_collision", ""), fontsize=8)
        xs.extend([float(v) for v in closed[:, axes[0]]])
        ys.extend([float(v) for v in closed[:, axes[1]]])
        all_projected_corners.extend(corners[:, axes])

    if axes == (0, 1) and len(all_projected_corners) >= 3:
        hull = convex_hull(np.array(all_projected_corners))
        hull_closed = np.vstack([hull, hull[0]])
        ax.fill(
            hull_closed[:, 0],
            hull_closed[:, 1],
            color="#90be6d",
            alpha=0.14,
            edgecolor="#4d908e",
            linewidth=2.0,
            linestyle="--",
            label="double-support convex hull",
        )
        xs.extend([float(v) for v in hull_closed[:, 0]])
        ys.extend([float(v) for v in hull_closed[:, 1]])

    ax.scatter([com[axes[0]]], [com[axes[1]]], marker="x", color="#d00000", s=120, linewidths=3, label="COM projection")
    ax.axhline(0.0, color="#888", linewidth=0.8, alpha=0.5)
    ax.axvline(0.0, color="#888", linewidth=0.8, alpha=0.5)
    set_equal_2d(ax, xs, ys)
    ax.set_xlabel(f"MuJoCo {labels[0]} (m)")
    ax.set_ylabel(f"MuJoCo {labels[1]} (m)")
    ax.set_title(title)
    ax.grid(True, linewidth=0.5, alpha=0.35)
    ax.legend(loc="best", fontsize=8)
    fig.tight_layout()
    fig.savefig(out)
    plt.close(fig)


def render_scene(model: mujoco.MjModel, data: mujoco.MjData, out_dir: Path) -> list[str]:
    outputs: list[str] = []
    try:
        renderer = mujoco.Renderer(model, height=480, width=640)
    except Exception as exc:
        return [f"renderer unavailable: {exc}"]

    cameras = {
        "render_iso.png": dict(distance=1.25, azimuth=135.0, elevation=-20.0),
        "render_front.png": dict(distance=1.10, azimuth=180.0, elevation=-5.0),
        "render_side.png": dict(distance=1.10, azimuth=90.0, elevation=-5.0),
        "render_top.png": dict(distance=1.25, azimuth=90.0, elevation=-89.0),
    }
    com = total_com(model, data)
    for filename, spec in cameras.items():
        camera = mujoco.MjvCamera()
        camera.type = mujoco.mjtCamera.mjCAMERA_FREE
        camera.lookat[:] = com
        camera.distance = float(spec["distance"])
        camera.azimuth = float(spec["azimuth"])
        camera.elevation = float(spec["elevation"])
        renderer.update_scene(data, camera=camera)
        pixels = renderer.render()
        path = out_dir / filename
        plt.imsave(path, pixels)
        outputs.append(str(path))
    renderer.close()
    return outputs


def main() -> int:
    parser = argparse.ArgumentParser(description="Create visual checks for foot contact pads and COM projection.")
    parser.add_argument("--model", type=Path, required=True)
    parser.add_argument("--pose-json", type=Path)
    parser.add_argument("--out-dir", type=Path, default=Path("outputs/analysis/contact_visualization"))
    parser.add_argument("--doc-dir", type=Path, default=Path("docs/hardware_validation"))
    args = parser.parse_args()

    out_dir = args.out_dir.resolve()
    doc_dir = args.doc_dir.resolve()
    out_dir.mkdir(parents=True, exist_ok=True)
    doc_dir.mkdir(parents=True, exist_ok=True)

    model = mujoco.MjModel.from_xml_path(str(args.model.resolve()))
    data = mujoco.MjData(model)
    load_pose(model, data, args.pose_json.resolve() if args.pose_json else None)
    records = foot_records(model, data)
    com = total_com(model, data)

    top_path = out_dir / "contact_top_xy.png"
    side_path = out_dir / "contact_side_xz.png"
    rear_path = out_dir / "contact_rear_yz.png"
    plot_projection(records, com, (0, 1), ("X forward", "Y lateral"), "Top view: foot contact pads and COM", top_path)
    plot_projection(records, com, (0, 2), ("X forward", "Z up"), "Side view: contact height and COM", side_path)
    plot_projection(records, com, (1, 2), ("Y lateral", "Z up"), "Rear view: lateral support and COM", rear_path)

    render_outputs = render_scene(model, data, out_dir)
    payload = {
        "model": str(args.model.resolve()),
        "pose_json": str(args.pose_json.resolve()) if args.pose_json else None,
        "com_world": [float(v) for v in com],
        "foot_geoms": [
            {
                "name": str(record["name"]),
                "center": [float(v) for v in record["center"]],
                "halfsize": [float(v) for v in record["half"]],
            }
            for record in records
        ],
        "images": [str(top_path), str(side_path), str(rear_path)] + render_outputs,
    }
    json_path = out_dir / "visualization_summary.json"
    json_path.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")

    print(json.dumps(payload, indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
