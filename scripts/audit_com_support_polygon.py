from __future__ import annotations

import argparse
import json
from pathlib import Path

import mujoco
import numpy as np

from render_pd_standing import hinge_joint_info, load_targets
from sweep_stabilized_standing import initialize, total_com


def geom_name(model: mujoco.MjModel, geom_id: int) -> str:
    return mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, geom_id) or f"geom_{geom_id}"


def side_for_geom(name: str) -> str | None:
    if name == "foot_L_1_sole_collision" or name.startswith("foot_L_1_sole_pad_"):
        return "left"
    if name == "foot_R_v1_1_sole_collision" or name.startswith("foot_R_v1_1_sole_pad_"):
        return "right"
    return None


def box_corners(model: mujoco.MjModel, data: mujoco.MjData, geom_id: int) -> np.ndarray:
    center = np.asarray(data.geom_xpos[geom_id], dtype=np.float64)
    xmat = np.asarray(data.geom_xmat[geom_id], dtype=np.float64).reshape(3, 3)
    half = np.asarray(model.geom_size[geom_id], dtype=np.float64)
    corners = []
    for sx in (-1.0, 1.0):
        for sy in (-1.0, 1.0):
            for sz in (-1.0, 1.0):
                local = np.array([sx * half[0], sy * half[1], sz * half[2]], dtype=np.float64)
                corners.append(center + xmat @ local)
    return np.asarray(corners)


def convex_hull(points: np.ndarray) -> np.ndarray:
    pts = sorted({(float(p[0]), float(p[1])) for p in points})
    if len(pts) <= 1:
        return np.asarray(pts, dtype=np.float64)

    def cross(o: tuple[float, float], a: tuple[float, float], b: tuple[float, float]) -> float:
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    lower: list[tuple[float, float]] = []
    for p in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0.0:
            lower.pop()
        lower.append(p)

    upper: list[tuple[float, float]] = []
    for p in reversed(pts):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0.0:
            upper.pop()
        upper.append(p)

    return np.asarray(lower[:-1] + upper[:-1], dtype=np.float64)


def point_segment_distance(point: np.ndarray, a: np.ndarray, b: np.ndarray) -> float:
    ab = b - a
    denom = float(np.dot(ab, ab))
    if denom <= 1e-18:
        return float(np.linalg.norm(point - a))
    t = float(np.clip(np.dot(point - a, ab) / denom, 0.0, 1.0))
    projection = a + t * ab
    return float(np.linalg.norm(point - projection))


def polygon_area(poly: np.ndarray) -> float:
    if len(poly) < 3:
        return 0.0
    x = poly[:, 0]
    y = poly[:, 1]
    return float(0.5 * abs(np.dot(x, np.roll(y, -1)) - np.dot(y, np.roll(x, -1))))


def polygon_centroid(poly: np.ndarray) -> np.ndarray:
    if len(poly) == 0:
        return np.zeros(2, dtype=np.float64)
    if len(poly) < 3:
        return np.mean(poly, axis=0)
    signed_twice_area = 0.0
    cx = 0.0
    cy = 0.0
    for idx in range(len(poly)):
        x0, y0 = poly[idx]
        x1, y1 = poly[(idx + 1) % len(poly)]
        cross = x0 * y1 - x1 * y0
        signed_twice_area += cross
        cx += (x0 + x1) * cross
        cy += (y0 + y1) * cross
    if abs(signed_twice_area) <= 1e-18:
        return np.mean(poly, axis=0)
    return np.array([cx / (3.0 * signed_twice_area), cy / (3.0 * signed_twice_area)], dtype=np.float64)


def polygon_margin(point: np.ndarray, poly: np.ndarray) -> dict[str, float | bool]:
    if len(poly) < 3:
        return {"inside": False, "signed_margin": float("-inf"), "distance_to_edge": float("inf")}

    signs = []
    distances = []
    for idx in range(len(poly)):
        a = poly[idx]
        b = poly[(idx + 1) % len(poly)]
        edge = b - a
        rel = point - a
        cross = float(edge[0] * rel[1] - edge[1] * rel[0])
        signs.append(cross)
        distances.append(point_segment_distance(point, a, b))
    has_pos = any(v > 1e-9 for v in signs)
    has_neg = any(v < -1e-9 for v in signs)
    inside = not (has_pos and has_neg)
    distance = min(distances)
    return {
        "inside": inside,
        "signed_margin": float(distance if inside else -distance),
        "distance_to_edge": float(distance),
    }


def bounds_margin(point: np.ndarray, points: np.ndarray) -> dict[str, float | bool]:
    x_min = float(np.min(points[:, 0]))
    x_max = float(np.max(points[:, 0]))
    y_min = float(np.min(points[:, 1]))
    y_max = float(np.max(points[:, 1]))
    x = float(point[0])
    y = float(point[1])
    return {
        "x_min": x_min,
        "x_max": x_max,
        "y_min": y_min,
        "y_max": y_max,
        "inside_x": x_min <= x <= x_max,
        "inside_y": y_min <= y <= y_max,
        "x_min_margin": x - x_min,
        "x_max_margin": x_max - x,
        "y_min_margin": y - y_min,
        "y_max_margin": y_max - y,
        "min_axis_margin": min(x - x_min, x_max - x, y - y_min, y_max - y),
    }


def write_plot(out_path: Path, polygons: dict[str, np.ndarray], com_xy: np.ndarray) -> bool:
    try:
        import matplotlib.pyplot as plt
    except Exception:
        return False

    fig, ax = plt.subplots(figsize=(7.0, 5.0))
    colors = {"left": "#2563eb", "right": "#dc2626", "both": "#111827"}
    for side, poly in polygons.items():
        if len(poly) == 0:
            continue
        closed = np.vstack([poly, poly[0]])
        ax.plot(closed[:, 0], closed[:, 1], color=colors.get(side, "#6b7280"), linewidth=2, label=f"{side} hull")
        ax.fill(poly[:, 0], poly[:, 1], color=colors.get(side, "#6b7280"), alpha=0.10)
    ax.scatter([com_xy[0]], [com_xy[1]], color="#f59e0b", edgecolor="black", s=80, zorder=5, label="COM projection")
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("MuJoCo X (m)")
    ax.set_ylabel("MuJoCo Y (m)")
    ax.grid(True, alpha=0.25)
    ax.legend(loc="best")
    fig.tight_layout()
    fig.savefig(out_path, dpi=160)
    plt.close(fig)
    return True


def main() -> int:
    parser = argparse.ArgumentParser(description="Audit COM projection against multipoint foot support polygons.")
    parser.add_argument("--model", type=Path, required=True)
    parser.add_argument("--pose-json", type=Path, required=True)
    parser.add_argument("--out-dir", type=Path, required=True)
    args = parser.parse_args()

    out_dir = args.out_dir.resolve()
    out_dir.mkdir(parents=True, exist_ok=True)

    model = mujoco.MjModel.from_xml_path(str(args.model.resolve()))
    data = mujoco.MjData(model)
    joints = hinge_joint_info(model)
    base_z, target_q = load_targets(model, joints, args.pose_json.resolve())
    initialize(model, data, joints, base_z, target_q)
    mujoco.mj_forward(model, data)

    points_by_side: dict[str, list[np.ndarray]] = {"left": [], "right": []}
    geom_records = []
    for geom_id in range(model.ngeom):
        name = geom_name(model, geom_id)
        side = side_for_geom(name)
        if side is None:
            continue
        corners = box_corners(model, data, geom_id)
        points_by_side[side].append(corners[:, :2])
        geom_records.append(
            {
                "name": name,
                "side": side,
                "center": [float(v) for v in data.geom_xpos[geom_id]],
                "halfsize": [float(v) for v in model.geom_size[geom_id]],
                "lowest_z": float(np.min(corners[:, 2])),
                "highest_z": float(np.max(corners[:, 2])),
            }
        )

    side_points: dict[str, np.ndarray] = {}
    polygons: dict[str, np.ndarray] = {}
    for side, chunks in points_by_side.items():
        if chunks:
            side_points[side] = np.vstack(chunks)
            polygons[side] = convex_hull(side_points[side])
        else:
            side_points[side] = np.empty((0, 2), dtype=np.float64)
            polygons[side] = np.empty((0, 2), dtype=np.float64)

    both_points = np.vstack([points for points in side_points.values() if len(points)])
    polygons["both"] = convex_hull(both_points)
    com = total_com(model, data)
    com_xy = com[:2]

    support = {}
    for side, poly in polygons.items():
        points = both_points if side == "both" else side_points[side]
        support[side] = {
            "area_m2": polygon_area(poly),
            "centroid_xy": [float(v) for v in polygon_centroid(poly)],
            "hull_xy": [[float(v) for v in row] for row in poly],
            "bounds": bounds_margin(com_xy, points),
            "polygon_margin": polygon_margin(com_xy, poly),
        }

    plot_path = out_dir / "com_support_polygon.png"
    plotted = write_plot(plot_path, polygons, com_xy)
    payload = {
        "model": str(args.model.resolve()),
        "pose_json": str(args.pose_json.resolve()),
        "base_z": float(base_z),
        "total_mass": float(np.sum(model.body_mass)),
        "com_world": [float(v) for v in com],
        "com_projection_xy": [float(v) for v in com_xy],
        "initial_contacts": int(data.ncon),
        "geoms": geom_records,
        "support": support,
        "plot": str(plot_path) if plotted else None,
    }
    (out_dir / "com_support_polygon_audit.json").write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(json.dumps(payload, indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
