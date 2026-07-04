from __future__ import annotations

import argparse
import json
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


def infer_base_z(model: mujoco.MjModel, data: mujoco.MjData, clearance: float) -> float:
    data.qpos[:] = model.qpos0
    data.qvel[:] = 0.0
    data.qpos[2] = 0.0
    data.qpos[3:7] = np.array([1.0, 0.0, 0.0, 0.0])
    mujoco.mj_forward(model, data)
    floor_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "floor")
    lows = [
        geom_low_z(model, data, geom_id)
        for geom_id in range(model.ngeom)
        if geom_id != floor_id and model.geom_contype[geom_id] != 0
    ]
    return clearance - min(lows)


def total_com(model: mujoco.MjModel, data: mujoco.MjData) -> np.ndarray:
    mass = np.asarray(model.body_mass)
    return (np.asarray(data.xipos) * mass[:, None]).sum(axis=0) / mass.sum()


def foot_support_aabb(model: mujoco.MjModel, data: mujoco.MjData) -> dict[str, float]:
    xs: list[float] = []
    ys: list[float] = []
    for name in ("foot_L_1_sole_collision", "foot_R_v1_1_sole_collision"):
        geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name)
        if geom_id < 0:
            continue
        center = np.asarray(data.geom_xpos[geom_id])
        size = np.asarray(model.geom_size[geom_id])
        xs.extend([float(center[0] - size[0]), float(center[0] + size[0])])
        ys.extend([float(center[1] - size[1]), float(center[1] + size[1])])
    if not xs or not ys:
        return {"x_min": 0.0, "x_max": 0.0, "y_min": 0.0, "y_max": 0.0}
    return {"x_min": min(xs), "x_max": max(xs), "y_min": min(ys), "y_max": max(ys)}


def margin_to_aabb(point: np.ndarray, aabb: dict[str, float]) -> dict[str, float | bool]:
    x, y = float(point[0]), float(point[1])
    return {
        "inside_x": aabb["x_min"] <= x <= aabb["x_max"],
        "inside_y": aabb["y_min"] <= y <= aabb["y_max"],
        "x_margin_min": x - aabb["x_min"],
        "x_margin_max": aabb["x_max"] - x,
        "y_margin_min": y - aabb["y_min"],
        "y_margin_max": aabb["y_max"] - y,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description="Analyze neutral standing COM relative to simplified foot support.")
    parser.add_argument("--model", type=Path, default=Path("envs/robots/urdf_f/URDF_F_contact.xml"))
    parser.add_argument("--clearance", type=float, default=0.005)
    parser.add_argument("--doc-dir", type=Path, default=Path("docs/hardware_validation"))
    args = parser.parse_args()

    model_path = args.model.resolve()
    doc_dir = args.doc_dir.resolve()
    doc_dir.mkdir(parents=True, exist_ok=True)

    model = mujoco.MjModel.from_xml_path(str(model_path))
    data = mujoco.MjData(model)
    base_z = infer_base_z(model, data, args.clearance)
    data.qpos[:] = model.qpos0
    data.qvel[:] = 0.0
    data.qpos[0:3] = np.array([0.0, 0.0, base_z])
    data.qpos[3:7] = np.array([1.0, 0.0, 0.0, 0.0])
    mujoco.mj_forward(model, data)

    com = total_com(model, data)
    aabb = foot_support_aabb(model, data)
    margin = margin_to_aabb(com, aabb)
    foot_records = []
    for name in ("foot_L_1_sole_collision", "foot_R_v1_1_sole_collision"):
        geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name)
        if geom_id >= 0:
            foot_records.append(
                {
                    "name": name,
                    "world_pos": [float(v) for v in data.geom_xpos[geom_id]],
                    "halfsize": [float(v) for v in model.geom_size[geom_id]],
                }
            )

    payload = {
        "model_path": str(model_path),
        "base_z": base_z,
        "total_mass_kg": float(model.body_mass.sum()),
        "com_world": [float(v) for v in com],
        "support_aabb": aabb,
        "support_margin": margin,
        "foot_collision_geoms": foot_records,
    }

    json_path = doc_dir / "standing_geometry_analysis.json"
    json_path.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    md_path = doc_dir / "08_standing_geometry_analysis.md"
    md_path.write_text(
        "\n".join(
            [
                "# Standing Geometry Analysis",
                "",
                "## Purpose",
                "",
                "Contact 단순화 후에도 neutral PD standing이 실패했기 때문에, 초기 자세의 전체 COM이 발 지지 영역 안에 있는지 확인한다.",
                "",
                "## Command",
                "",
                "```bash",
                "python3 scripts/analyze_standing_geometry.py",
                "```",
                "",
                "## Summary",
                "",
                f"- Model: `{model_path}`",
                f"- Initial base z: `{base_z:.6f}`",
                f"- Total mass: `{model.body_mass.sum():.6f} kg`",
                f"- COM world: `{payload['com_world']}`",
                f"- Support AABB: `{aabb}`",
                f"- COM inside support x: `{margin['inside_x']}`",
                f"- COM inside support y: `{margin['inside_y']}`",
                f"- Support margins: `{margin}`",
                "",
                "## Foot Collision Geoms",
                "",
                *[
                    f"- `{record['name']}` world_pos=`{record['world_pos']}` halfsize=`{record['halfsize']}`"
                    for record in foot_records
                ],
                "",
                "## Interpretation",
                "",
                "Neutral joint pose is not a verified standing pose. If the COM projection is outside the simplified foot support area, joint-space PD cannot reliably produce standing. The next step is to define or search for a standing pose that places both feet on the floor and the COM projection inside the support polygon.",
                "",
            ]
        ),
        encoding="utf-8",
    )

    print(f"Wrote {json_path}")
    print(f"Wrote {md_path}")
    print(json.dumps(payload, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
