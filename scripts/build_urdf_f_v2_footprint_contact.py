#!/usr/bin/env python3
"""Add STL-fitted foot contact pads to the urdf_f_v2 named MJCF.

The named model produced by apply_joint_mapping.py carries visual-only foot
meshes (contype=0). The RL env (`UrdfFEnv`) senses ground contact through box
geoms named `<foot_body>_sole_pad_*`. This script measures each foot mesh's
axis-aligned footprint in its body frame and tiles it with a 2x2 grid of
collision pads at the sole, mirroring the old model's
`URDF_F_link_virtual_stl_footprint_contact.xml`.

Usage:
  python3 scripts/build_urdf_f_v2_footprint_contact.py \
      --source envs/robots/urdf_f_v2/URDF_v2_named.xml \
      --out envs/robots/urdf_f_v2/URDF_F_v2_footprint_contact.xml
"""
from __future__ import annotations

import argparse
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco
import numpy as np

MESH = mujoco.mjtGeom.mjGEOM_MESH


def foot_footprint(model: mujoco.MjModel, body_name: str) -> tuple[np.ndarray, np.ndarray]:
    """Return (lo, hi) corner of the foot mesh AABB in the body frame."""
    bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body_name)
    if bid < 0:
        raise ValueError(f"body {body_name!r} not found")
    for g in range(model.ngeom):
        if model.geom_bodyid[g] != bid or model.geom_type[g] != MESH:
            continue
        center = model.geom_aabb[g][:3]
        half = model.geom_aabb[g][3:]
        gpos = model.geom_pos[g].copy()
        rot = np.zeros(9)
        mujoco.mju_quat2Mat(rot, model.geom_quat[g])
        rot = rot.reshape(3, 3)
        corners = []
        for sx in (-1, 1):
            for sy in (-1, 1):
                for sz in (-1, 1):
                    local = center + np.array([sx, sy, sz]) * half
                    corners.append(gpos + rot @ local)
        corners = np.array(corners)
        return corners.min(0), corners.max(0)
    raise ValueError(f"no mesh geom on body {body_name!r}")


def apply_joint_dynamics(root: ET.Element) -> int:
    """Set actuator reflected inertia + damping on hinge joints.

    The URDF carries no joint damping/armature, so build_mujoco_mjcf.py produces
    frictionless, inertia-free joints that make PD control numerically explosive
    (the base gets launched). Mirror the old model's values: geared motors add
    large reflected inertia (armature), and ankles are stiffer than hip/knee.
    """
    applied = 0
    for joint in root.iter("joint"):
        name = joint.attrib.get("name", "")
        if joint.attrib.get("type") == "free" or not name:
            continue
        if "ankle" in name:
            armature, damping = "0.05", "0.4"
        else:
            armature, damping = "0.02", "0.2"
        joint.attrib["armature"] = armature
        joint.attrib["damping"] = damping
        joint.attrib["frictionloss"] = "0.02"
        joint.attrib.setdefault("actuatorfrcrange", "-100 100")
        applied += 1
    return applied


def make_pads(body_name: str, lo: np.ndarray, hi: np.ndarray, thickness: float) -> list[ET.Element]:
    """Tile the footprint x-y with a 2x2 grid of box pads seated at the sole."""
    x_half = (hi[0] - lo[0]) / 4.0
    y_half = (hi[1] - lo[1]) / 4.0
    x_c = [lo[0] + x_half, hi[0] - x_half]
    y_c = [lo[1] + y_half, hi[1] - y_half]
    z_half = thickness / 2.0
    z_c = lo[2] + z_half  # pad bottom flush with mesh sole
    pads = []
    for xi, xc in zip(("neg", "pos"), x_c):
        for yi, yc in zip(("rear", "front"), y_c):
            pads.append(
                ET.Element(
                    "geom",
                    {
                        "name": f"{body_name}_sole_pad_{yi}_{xi}",
                        "type": "box",
                        "pos": f"{xc:.7g} {yc:.7g} {z_c:.7g}",
                        "size": f"{x_half:.7g} {y_half:.7g} {z_half:.7g}",
                        "rgba": "0.1 0.8 0.2 0.35",
                        "friction": "1.0 0.02 0.001",
                        "condim": "3",
                    },
                )
            )
    return pads


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--source", type=Path, default=Path("envs/robots/urdf_f_v2/URDF_v2_named.xml"))
    ap.add_argument("--out", type=Path, default=Path("envs/robots/urdf_f_v2/URDF_F_v2_footprint_contact.xml"))
    ap.add_argument("--feet", nargs="+", default=["foot_L_1", "foot_R_1"])
    ap.add_argument("--thickness", type=float, default=0.03)
    args = ap.parse_args()

    model = mujoco.MjModel.from_xml_path(str(args.source.resolve()))
    footprints = {f: foot_footprint(model, f) for f in args.feet}

    tree = ET.parse(args.source)
    root = tree.getroot()
    n_dyn = apply_joint_dynamics(root)
    print(f"applied joint dynamics (armature/damping/frictionloss) to {n_dyn} hinge joints")
    for foot in args.feet:
        body = root.find(f".//body[@name='{foot}']")
        if body is None:
            raise ValueError(f"body {foot!r} not in XML")
        lo, hi = footprints[foot]
        for pad in make_pads(foot, lo, hi, args.thickness):
            body.append(pad)
        print(f"{foot}: footprint x[{lo[0]:+.4f},{hi[0]:+.4f}] "
              f"y[{lo[1]:+.4f},{hi[1]:+.4f}] sole_z={lo[2]:+.4f} -> 4 pads")

    ET.indent(root, space="  ")
    args.out.parent.mkdir(parents=True, exist_ok=True)
    tree.write(args.out, encoding="utf-8", xml_declaration=False)

    compiled = mujoco.MjModel.from_xml_path(str(args.out.resolve()))
    n_pads = sum(
        1
        for g in range(compiled.ngeom)
        if (mujoco.mj_id2name(compiled, mujoco.mjtObj.mjOBJ_GEOM, g) or "").find("_sole_pad_") >= 0
    )
    print(f"Wrote {args.out}")
    print(f"Compiled nq={compiled.nq} nv={compiled.nv} nu={compiled.nu} "
          f"ngeom={compiled.ngeom} sole_pads={n_pads}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
