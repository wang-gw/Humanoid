from __future__ import annotations

import argparse
import re
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco


def safe_name(value: str) -> str:
    return re.sub(r"[^A-Za-z0-9_]+", "_", value).strip("_")


def ensure_floor(root: ET.Element) -> None:
    worldbody = root.find("worldbody")
    if worldbody is None:
        worldbody = ET.SubElement(root, "worldbody")
    for geom in worldbody.findall("geom"):
        if geom.attrib.get("name") == "floor":
            return
    floor = ET.Element(
        "geom",
        {
            "name": "floor",
            "type": "plane",
            "size": "5 5 0.05",
            "rgba": "0.45 0.45 0.45 1",
            "friction": "0.8 0.02 0.001",
        },
    )
    worldbody.insert(0, floor)


def collect_hinge_joints(root: ET.Element) -> list[str]:
    joints: list[str] = []
    for joint in root.iter("joint"):
        if joint.attrib.get("type") == "free":
            continue
        name = joint.attrib.get("name")
        if name:
            joints.append(name)
    return joints


def replace_actuators(root: ET.Element, joints: list[str], torque_limit: float) -> None:
    existing = root.find("actuator")
    if existing is not None:
        root.remove(existing)
    actuator = ET.SubElement(root, "actuator")
    for joint_name in joints:
        ET.SubElement(
            actuator,
            "motor",
            {
                "name": f"motor_{safe_name(joint_name)}",
                "joint": joint_name,
                "gear": "1",
                "ctrlrange": f"{-torque_limit:g} {torque_limit:g}",
                "forcerange": f"{-torque_limit:g} {torque_limit:g}",
            },
        )


def main() -> int:
    parser = argparse.ArgumentParser(description="Build an actuator-equipped MJCF from the prepared URDF.")
    parser.add_argument("--urdf", type=Path, default=Path("envs/robots/urdf_f/URDF_F_mujoco.urdf"))
    parser.add_argument("--out", type=Path, default=Path("envs/robots/urdf_f/URDF_F_mujoco.xml"))
    parser.add_argument("--torque-limit", type=float, default=100.0)
    args = parser.parse_args()

    urdf_path = args.urdf.resolve()
    out_path = args.out.resolve()
    out_path.parent.mkdir(parents=True, exist_ok=True)

    model = mujoco.MjModel.from_xml_path(str(urdf_path))
    mujoco.mj_saveLastXML(str(out_path), model)

    root = ET.parse(out_path).getroot()
    ensure_floor(root)
    joints = collect_hinge_joints(root)
    replace_actuators(root, joints, float(args.torque_limit))
    ET.indent(root, space="  ")
    ET.ElementTree(root).write(out_path, encoding="utf-8", xml_declaration=False)

    compiled = mujoco.MjModel.from_xml_path(str(out_path))
    print(f"Wrote {out_path}")
    print(f"Actuated joints: {len(joints)}")
    print(f"Compiled nq={compiled.nq} nv={compiled.nv} nu={compiled.nu} nbody={compiled.nbody}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
