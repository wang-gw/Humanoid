from __future__ import annotations

import argparse
import json
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco


def main() -> int:
    parser = argparse.ArgumentParser(description="Create a new MJCF variant with updated motor ctrlrange/forcerange.")
    parser.add_argument("--source", type=Path, default=Path("envs/robots/urdf_f_link/URDF_F_link_user_size_mass_contact.xml"))
    parser.add_argument("--limit", type=float, required=True)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--doc-dir", type=Path, default=Path("docs/hardware_validation"))
    args = parser.parse_args()

    source = args.source.resolve()
    out = args.out.resolve()
    doc_dir = args.doc_dir.resolve()
    doc_dir.mkdir(parents=True, exist_ok=True)

    tree = ET.parse(source)
    root = tree.getroot()
    root.attrib["model"] = out.stem
    limit = float(args.limit)
    records = []
    for motor in root.findall("./actuator/motor"):
        old_ctrl = motor.attrib.get("ctrlrange", "")
        old_force = motor.attrib.get("forcerange", "")
        new_range = f"{-limit:g} {limit:g}"
        motor.attrib["ctrlrange"] = new_range
        motor.attrib["forcerange"] = new_range
        records.append(
            {
                "motor": motor.attrib.get("name", ""),
                "joint": motor.attrib.get("joint", ""),
                "old_ctrlrange": old_ctrl,
                "old_forcerange": old_force,
                "new_range": new_range,
            }
        )
    joint_records = []
    motor_joints = {record["joint"] for record in records if record["joint"]}
    for joint in root.iter("joint"):
        joint_name = joint.attrib.get("name", "")
        if joint_name not in motor_joints:
            continue
        old_actuator_force = joint.attrib.get("actuatorfrcrange", "")
        new_range = f"{-limit:g} {limit:g}"
        joint.attrib["actuatorfrcrange"] = new_range
        joint_records.append(
            {
                "joint": joint_name,
                "old_actuatorfrcrange": old_actuator_force,
                "new_actuatorfrcrange": new_range,
            }
        )

    ET.indent(root, space="  ")
    tree.write(out, encoding="utf-8", xml_declaration=False)
    compiled = mujoco.MjModel.from_xml_path(str(out))
    payload = {
        "source": str(source),
        "out": str(out),
        "limit_nm": limit,
        "motors": records,
        "joints": joint_records,
        "compiled": {
            "nq": int(compiled.nq),
            "nv": int(compiled.nv),
            "nu": int(compiled.nu),
            "njnt": int(compiled.njnt),
            "ngeom": int(compiled.ngeom),
        },
    }
    report_path = doc_dir / f"actuator_limit_{limit:g}_report.json"
    report_path.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(f"Wrote {out}")
    print(f"Wrote {report_path}")
    print(json.dumps(payload["compiled"], indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
