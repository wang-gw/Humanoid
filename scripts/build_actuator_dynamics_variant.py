from __future__ import annotations

import argparse
import json
import xml.etree.ElementTree as ET
from pathlib import Path


def classify_joint(name: str) -> str:
    if "ankle" in name:
        return "ankle"
    if "knee" in name:
        return "knee"
    if "hip" in name:
        return "hip"
    return "other"


def main() -> int:
    parser = argparse.ArgumentParser(description="Create an MJCF variant with experimental joint armature/damping/frictionloss.")
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--hip-armature", type=float, default=0.02)
    parser.add_argument("--hip-damping", type=float, default=0.2)
    parser.add_argument("--knee-armature", type=float, default=0.02)
    parser.add_argument("--knee-damping", type=float, default=0.2)
    parser.add_argument("--ankle-armature", type=float, default=0.05)
    parser.add_argument("--ankle-damping", type=float, default=0.4)
    parser.add_argument("--frictionloss", type=float, default=0.02)
    parser.add_argument("--report", type=Path, default=Path("docs/hardware_validation/actuator_dynamics_variant_report.json"))
    args = parser.parse_args()

    input_path = args.input.resolve()
    output_path = args.output.resolve()
    report_path = args.report.resolve()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    report_path.parent.mkdir(parents=True, exist_ok=True)

    tree = ET.parse(input_path)
    root = tree.getroot()
    root.set("model", f"{root.attrib.get('model', 'model')}_actuator_dynamics")

    edited = []
    for joint in root.iter("joint"):
        if joint.attrib.get("type") not in (None, "hinge"):
            continue
        name = joint.attrib.get("name", "")
        if name == "floating_base":
            continue
        kind = classify_joint(name)
        before = dict(joint.attrib)
        if kind == "ankle":
            joint.set("armature", f"{args.ankle_armature:g}")
            joint.set("damping", f"{args.ankle_damping:g}")
        elif kind == "knee":
            joint.set("armature", f"{args.knee_armature:g}")
            joint.set("damping", f"{args.knee_damping:g}")
        else:
            joint.set("armature", f"{args.hip_armature:g}")
            joint.set("damping", f"{args.hip_damping:g}")
        joint.set("frictionloss", f"{args.frictionloss:g}")
        edited.append({"name": name, "kind": kind, "before": before, "after": dict(joint.attrib)})

    ET.indent(tree, space="  ")
    tree.write(output_path, encoding="unicode", xml_declaration=False)
    output_path.write_text(output_path.read_text(encoding="utf-8") + "\n", encoding="utf-8")

    report = {
        "input": str(input_path),
        "output": str(output_path),
        "hip_armature": args.hip_armature,
        "hip_damping": args.hip_damping,
        "knee_armature": args.knee_armature,
        "knee_damping": args.knee_damping,
        "ankle_armature": args.ankle_armature,
        "ankle_damping": args.ankle_damping,
        "frictionloss": args.frictionloss,
        "edited": edited,
    }
    report_path.write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
