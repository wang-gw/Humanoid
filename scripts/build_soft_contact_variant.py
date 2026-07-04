from __future__ import annotations

import argparse
import json
import xml.etree.ElementTree as ET
from pathlib import Path


CONTACT_GEOMS = {"floor", "foot_L_1_sole_collision", "foot_R_v1_1_sole_collision"}


def main() -> int:
    parser = argparse.ArgumentParser(description="Create an MJCF variant with softer floor/sole contact only.")
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--solref", default="0.02 1")
    parser.add_argument("--solimp", default="0.9 0.95 0.001")
    parser.add_argument("--friction", default="1.2 0.03 0.003")
    parser.add_argument("--condim", default="3")
    parser.add_argument("--report", type=Path, default=Path("docs/hardware_validation/soft_contact_variant_report.json"))
    args = parser.parse_args()

    input_path = args.input.resolve()
    output_path = args.output.resolve()
    report_path = args.report.resolve()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    report_path.parent.mkdir(parents=True, exist_ok=True)

    tree = ET.parse(input_path)
    root = tree.getroot()
    original_model = root.attrib.get("model", "model")
    root.set("model", f"{original_model}_soft_contact")

    edited = []
    for geom in root.iter("geom"):
        name = geom.attrib.get("name")
        if name not in CONTACT_GEOMS:
            continue
        before = dict(geom.attrib)
        geom.set("solref", args.solref)
        geom.set("solimp", args.solimp)
        geom.set("friction", args.friction)
        geom.set("condim", args.condim)
        edited.append({"name": name, "before": before, "after": dict(geom.attrib)})

    if len(edited) != len(CONTACT_GEOMS):
        found = {item["name"] for item in edited}
        missing = sorted(CONTACT_GEOMS - found)
        raise ValueError(f"Missing expected contact geoms: {missing}")

    ET.indent(tree, space="  ")
    tree.write(output_path, encoding="unicode", xml_declaration=False)
    output_path.write_text(output_path.read_text(encoding="utf-8") + "\n", encoding="utf-8")

    report = {
        "input": str(input_path),
        "output": str(output_path),
        "solref": args.solref,
        "solimp": args.solimp,
        "friction": args.friction,
        "condim": args.condim,
        "edited": edited,
    }
    report_path.write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
