from __future__ import annotations

import argparse
import json
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco


def parse_vec(value: str) -> list[float]:
    parts = [float(item) for item in value.split()]
    if len(parts) != 3:
        raise ValueError(f"Expected 3 values, got {value!r}")
    return parts


def fmt_vec(values: list[float]) -> str:
    return " ".join(f"{value:.8g}" for value in values)


def find_body(root: ET.Element, name: str) -> ET.Element:
    for body in root.iter("body"):
        if body.attrib.get("name") == name:
            return body
    raise ValueError(f"Body not found: {name}")


def is_sole_pad(name: str) -> bool:
    return name.startswith("foot_L_1_sole_pad_") or name.startswith("foot_R_v1_1_sole_pad_")


def scale_sole_pads(root: ET.Element, scale_x: float, scale_y: float) -> list[dict[str, object]]:
    foot_bodies = [find_body(root, "foot_L_1"), find_body(root, "foot_R_v1_1")]
    records: list[dict[str, object]] = []
    for body in foot_bodies:
        pads = [geom for geom in body.findall("geom") if is_sole_pad(geom.attrib.get("name", ""))]
        if not pads:
            continue
        centers = [parse_vec(geom.attrib["pos"]) for geom in pads]
        origin_x = sum(pos[0] for pos in centers) / len(centers)
        origin_y = sum(pos[1] for pos in centers) / len(centers)
        for geom, old_pos in zip(pads, centers):
            old_size = parse_vec(geom.attrib["size"])
            new_pos = list(old_pos)
            new_size = list(old_size)
            new_pos[0] = origin_x + (old_pos[0] - origin_x) * scale_x
            new_pos[1] = origin_y + (old_pos[1] - origin_y) * scale_y
            new_size[0] = old_size[0] * scale_x
            new_size[1] = old_size[1] * scale_y
            geom.attrib["pos"] = fmt_vec(new_pos)
            geom.attrib["size"] = fmt_vec(new_size)
            records.append(
                {
                    "name": geom.attrib["name"],
                    "old_pos": old_pos,
                    "new_pos": new_pos,
                    "old_size": old_size,
                    "new_size": new_size,
                }
            )
    return records


def shift_body_inertial(root: ET.Element, body_name: str, shift: list[float]) -> dict[str, object]:
    body = find_body(root, body_name)
    inertial = body.find("inertial")
    if inertial is None:
        raise ValueError(f"Body has no inertial element: {body_name}")
    old_pos = parse_vec(inertial.attrib.get("pos", "0 0 0"))
    new_pos = [old_pos[idx] + shift[idx] for idx in range(3)]
    inertial.attrib["pos"] = fmt_vec(new_pos)
    return {
        "body": body_name,
        "shift": shift,
        "old_pos": old_pos,
        "new_pos": new_pos,
        "mass": float(inertial.attrib["mass"]),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description="Build virtual design variants for hardware feasibility probes.")
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--report", type=Path, required=True)
    parser.add_argument("--sole-scale-x", type=float, default=1.0)
    parser.add_argument("--sole-scale-y", type=float, default=1.0)
    parser.add_argument("--base-com-shift", nargs=3, type=float, default=(0.0, 0.0, 0.0), metavar=("DX", "DY", "DZ"))
    args = parser.parse_args()

    tree = ET.parse(args.source)
    root = tree.getroot()
    root.attrib["model"] = args.out.stem

    sole_records = []
    if args.sole_scale_x != 1.0 or args.sole_scale_y != 1.0:
        sole_records = scale_sole_pads(root, args.sole_scale_x, args.sole_scale_y)

    base_shift_record = None
    shift = [float(v) for v in args.base_com_shift]
    if any(abs(v) > 0.0 for v in shift):
        base_shift_record = shift_body_inertial(root, "base_link", shift)

    args.out.parent.mkdir(parents=True, exist_ok=True)
    ET.indent(root, space="  ")
    tree.write(args.out, encoding="utf-8", xml_declaration=False)
    compiled = mujoco.MjModel.from_xml_path(str(args.out.resolve()))

    payload = {
        "source": str(args.source.resolve()),
        "out": str(args.out.resolve()),
        "sole_scale_x": args.sole_scale_x,
        "sole_scale_y": args.sole_scale_y,
        "base_com_shift": shift,
        "sole_pad_changes": sole_records,
        "base_shift": base_shift_record,
        "compiled": {
            "nq": int(compiled.nq),
            "nv": int(compiled.nv),
            "nu": int(compiled.nu),
            "njnt": int(compiled.njnt),
            "ngeom": int(compiled.ngeom),
        },
    }
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(json.dumps(payload, indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
