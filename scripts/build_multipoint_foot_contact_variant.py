from __future__ import annotations

import argparse
import json
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco


SOLE_GEOMS = {
    "left": ("foot_L_1", "foot_L_1_sole_collision", "foot_L_1_sole_pad"),
    "right": ("foot_R_v1_1", "foot_R_v1_1_sole_collision", "foot_R_v1_1_sole_pad"),
}


def find_body(root: ET.Element, name: str) -> ET.Element:
    for body in root.iter("body"):
        if body.attrib.get("name") == name:
            return body
    raise ValueError(f"Body not found: {name}")


def parse_xyz(value: str) -> tuple[float, float, float]:
    parts = tuple(float(item) for item in value.split())
    if len(parts) != 3:
        raise ValueError(f"Expected 3 values, got: {value}")
    return parts


def fmt_xyz(values: tuple[float, float, float]) -> str:
    return " ".join(f"{value:.8g}" for value in values)


def remove_sole_geoms(body: ET.Element) -> list[dict[str, str]]:
    removed = []
    for geom in list(body.findall("geom")):
        name = geom.attrib.get("name", "")
        if "sole_collision" in name or "sole_pad" in name:
            removed.append(dict(geom.attrib))
            body.remove(geom)
    return removed


def add_quadrant_pads(
    body: ET.Element,
    prefix: str,
    center: tuple[float, float, float],
    halfsize: tuple[float, float, float],
    pad_height: float,
    overlap: float,
    friction: str,
    solref: str | None,
    solimp: str | None,
) -> list[dict[str, str]]:
    cx, cy, cz = center
    hx, hy, hz = halfsize
    bottom_z = cz - hz
    pad_hz = pad_height / 2.0
    pad_z = bottom_z + pad_hz
    pad_hx = hx / 2.0 + overlap
    pad_hy = hy / 2.0 + overlap
    x_offsets = {"rear": -hx / 2.0, "front": hx / 2.0}
    y_offsets = {"right": -hy / 2.0, "left": hy / 2.0}

    records = []
    for x_label, x_offset in x_offsets.items():
        for y_label, y_offset in y_offsets.items():
            name = f"{prefix}_{x_label}_{y_label}"
            attrib = {
                "name": name,
                "type": "box",
                "pos": fmt_xyz((cx + x_offset, cy + y_offset, pad_z)),
                "size": fmt_xyz((pad_hx, pad_hy, pad_hz)),
                "rgba": "0.1 0.8 0.2 0.35",
                "friction": friction,
                "condim": "3",
            }
            if solref:
                attrib["solref"] = solref
            if solimp:
                attrib["solimp"] = solimp
            ET.SubElement(body, "geom", attrib)
            records.append(dict(attrib))
    return records


def main() -> int:
    parser = argparse.ArgumentParser(description="Split each sole collision box into four contact pads.")
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--report", type=Path, required=True)
    parser.add_argument("--pad-height", type=float, default=0.012)
    parser.add_argument("--overlap", type=float, default=0.002)
    parser.add_argument("--friction", default="1.0 0.02 0.001")
    parser.add_argument("--solref", default="")
    parser.add_argument("--solimp", default="")
    args = parser.parse_args()

    tree = ET.parse(args.source)
    root = tree.getroot()
    root.attrib["model"] = args.out.stem

    all_removed = {}
    all_records = {}
    for side, (body_name, sole_name, prefix) in SOLE_GEOMS.items():
        body = find_body(root, body_name)
        source_geom = None
        for geom in body.findall("geom"):
            if geom.attrib.get("name") == sole_name:
                source_geom = geom
                break
        if source_geom is None:
            raise ValueError(f"Sole geom not found: {sole_name}")
        center = parse_xyz(source_geom.attrib["pos"])
        halfsize = parse_xyz(source_geom.attrib["size"])
        all_removed[side] = remove_sole_geoms(body)
        all_records[side] = add_quadrant_pads(
            body=body,
            prefix=prefix,
            center=center,
            halfsize=halfsize,
            pad_height=args.pad_height,
            overlap=args.overlap,
            friction=args.friction,
            solref=args.solref or None,
            solimp=args.solimp or None,
        )

    args.out.parent.mkdir(parents=True, exist_ok=True)
    ET.indent(root, space="  ")
    tree.write(args.out, encoding="utf-8", xml_declaration=False)
    compiled = mujoco.MjModel.from_xml_path(str(args.out.resolve()))

    payload = {
        "source": str(args.source.resolve()),
        "out": str(args.out.resolve()),
        "pad_height": args.pad_height,
        "overlap": args.overlap,
        "friction": args.friction,
        "solref": args.solref,
        "solimp": args.solimp,
        "removed": all_removed,
        "pads": all_records,
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
