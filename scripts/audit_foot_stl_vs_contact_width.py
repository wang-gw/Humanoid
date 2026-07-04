from __future__ import annotations

import argparse
import json
import struct
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np


FOOT_STLS = {
    "left": "foot_L.stl",
    "right": "foot_R.stl",
}

FOOT_BODIES = {
    "left": "foot_L_1",
    "right": "foot_R_v1_1",
}


def read_binary_stl_bounds(path: Path) -> dict[str, object]:
    data = path.read_bytes()
    if len(data) < 84:
        raise ValueError(f"STL too small: {path}")
    tri_count = struct.unpack_from("<I", data, 80)[0]
    expected = 84 + tri_count * 50
    if expected > len(data):
        raise ValueError(f"STL size mismatch: {path} expected={expected} actual={len(data)}")
    mins = np.array([np.inf, np.inf, np.inf], dtype=np.float64)
    maxs = np.array([-np.inf, -np.inf, -np.inf], dtype=np.float64)
    offset = 84
    for _ in range(tri_count):
        offset += 12
        for _vertex in range(3):
            vertex = np.array(struct.unpack_from("<fff", data, offset), dtype=np.float64)
            mins = np.minimum(mins, vertex)
            maxs = np.maximum(maxs, vertex)
            offset += 12
        offset += 2
    size = maxs - mins
    return {
        "file": str(path.resolve()),
        "triangles": int(tri_count),
        "min_mm": [float(v) for v in mins],
        "max_mm": [float(v) for v in maxs],
        "size_mm": [float(v) for v in size],
        "center_mm": [float(v) for v in (mins + maxs) / 2.0],
    }


def parse_vec(value: str) -> np.ndarray:
    parts = np.array([float(item) for item in value.split()], dtype=np.float64)
    if len(parts) != 3:
        raise ValueError(f"Expected 3 values, got {value!r}")
    return parts


def find_body(root: ET.Element, name: str) -> ET.Element:
    for body in root.iter("body"):
        if body.attrib.get("name") == name:
            return body
    raise ValueError(f"Body not found: {name}")


def contact_bounds_for_body(root: ET.Element, body_name: str) -> dict[str, object]:
    body = find_body(root, body_name)
    mins = np.array([np.inf, np.inf, np.inf], dtype=np.float64)
    maxs = np.array([-np.inf, -np.inf, -np.inf], dtype=np.float64)
    geoms = []
    for geom in body.findall("geom"):
        name = geom.attrib.get("name", "")
        if "sole_pad" not in name and "sole_collision" not in name:
            continue
        pos = parse_vec(geom.attrib["pos"])
        size = parse_vec(geom.attrib["size"])
        local_min = pos - size
        local_max = pos + size
        mins = np.minimum(mins, local_min)
        maxs = np.maximum(maxs, local_max)
        geoms.append(
            {
                "name": name,
                "pos_m": [float(v) for v in pos],
                "halfsize_m": [float(v) for v in size],
                "min_m": [float(v) for v in local_min],
                "max_m": [float(v) for v in local_max],
            }
        )
    if not geoms:
        raise ValueError(f"No sole contact geoms found in body: {body_name}")
    size = maxs - mins
    return {
        "body": body_name,
        "min_m": [float(v) for v in mins],
        "max_m": [float(v) for v in maxs],
        "size_m": [float(v) for v in size],
        "min_mm": [float(v * 1000.0) for v in mins],
        "max_mm": [float(v * 1000.0) for v in maxs],
        "size_mm": [float(v * 1000.0) for v in size],
        "geoms": geoms,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description="Compare real foot STL bounds with MuJoCo sole contact pad bounds.")
    parser.add_argument("--model", type=Path, required=True)
    parser.add_argument("--link-dir", type=Path, default=Path("link"))
    parser.add_argument("--out-dir", type=Path, required=True)
    args = parser.parse_args()

    out_dir = args.out_dir.resolve()
    out_dir.mkdir(parents=True, exist_ok=True)
    root = ET.parse(args.model).getroot()

    feet = {}
    for side, stl_name in FOOT_STLS.items():
        stl_bounds = read_binary_stl_bounds(args.link_dir / stl_name)
        contact_bounds = contact_bounds_for_body(root, FOOT_BODIES[side])
        stl_size = np.array(stl_bounds["size_mm"], dtype=np.float64)
        contact_size = np.array(contact_bounds["size_mm"], dtype=np.float64)
        feet[side] = {
            "stl": stl_bounds,
            "contact": contact_bounds,
            "size_ratio_contact_over_stl_xyz": [float(v) for v in contact_size / np.maximum(stl_size, 1e-9)],
            "stl_sorted_sizes_mm": [float(v) for v in sorted(stl_size)],
            "contact_sorted_sizes_mm": [float(v) for v in sorted(contact_size)],
        }

    required_contact_width_mm = 129.5
    payload = {
        "model": str(args.model.resolve()),
        "link_dir": str(args.link_dir.resolve()),
        "feet": feet,
        "reference_from_sweep": {
            "current_contact_width_mm": 74.0,
            "minimum_passing_scale_y": 1.75,
            "minimum_passing_contact_width_mm": required_contact_width_mm,
        },
        "interpretation_note": (
            "STL bounds are in CAD mesh local axes. Contact bounds are in MuJoCo foot body local axes. "
            "Axis correspondence must be interpreted with the known CAD/MuJoCo mapping, so this audit reports "
            "all XYZ sizes and sorted dimensions instead of assuming a single width axis."
        ),
    }
    out_path = out_dir / "foot_stl_vs_contact_width.json"
    out_path.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(json.dumps(payload, indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
