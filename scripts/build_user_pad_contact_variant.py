from __future__ import annotations

import argparse
import json
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco
import numpy as np


USER_PAD_CAD_LOCAL = {
    "left": {
        "body": "foot_L_1",
        "geom": "foot_L_1_sole_collision",
        "center_mm": (0.0, -35.750, 20.000),
        "halfsize_mm": (35.000, 24.250, 20.000),
    },
    "right": {
        "body": "foot_R_v1_1",
        "geom": "foot_R_v1_1_sole_collision",
        "center_mm": (0.0, -35.750, -20.000),
        "halfsize_mm": (35.000, 24.250, 20.000),
    },
}


def find_body(root: ET.Element, name: str) -> ET.Element:
    for body in root.iter("body"):
        if body.attrib.get("name") == name:
            return body
    raise ValueError(f"Body not found: {name}")


def remove_existing_contacts(root: ET.Element) -> int:
    removed = 0
    for body in root.iter("body"):
        for geom in list(body.findall("geom")):
            name = geom.attrib.get("name", "")
            if "sole_collision" in name or "sole_pad" in name:
                body.remove(geom)
                removed += 1
    return removed


def fmt(values: np.ndarray | list[float]) -> str:
    return " ".join(f"{float(value):.8g}" for value in values)


def transform_pad(center_mm: tuple[float, float, float], half_mm: tuple[float, float, float], mode: str) -> tuple[np.ndarray, np.ndarray]:
    center = np.array(center_mm, dtype=np.float64) / 1000.0
    half = np.array(half_mm, dtype=np.float64) / 1000.0

    if mode.startswith("axis_confirmed"):
        # Confirmed global convention: MuJoCo XYZ = [CAD Y, CAD X, CAD Z].
        out_center = np.array([center[1], center[0], center[2]], dtype=np.float64)
        out_half = np.array([half[1], half[0], half[2]], dtype=np.float64)
    elif mode.startswith("as_mjcf_local"):
        # Diagnostic fallback: interpret the supplied local XYZ as already matching the current MJCF foot body frame.
        out_center = center.copy()
        out_half = half.copy()
    elif mode.startswith("toeheel_z"):
        # User-confirmed foot-local convention:
        # local X = lateral width, local Y = vertical pad thickness, local Z = toe/heel direction.
        # Current MJCF uses X forward, Y lateral, Z up, so map [foot Z, foot X, foot Y].
        out_center = np.array([center[2], center[0], center[1]], dtype=np.float64)
        out_half = np.array([half[2], half[0], half[1]], dtype=np.float64)
    else:
        raise ValueError(f"Unknown mode: {mode}")

    if mode.endswith("_grounded"):
        out_center[2] = -out_half[2]
    return out_center, out_half


def main() -> int:
    parser = argparse.ArgumentParser(description="Build user-supplied local contact pad variants.")
    parser.add_argument("--source", type=Path, default=Path("envs/robots/urdf_f_link/URDF_F_link_user_size_mass_inertia_direct_nobase_contact.xml"))
    parser.add_argument(
        "--mode",
        choices=[
            "axis_confirmed_raw",
            "axis_confirmed_grounded",
            "as_mjcf_local_raw",
            "as_mjcf_local_grounded",
            "toeheel_z_raw",
            "toeheel_z_grounded",
        ],
        default="axis_confirmed_grounded",
    )
    parser.add_argument("--out", type=Path, default=Path("envs/robots/urdf_f_link/URDF_F_link_user_size_mass_inertia_user_pad_contact.xml"))
    parser.add_argument("--friction", default="1.0 0.02 0.001")
    parser.add_argument("--doc-dir", type=Path, default=Path("docs/hardware_validation"))
    args = parser.parse_args()

    source = args.source.resolve()
    out = args.out.resolve()
    doc_dir = args.doc_dir.resolve()
    doc_dir.mkdir(parents=True, exist_ok=True)

    tree = ET.parse(source)
    root = tree.getroot()
    root.attrib["model"] = out.stem
    removed = remove_existing_contacts(root)

    records = []
    for side, spec in USER_PAD_CAD_LOCAL.items():
        center, half = transform_pad(spec["center_mm"], spec["halfsize_mm"], args.mode)
        body = find_body(root, str(spec["body"]))
        ET.SubElement(
            body,
            "geom",
            {
                "name": str(spec["geom"]),
                "type": "box",
                "pos": fmt(center),
                "size": fmt(half),
                "rgba": "0.1 0.8 0.2 0.35",
                "friction": args.friction,
                "condim": "3",
            },
        )
        records.append(
            {
                "side": side,
                "body": spec["body"],
                "geom": spec["geom"],
                "input_center_mm": spec["center_mm"],
                "input_halfsize_mm": spec["halfsize_mm"],
                "center_xyz_in_body": [float(v) for v in center],
                "halfsize_xyz_m": [float(v) for v in half],
            }
        )

    ET.indent(root, space="  ")
    tree.write(out, encoding="utf-8", xml_declaration=False)
    compiled = mujoco.MjModel.from_xml_path(str(out))
    payload = {
        "source": str(source),
        "out": str(out),
        "mode": args.mode,
        "removed_contact_geoms": removed,
        "friction": args.friction,
        "records": records,
        "compiled": {"nq": int(compiled.nq), "nv": int(compiled.nv), "nu": int(compiled.nu), "njnt": int(compiled.njnt), "ngeom": int(compiled.ngeom)},
    }
    json_path = doc_dir / f"user_pad_contact_variant_{args.mode}_report.json"
    json_path.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(f"Wrote {out}")
    print(f"Wrote {json_path}")
    print(json.dumps(payload["compiled"], indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
