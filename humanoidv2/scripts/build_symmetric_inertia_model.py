#!/usr/bin/env python3
"""Build a mass-preserving sagittal-mirror version of the robot inertials."""

from __future__ import annotations

import argparse
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_SOURCE = ROOT / "models/urdf_f_v2/URDF_F_v2_footprint_contact.xml"
DEFAULT_OUTPUT = ROOT / "models/urdf_f_v2/URDF_F_v2_footprint_contact_symmetric_inertia.xml"
MIRROR = np.diag([-1.0, 1.0, 1.0])
BODY_PAIRS = (
    ("hipjoint1_L_1", "hipjoint1_R_1"),
    ("thigh_L_1", "thigh_R_1"),
    ("calf_L_1", "calf_R_1"),
    ("footJ_L_1", "footJ_R_1"),
    ("foot_L_1", "foot_R_1"),
)


def inertia_tensor(model: mujoco.MjModel, body_id: int) -> np.ndarray:
    rotation_flat = np.empty(9, dtype=np.float64)
    mujoco.mju_quat2Mat(rotation_flat, model.body_iquat[body_id])
    rotation = rotation_flat.reshape(3, 3)
    return rotation @ np.diag(model.body_inertia[body_id]) @ rotation.T


def principal_inertia(tensor: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    values, rotation = np.linalg.eigh(0.5 * (tensor + tensor.T))
    if np.linalg.det(rotation) < 0.0:
        rotation[:, 0] *= -1.0
    quaternion = np.empty(4, dtype=np.float64)
    mujoco.mju_mat2Quat(quaternion, rotation.ravel())
    return values, quaternion


def set_inertial(
    body_elements: dict[str, ET.Element],
    body_name: str,
    mass: float,
    center: np.ndarray,
    tensor: np.ndarray,
) -> None:
    diagonal, quaternion = principal_inertia(tensor)
    inertial = body_elements[body_name].find("inertial")
    if inertial is None:
        raise ValueError(f"Body {body_name} has no explicit inertial")
    inertial.set("mass", f"{mass:.12g}")
    inertial.set("pos", " ".join(f"{value:.12g}" for value in center))
    inertial.set("diaginertia", " ".join(f"{value:.12g}" for value in diagonal))
    inertial.set("quat", " ".join(f"{value:.12g}" for value in quaternion))


def build(source: Path, output: Path) -> None:
    source = source.resolve()
    output = output.resolve()
    if source == output:
        raise ValueError("The symmetric model must not overwrite the source model")
    model = mujoco.MjModel.from_xml_path(str(source))
    parser = ET.XMLParser(target=ET.TreeBuilder(insert_comments=True))
    tree = ET.parse(source, parser=parser)
    body_elements = {
        body.attrib["name"]: body for body in tree.getroot().iter("body") if "name" in body.attrib
    }
    source_mass = float(model.body_mass.sum())

    for left_name, right_name in BODY_PAIRS:
        left_id = model.body(left_name).id
        right_id = model.body(right_name).id
        mass = 0.5 * (model.body_mass[left_id] + model.body_mass[right_id])
        left_center = 0.5 * (model.body_ipos[left_id] + MIRROR @ model.body_ipos[right_id])
        left_tensor = 0.5 * (
            inertia_tensor(model, left_id)
            + MIRROR @ inertia_tensor(model, right_id) @ MIRROR
        )
        set_inertial(body_elements, left_name, mass, left_center, left_tensor)
        set_inertial(body_elements, right_name, mass, MIRROR @ left_center, MIRROR @ left_tensor @ MIRROR)

    base_id = model.body("base_link").id
    base_center = model.body_ipos[base_id].copy()
    base_center[0] = 0.0
    base_tensor = inertia_tensor(model, base_id)
    base_tensor = 0.5 * (base_tensor + MIRROR @ base_tensor @ MIRROR)
    set_inertial(body_elements, "base_link", model.body_mass[base_id], base_center, base_tensor)

    output.parent.mkdir(parents=True, exist_ok=True)
    ET.indent(tree, space="  ")
    tree.write(output, encoding="unicode", short_empty_elements=True)
    generated = mujoco.MjModel.from_xml_path(str(output))
    if not np.isclose(generated.body_mass.sum(), source_mass, rtol=0.0, atol=1.0e-12):
        raise RuntimeError("Symmetrization changed the total mass")
    print(f"source={source}")
    print(f"output={output}")
    print(f"total_mass_kg={generated.body_mass.sum():.9f}")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", type=Path, default=DEFAULT_SOURCE)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args()
    build(args.source, args.output)


if __name__ == "__main__":
    main()
