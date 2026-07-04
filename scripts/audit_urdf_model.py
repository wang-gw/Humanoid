from __future__ import annotations

import argparse
import json
import math
import re
import xml.etree.ElementTree as ET
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any


@dataclass
class LinkAudit:
    name: str
    mass_kg: float | None
    com_xyz: list[float] | None
    inertia: dict[str, float] | None
    visual_mesh_count: int
    collision_mesh_count: int


@dataclass
class JointAudit:
    name: str
    joint_type: str
    parent: str | None
    child: str | None
    origin_xyz: list[float] | None
    origin_rpy: list[float] | None
    axis_xyz: list[float] | None
    lower_rad: float | None
    upper_rad: float | None
    effort: float | None
    velocity: float | None


def parse_floats(text: str | None) -> list[float] | None:
    if text is None:
        return None
    values = [float(part) for part in text.split()]
    return values


def parse_float_attr(node: ET.Element | None, key: str) -> float | None:
    if node is None:
        return None
    value = node.attrib.get(key)
    return None if value is None else float(value)


def find_child(node: ET.Element, name: str) -> ET.Element | None:
    for child in node:
        if child.tag == name:
            return child
    return None


def find_nested(node: ET.Element, path: list[str]) -> ET.Element | None:
    current: ET.Element | None = node
    for part in path:
        if current is None:
            return None
        current = find_child(current, part)
    return current


def parse_model(path: Path) -> tuple[list[LinkAudit], list[JointAudit]]:
    root = ET.parse(path).getroot()
    links: list[LinkAudit] = []
    joints: list[JointAudit] = []

    for link in root:
        if link.tag != "link":
            continue
        inertial = find_child(link, "inertial")
        mass_node = find_child(inertial, "mass") if inertial is not None else None
        origin_node = find_child(inertial, "origin") if inertial is not None else None
        inertia_node = find_child(inertial, "inertia") if inertial is not None else None
        inertia = None
        if inertia_node is not None:
            inertia = {
                key: float(inertia_node.attrib[key])
                for key in ("ixx", "iyy", "izz", "ixy", "iyz", "ixz")
                if key in inertia_node.attrib
            }

        visual_meshes = 0
        collision_meshes = 0
        for child in link:
            if child.tag == "visual" and find_nested(child, ["geometry", "mesh"]) is not None:
                visual_meshes += 1
            if child.tag == "collision" and find_nested(child, ["geometry", "mesh"]) is not None:
                collision_meshes += 1

        links.append(
            LinkAudit(
                name=link.attrib["name"],
                mass_kg=parse_float_attr(mass_node, "value"),
                com_xyz=parse_floats(origin_node.attrib.get("xyz") if origin_node is not None else None),
                inertia=inertia,
                visual_mesh_count=visual_meshes,
                collision_mesh_count=collision_meshes,
            )
        )

    for joint in root:
        if joint.tag != "joint":
            continue
        origin = find_child(joint, "origin")
        axis = find_child(joint, "axis")
        limit = find_child(joint, "limit")
        parent = find_child(joint, "parent")
        child = find_child(joint, "child")
        joints.append(
            JointAudit(
                name=joint.attrib["name"],
                joint_type=joint.attrib["type"],
                parent=parent.attrib.get("link") if parent is not None else None,
                child=child.attrib.get("link") if child is not None else None,
                origin_xyz=parse_floats(origin.attrib.get("xyz") if origin is not None else None),
                origin_rpy=parse_floats(origin.attrib.get("rpy") if origin is not None else None),
                axis_xyz=parse_floats(axis.attrib.get("xyz") if axis is not None else None),
                lower_rad=parse_float_attr(limit, "lower"),
                upper_rad=parse_float_attr(limit, "upper"),
                effort=parse_float_attr(limit, "effort"),
                velocity=parse_float_attr(limit, "velocity"),
            )
        )

    return links, joints


def side_key(name: str) -> tuple[str, str] | None:
    if "_L" in name:
        return "L", name.replace("_L", "_X")
    if "_R" in name:
        return "R", name.replace("_R", "_X")
    return None


def build_findings(links: list[LinkAudit], joints: list[JointAudit]) -> list[str]:
    findings: list[str] = []
    revolute = [joint for joint in joints if joint.joint_type == "revolute"]

    missing_inertial = [link.name for link in links if link.mass_kg is None or link.inertia is None]
    if missing_inertial:
        findings.append(f"{len(missing_inertial)} links are missing mass or inertia.")

    auto_named = [joint.name for joint in revolute if re.fullmatch(r"Revolute \d+", joint.name)]
    if auto_named:
        findings.append(f"{len(auto_named)} revolute joints use CAD-generated names.")

    efforts = sorted({joint.effort for joint in revolute})
    velocities = sorted({joint.velocity for joint in revolute})
    if len(efforts) == 1:
        findings.append(f"All revolute joints share the same effort limit: {efforts[0]}.")
    if len(velocities) == 1:
        findings.append(f"All revolute joints share the same velocity limit: {velocities[0]}.")

    mesh_collisions = [link.name for link in links if link.collision_mesh_count > 0]
    if mesh_collisions:
        findings.append(
            f"{len(mesh_collisions)} links use mesh collision geometry; simplified collision should be considered."
        )

    paired: dict[str, dict[str, LinkAudit]] = {}
    for link in links:
        key = side_key(link.name)
        if key is None:
            continue
        side, base = key
        paired.setdefault(base, {})[side] = link
    mass_mismatches = []
    for base, sides in paired.items():
        left = sides.get("L")
        right = sides.get("R")
        if left is None or right is None or left.mass_kg is None or right.mass_kg is None:
            continue
        if not math.isclose(left.mass_kg, right.mass_kg, rel_tol=0.02, abs_tol=0.02):
            mass_mismatches.append((base, left.mass_kg, right.mass_kg))
    if mass_mismatches:
        findings.append(f"{len(mass_mismatches)} left/right link mass pairs differ by more than tolerance.")

    return findings


def markdown_report(model_path: Path, links: list[LinkAudit], joints: list[JointAudit], findings: list[str]) -> str:
    revolute = [joint for joint in joints if joint.joint_type == "revolute"]
    fixed = [joint for joint in joints if joint.joint_type == "fixed"]
    total_mass = sum(link.mass_kg or 0.0 for link in links)

    lines = [
        "# Initial URDF/Xacro Model Audit",
        "",
        "## Source",
        "",
        f"- Model file: `{model_path}`",
        "- Purpose: RL walking feasibility and hardware sanity validation baseline",
        "",
        "## Summary",
        "",
        f"- Links with mass entries: `{sum(1 for link in links if link.mass_kg is not None)}`",
        f"- Total modeled mass: `{total_mass:.6f} kg`",
        f"- Total joints: `{len(joints)}`",
        f"- Revolute joints: `{len(revolute)}`",
        f"- Fixed joints: `{len(fixed)}`",
        "",
        "## Findings",
        "",
    ]
    if findings:
        lines.extend(f"- {finding}" for finding in findings)
    else:
        lines.append("- No immediate structural findings from static XML audit.")

    lines.extend(
        [
            "",
            "## Revolute Joint Inventory",
            "",
            "| Joint | Parent | Child | Axis | Lower rad | Upper rad | Effort | Velocity |",
            "| --- | --- | --- | --- | ---: | ---: | ---: | ---: |",
        ]
    )
    for joint in revolute:
        axis = " ".join(f"{value:g}" for value in joint.axis_xyz) if joint.axis_xyz else ""
        lines.append(
            "| "
            + " | ".join(
                [
                    joint.name,
                    joint.parent or "",
                    joint.child or "",
                    axis,
                    "" if joint.lower_rad is None else f"{joint.lower_rad:.6f}",
                    "" if joint.upper_rad is None else f"{joint.upper_rad:.6f}",
                    "" if joint.effort is None else f"{joint.effort:g}",
                    "" if joint.velocity is None else f"{joint.velocity:g}",
                ]
            )
            + " |"
        )

    lines.extend(
        [
            "",
            "## Next Validation Step",
            "",
            "1. Convert or mirror the model into a MuJoCo-loadable asset.",
            "2. Define a neutral standing pose and verify ground contact stability.",
            "3. Run PD standing, squat, weight-shift, and one-leg-support probes.",
            "4. Record joint torque, joint velocity, base pose, contact force, and failure reason.",
        ]
    )
    return "\n".join(lines) + "\n"


def main() -> int:
    parser = argparse.ArgumentParser(description="Audit URDF/xacro robot model structure.")
    parser.add_argument(
        "--model",
        type=Path,
        default=Path("URDF_F_description/URDF_F_description/urdf/URDF_F.xacro"),
    )
    parser.add_argument("--out-dir", type=Path, default=Path("docs/hardware_validation"))
    args = parser.parse_args()

    model_path = args.model.resolve()
    out_dir = args.out_dir.resolve()
    out_dir.mkdir(parents=True, exist_ok=True)

    links, joints = parse_model(model_path)
    findings = build_findings(links, joints)

    payload: dict[str, Any] = {
        "model_path": str(model_path),
        "summary": {
            "link_count": len(links),
            "joint_count": len(joints),
            "revolute_joint_count": sum(1 for joint in joints if joint.joint_type == "revolute"),
            "fixed_joint_count": sum(1 for joint in joints if joint.joint_type == "fixed"),
            "total_mass_kg": sum(link.mass_kg or 0.0 for link in links),
        },
        "findings": findings,
        "links": [asdict(link) for link in links],
        "joints": [asdict(joint) for joint in joints],
    }

    json_path = out_dir / "initial_model_audit.json"
    md_path = out_dir / "initial_model_audit.md"
    json_path.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    md_path.write_text(markdown_report(model_path, links, joints, findings), encoding="utf-8")

    print(f"Wrote {json_path}")
    print(f"Wrote {md_path}")
    print(f"Total mass: {payload['summary']['total_mass_kg']:.6f} kg")
    print(f"Revolute joints: {payload['summary']['revolute_joint_count']}")
    if findings:
        print("Findings:")
        for finding in findings:
            print(f"- {finding}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
