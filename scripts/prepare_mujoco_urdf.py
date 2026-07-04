from __future__ import annotations

import argparse
import shutil
import xml.etree.ElementTree as ET
from pathlib import Path


def strip_namespace(tag: str) -> str:
    if "}" in tag:
        return tag.rsplit("}", 1)[1]
    return tag


def copy_element_without_xacro(node: ET.Element) -> ET.Element | None:
    if strip_namespace(node.tag) == "include":
        return None
    copied = ET.Element(strip_namespace(node.tag), dict(node.attrib))
    copied.text = node.text
    copied.tail = node.tail
    for child in node:
        child_copy = copy_element_without_xacro(child)
        if child_copy is not None:
            copied.append(child_copy)
    return copied


def rewrite_mesh_paths(root: ET.Element, mesh_dir: Path) -> None:
    for mesh in root.iter("mesh"):
        filename = mesh.attrib.get("filename")
        if not filename:
            continue
        mesh_name = Path(filename).name
        mesh.attrib["filename"] = mesh_name


def main() -> int:
    parser = argparse.ArgumentParser(description="Prepare a MuJoCo-loadable URDF from the CAD xacro export.")
    parser.add_argument(
        "--source",
        type=Path,
        default=Path("URDF_F_description/URDF_F_description/urdf/URDF_F.xacro"),
    )
    parser.add_argument(
        "--source-mesh-dir",
        type=Path,
        default=Path("URDF_F_description/URDF_F_description/meshes"),
    )
    parser.add_argument("--out-dir", type=Path, default=Path("envs/robots/urdf_f"))
    parser.add_argument(
        "--fixed-base",
        action="store_true",
        help="Do not add the floating base joint required for walking simulation.",
    )
    args = parser.parse_args()

    source = args.source.resolve()
    source_mesh_dir = args.source_mesh_dir.resolve()
    out_dir = args.out_dir.resolve()
    out_dir.mkdir(parents=True, exist_ok=True)

    parsed = ET.parse(source).getroot()
    root = ET.Element("robot", {"name": parsed.attrib.get("name", "URDF_F")})
    if not args.fixed_base:
        root.append(ET.Element("link", {"name": "world"}))
        floating = ET.Element("joint", {"name": "floating_base", "type": "floating"})
        ET.SubElement(floating, "parent", {"link": "world"})
        ET.SubElement(floating, "child", {"link": "base_link"})
        ET.SubElement(floating, "origin", {"xyz": "0 0 0", "rpy": "0 0 0"})
        root.append(floating)
    for child in parsed:
        child_copy = copy_element_without_xacro(child)
        if child_copy is not None:
            root.append(child_copy)

    for mesh_path in sorted(source_mesh_dir.glob("*.stl")):
        shutil.copy2(mesh_path, out_dir / mesh_path.name)

    rewrite_mesh_paths(root, out_dir)
    ET.indent(root, space="  ")
    out_path = out_dir / "URDF_F_mujoco.urdf"
    ET.ElementTree(root).write(out_path, encoding="utf-8", xml_declaration=True)

    print(f"Wrote {out_path}")
    print(f"Copied {len(list(out_dir.glob('*.stl')))} STL meshes to {out_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
