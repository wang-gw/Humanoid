from __future__ import annotations

import argparse
import json
import shutil
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco


MESH_FILE_MAP = {
    "base_link": ("base_link.stl", "base_link.stl"),
    "thighJR_L_1": ("thighJR_L.stl", "thighJR_L.stl"),
    "AK45-36_trL_1": ("AK45-36_trL.stl", "AK45-36_trL.stl"),
    "thigh_L_1": ("thigh_L.stl", "thigh_L.stl"),
    "AK45-36_kpL_1": ("AK45-36_kpL.stl", "AK45-36_kpL.stl"),
    "calf_L_1": ("calf_L.stl", "calf_L.stl"),
    "footJ_L_1": ("footJ_L.stl", "footJ_L.stl"),
    "AK45-10_frL_1": ("AK45-10_frL.stl", "AK45-10_frL.stl"),
    "foot_L_1": ("foot_L.stl", "foot_L.stl"),
    "thighJ_L_1": ("thighJ_L.stl", "thighJ_L.stl"),
    "AK45-36_tpL_1": ("AK45-36_tpL.stl", "AK45-36_tpL.stl"),
    "AK45-36_fpL_1": ("AK45-36_fpL.stl", "AK45-36_fpL.stl"),
    "thighJR_R_1": ("thighJR_R.stl", "thighJR_R.stl"),
    "AK45-36_trR_1": ("AK45-36_trL (1).stl", "AK45-36_trR.stl"),
    "thigh_R_1": ("thigh_R.stl", "thigh_R.stl"),
    "AK45-36_kpR_1": ("AK45-36_kpR.stl", "AK45-36_kpR.stl"),
    "calf_R_1": ("calf_R.stl", "calf_R.stl"),
    "AK45-36_fpR_1": ("AK45-36_fpR.stl", "AK45-36_fpR.stl"),
    "footJ_R_1": ("footJ_R.stl", "footJ_R.stl"),
    "AK45-10_R_1": ("AK45-10_frR.stl", "AK45-10_frR.stl"),
    "thighJ_R_1": ("thighJ_R.stl", "thighJ_R.stl"),
    "AK45-36_tpR_1": ("AK45-36_tpR.stl", "AK45-36_tpR.stl"),
    "foot_R_v1_1": ("foot_R.stl", "foot_R.stl"),
}


def update_mesh_assets(root: ET.Element, link_dir: Path, out_dir: Path) -> list[dict[str, str]]:
    records: list[dict[str, str]] = []
    for mesh in root.findall("./asset/mesh"):
        mesh_name = mesh.attrib.get("name")
        if mesh_name not in MESH_FILE_MAP:
            continue
        source_name, dest_name = MESH_FILE_MAP[mesh_name]
        source = link_dir / source_name
        dest = out_dir / dest_name
        if not source.exists():
            raise FileNotFoundError(f"Missing source STL for mesh {mesh_name}: {source}")
        shutil.copy2(source, dest)
        mesh.attrib["file"] = dest_name
        records.append(
            {
                "mesh_name": mesh_name,
                "source_file": str(source),
                "dest_file": str(dest),
            }
        )
    return records


def main() -> int:
    parser = argparse.ArgumentParser(description="새 link STL export를 사용하는 별도 named MJCF 모델을 생성한다.")
    parser.add_argument("--source", type=Path, default=Path("envs/robots/urdf_f/URDF_F_named.xml"))
    parser.add_argument("--link-dir", type=Path, default=Path("link"))
    parser.add_argument("--out-dir", type=Path, default=Path("envs/robots/urdf_f_link"))
    parser.add_argument("--out-name", default="URDF_F_link_named.xml")
    parser.add_argument("--doc-dir", type=Path, default=Path("docs/hardware_validation"))
    args = parser.parse_args()

    source = args.source.resolve()
    link_dir = args.link_dir.resolve()
    out_dir = args.out_dir.resolve()
    doc_dir = args.doc_dir.resolve()
    out_dir.mkdir(parents=True, exist_ok=True)
    doc_dir.mkdir(parents=True, exist_ok=True)

    root = ET.parse(source).getroot()
    root.attrib["model"] = "URDF_F_link_named"
    records = update_mesh_assets(root, link_dir, out_dir)
    ET.indent(root, space="  ")
    out_xml = out_dir / args.out_name
    ET.ElementTree(root).write(out_xml, encoding="utf-8", xml_declaration=False)

    model = mujoco.MjModel.from_xml_path(str(out_xml))
    report = {
        "source_model": str(source),
        "link_dir": str(link_dir),
        "out_model": str(out_xml),
        "mesh_records": records,
        "compiled": {
            "nq": int(model.nq),
            "nv": int(model.nv),
            "nu": int(model.nu),
            "njnt": int(model.njnt),
            "ngeom": int(model.ngeom),
        },
    }
    report_path = doc_dir / "link_stl_named_model_report.json"
    report_path.write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")

    md_path = doc_dir / "25_link_stl_named_model.md"
    lines = [
        "# link STL 기반 named 모델 생성",
        "",
        "## 목적",
        "",
        "새로 export된 `link/` STL 파일을 기존 모델에 덮어쓰지 않고, 별도 MJCF asset 세트로 구성한다.",
        "",
        "## 확정한 매핑",
        "",
        "- `AK45-36_trL (1).stl`은 오른쪽 `AK45-36_trR` 역할로 사용한다.",
        "- 오른쪽 foot mesh는 `foot_R.stl`을 사용한다.",
        "",
        "## 실행 명령",
        "",
        "```bash",
        "python3 scripts/build_link_stl_named_model.py",
        "```",
        "",
        "## 산출물",
        "",
        f"- 새 모델: `{out_xml}`",
        f"- 리포트 JSON: `{report_path}`",
        "",
        "## 컴파일 확인",
        "",
        f"- nq: `{model.nq}`",
        f"- nv: `{model.nv}`",
        f"- nu: `{model.nu}`",
        f"- joint 수: `{model.njnt}`",
        f"- geom 수: `{model.ngeom}`",
        "",
        "## Mesh 파일 매핑",
        "",
        "| mesh name | source STL | copied STL |",
        "| --- | --- | --- |",
    ]
    for record in records:
        lines.append(
            f"| `{record['mesh_name']}` | `{Path(record['source_file']).name}` | `{Path(record['dest_file']).name}` |"
        )
    lines.extend(
        [
            "",
            "## 판단",
            "",
            "모델은 MuJoCo에서 컴파일된다. 다만 새 STL은 기존 STL과 로컬 축 방향이 다른 파일들이 있으므로, 시각적 정렬과 contact primitive 위치를 별도로 확인해야 한다.",
            "",
        ]
    )
    md_path.write_text("\n".join(lines), encoding="utf-8")

    print(f"Wrote {out_xml}")
    print(f"Wrote {report_path}")
    print(f"Wrote {md_path}")
    print(json.dumps(report["compiled"], indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
