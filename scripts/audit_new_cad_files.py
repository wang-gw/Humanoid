from __future__ import annotations

import argparse
import csv
import json
import re
import struct
from pathlib import Path

import numpy as np


def read_binary_stl_bounds(path: Path) -> dict[str, object]:
    data = path.read_bytes()
    if len(data) < 84:
        raise ValueError(f"STL too small: {path}")
    tri_count = struct.unpack_from("<I", data, 80)[0]
    expected = 84 + tri_count * 50
    if expected > len(data):
        raise ValueError(f"STL size mismatch: {path} expected={expected} actual={len(data)}")
    mins = np.array([np.inf, np.inf, np.inf], dtype=float)
    maxs = np.array([-np.inf, -np.inf, -np.inf], dtype=float)
    offset = 84
    for _ in range(tri_count):
        # normal 12 bytes, then 3 vertices.
        offset += 12
        for _vertex in range(3):
            v = np.array(struct.unpack_from("<fff", data, offset), dtype=float)
            mins = np.minimum(mins, v)
            maxs = np.maximum(maxs, v)
            offset += 12
        offset += 2
    return {
        "file": str(path),
        "bytes": path.stat().st_size,
        "triangles": int(tri_count),
        "min_xyz": [float(v) for v in mins],
        "max_xyz": [float(v) for v in maxs],
        "size_xyz": [float(v) for v in (maxs - mins)],
    }


def canonical_stl_name(name: str) -> str:
    stem = Path(name).stem
    stem = stem.replace(" (1)", "")
    stem = re.sub(r"_v\d+$", "", stem)
    stem = re.sub(r"_1$", "", stem)
    stem = stem.replace("AK45-10_R", "AK45-10_frR")
    return stem


def parse_step_summary(path: Path) -> dict[str, object]:
    text = path.read_text(encoding="utf-8", errors="replace")
    header = text[:5000]
    product_names = sorted(set(re.findall(r"PRODUCT\('([^']*)'", text)))
    shape_names = sorted(set(re.findall(r"PRODUCT_DEFINITION_SHAPE\('([^']*)'", text)))
    entity_counts = {
        "ITEM_DEFINED_TRANSFORMATION": text.count("ITEM_DEFINED_TRANSFORMATION"),
        "SHAPE_REPRESENTATION": text.count("SHAPE_REPRESENTATION"),
        "PRODUCT": text.count("PRODUCT("),
        "NEXT_ASSEMBLY_USAGE_OCCURRENCE": text.count("NEXT_ASSEMBLY_USAGE_OCCURRENCE"),
        "AXIS2_PLACEMENT_3D": text.count("AXIS2_PLACEMENT_3D"),
        "CARTESIAN_POINT": text.count("CARTESIAN_POINT"),
    }
    return {
        "file": str(path),
        "bytes": path.stat().st_size,
        "header_excerpt": header,
        "product_names": product_names,
        "shape_names": shape_names[:100],
        "entity_counts": entity_counts,
    }


def compare_stl_sets(new_records: list[dict[str, object]], old_records: list[dict[str, object]]) -> list[dict[str, object]]:
    old_by_key = {canonical_stl_name(Path(record["file"]).name): record for record in old_records}
    rows = []
    for new in new_records:
        key = canonical_stl_name(Path(new["file"]).name)
        old = old_by_key.get(key)
        row: dict[str, object] = {
            "canonical_name": key,
            "new_file": Path(new["file"]).name,
            "new_triangles": new["triangles"],
            "new_size_xyz": new["size_xyz"],
            "old_file": "",
            "old_triangles": "",
            "old_size_xyz": "",
            "size_diff_norm": "",
            "matched": False,
        }
        if old is not None:
            new_size = np.array(new["size_xyz"], dtype=float)
            old_size = np.array(old["size_xyz"], dtype=float)
            row.update(
                {
                    "old_file": Path(old["file"]).name,
                    "old_triangles": old["triangles"],
                    "old_size_xyz": old["size_xyz"],
                    "size_diff_norm": float(np.linalg.norm(new_size - old_size)),
                    "matched": True,
                }
            )
        rows.append(row)
    return rows


def main() -> int:
    parser = argparse.ArgumentParser(description="새 STEP/STL CAD 파일을 감사한다.")
    parser.add_argument("--step", type=Path, default=Path("URDF_F_.step"))
    parser.add_argument("--link-dir", type=Path, default=Path("link"))
    parser.add_argument("--old-mesh-dir", type=Path, default=Path("URDF_F_description/URDF_F_description/meshes"))
    parser.add_argument("--out-dir", type=Path, default=Path("outputs/analysis/new_cad_files"))
    parser.add_argument("--doc-dir", type=Path, default=Path("docs/hardware_validation"))
    args = parser.parse_args()

    out_dir = args.out_dir.resolve()
    doc_dir = args.doc_dir.resolve()
    out_dir.mkdir(parents=True, exist_ok=True)
    doc_dir.mkdir(parents=True, exist_ok=True)

    new_records = [read_binary_stl_bounds(path) for path in sorted(args.link_dir.glob("*.stl"))]
    old_records = [read_binary_stl_bounds(path) for path in sorted(args.old_mesh_dir.glob("*.stl"))]
    comparisons = compare_stl_sets(new_records, old_records)
    step_summary = parse_step_summary(args.step)

    summary = {
        "step": step_summary,
        "new_link_stl_count": len(new_records),
        "old_mesh_stl_count": len(old_records),
        "new_link_stls": new_records,
        "old_mesh_stls": old_records,
        "comparisons": comparisons,
    }
    json_path = out_dir / "new_cad_files_audit.json"
    json_path.write_text(json.dumps(summary, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")

    stl_csv = out_dir / "link_stl_bounds.csv"
    with stl_csv.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=["file", "bytes", "triangles", "min_xyz", "max_xyz", "size_xyz"])
        writer.writeheader()
        writer.writerows(new_records)

    comparison_csv = out_dir / "stl_name_comparison.csv"
    with comparison_csv.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(
            handle,
            fieldnames=[
                "canonical_name",
                "new_file",
                "new_triangles",
                "new_size_xyz",
                "old_file",
                "old_triangles",
                "old_size_xyz",
                "size_diff_norm",
                "matched",
            ],
        )
        writer.writeheader()
        writer.writerows(comparisons)

    missing = [row for row in comparisons if not row["matched"]]
    duplicates = {}
    for path in sorted(args.link_dir.glob("*.stl")):
        key = canonical_stl_name(path.name)
        duplicates.setdefault(key, []).append(path.name)
    duplicates = {k: v for k, v in duplicates.items() if len(v) > 1}

    md_path = doc_dir / "24_step_and_link_stl_check.md"
    lines = [
        "# STEP와 link STL 확인",
        "",
        "## 확인 대상",
        "",
        f"- STEP: `{args.step.resolve()}`",
        f"- link STL 폴더: `{args.link_dir.resolve()}`",
        "",
        "## STEP 파일",
        "",
        f"- 크기: `{args.step.stat().st_size}` bytes",
        f"- schema/entity 기준: `{step_summary['entity_counts']}`",
        "",
        "STEP 헤더상 Autodesk Translation Framework에서 생성된 ASCII STEP 파일이다. 즉 Fusion 360에서 export한 중립 CAD 파일로 보인다.",
        "",
        "## STL 파일",
        "",
        f"- 새 link STL 수: `{len(new_records)}`",
        f"- 기존 URDF mesh STL 수: `{len(old_records)}`",
        f"- 기존 mesh와 이름 매칭된 새 STL 수: `{sum(1 for row in comparisons if row['matched'])}`",
        f"- 기존 mesh와 이름 매칭되지 않은 새 STL 수: `{len(missing)}`",
        "",
        "## 주의할 점",
        "",
        "- 새 STL 이름은 기존 URDF mesh 이름보다 정리되어 있지만, 일부 이름은 그대로 매칭되지 않는다.",
        "- `AK45-36_trL.stl`과 `AK45-36_trL (1).stl`이 동시에 존재한다. 하나는 오른쪽 hip roll actuator mesh일 가능성이 있으므로 이름 확인이 필요하다.",
        "- 기존에는 오른쪽 ankle actuator가 `AK45-10_R_1.stl`였고, 새 파일은 `AK45-10_frR.stl`이다. 이름은 더 일관적이지만 URDF reference를 업데이트해야 한다.",
        "",
        "## 중복/확인 필요 파일",
        "",
    ]
    if duplicates:
        for key, names in duplicates.items():
            lines.append(f"- canonical `{key}`: `{names}`")
    else:
        lines.append("- 중복 canonical 이름 없음")
    lines.extend(
        [
            "",
            "## 산출물",
            "",
            f"- JSON: `{json_path}`",
            f"- STL bounds CSV: `{stl_csv}`",
            f"- STL 비교 CSV: `{comparison_csv}`",
            "",
            "## 판단",
            "",
            "새 STEP와 STL 파일들은 검증에 필요한 CAD export로 보인다. 다만 바로 기존 MJCF에 덮어쓰기보다는, 파일명 매핑을 확정한 뒤 새 asset 세트를 별도 모델로 생성하는 것이 안전하다.",
            "",
        ]
    )
    md_path.write_text("\n".join(lines), encoding="utf-8")

    print(f"Wrote {json_path}")
    print(f"Wrote {stl_csv}")
    print(f"Wrote {comparison_csv}")
    print(f"Wrote {md_path}")
    print(json.dumps({"new_link_stl_count": len(new_records), "old_mesh_stl_count": len(old_records), "matched": sum(1 for row in comparisons if row["matched"]), "missing": len(missing), "duplicates": duplicates}, indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
