from __future__ import annotations

import argparse
import csv
import json
import re
import struct
from pathlib import Path

import numpy as np


def parse_vec(text: str) -> list[float]:
    return [float(part.strip()) for part in text.replace("\n", "").split(",") if part.strip()]


def parse_step(text: str) -> dict[str, dict[int, object]]:
    points: dict[int, list[float]] = {}
    directions: dict[int, list[float]] = {}
    placements: dict[int, dict[str, object]] = {}
    transforms: dict[int, dict[str, int]] = {}
    occurrences: dict[int, dict[str, object]] = {}

    for match in re.finditer(r"#(\d+)=CARTESIAN_POINT\([^,]*,\((.*?)\)\);", text, re.DOTALL):
        points[int(match.group(1))] = parse_vec(match.group(2))

    for match in re.finditer(r"#(\d+)=DIRECTION\([^,]*,\((.*?)\)\);", text, re.DOTALL):
        directions[int(match.group(1))] = parse_vec(match.group(2))

    for match in re.finditer(r"#(\d+)=AXIS2_PLACEMENT_3D\([^,]*,#(\d+),#(\d+),#(\d+)\);", text, re.DOTALL):
        placement_id = int(match.group(1))
        point_id = int(match.group(2))
        axis_id = int(match.group(3))
        ref_id = int(match.group(4))
        placements[placement_id] = {
            "point_ref": point_id,
            "axis_ref": axis_id,
            "ref_direction_ref": ref_id,
            "origin_mm": points.get(point_id),
            "axis": directions.get(axis_id),
            "ref_direction": directions.get(ref_id),
        }

    for match in re.finditer(r"#(\d+)=ITEM_DEFINED_TRANSFORMATION\(\$,\$,#(\d+),#(\d+)\);", text):
        transforms[int(match.group(1))] = {
            "source_placement": int(match.group(2)),
            "target_placement": int(match.group(3)),
        }

    for match in re.finditer(r"#(\d+)=NEXT_ASSEMBLY_USAGE_OCCURRENCE\((.*?)\);", text, re.DOTALL):
        body = match.group(2)
        strings = re.findall(r"'([^']*)'", body)
        refs = [int(value) for value in re.findall(r"#(\d+)", body)]
        if len(strings) < 1 or len(refs) < 2:
            continue
        occurrences[int(match.group(1))] = {
            "instance_name": strings[0],
            "name": strings[0].removesuffix(":1"),
            "parent_product_definition": refs[0],
            "child_product_definition": refs[1],
        }

    return {
        "points": points,
        "directions": directions,
        "placements": placements,
        "transforms": transforms,
        "occurrences": occurrences,
    }


def read_binary_stl_bounds(path: Path) -> dict[str, object]:
    data = path.read_bytes()
    tri_count = struct.unpack_from("<I", data, 80)[0]
    mins = np.array([np.inf, np.inf, np.inf], dtype=float)
    maxs = np.array([-np.inf, -np.inf, -np.inf], dtype=float)
    offset = 84
    for _ in range(tri_count):
        offset += 12
        for _vertex in range(3):
            vertex = np.array(struct.unpack_from("<fff", data, offset), dtype=float)
            mins = np.minimum(mins, vertex)
            maxs = np.maximum(maxs, vertex)
            offset += 12
        offset += 2
    center = 0.5 * (mins + maxs)
    return {
        "file": str(path),
        "min_xyz_mm": [float(v) for v in mins],
        "max_xyz_mm": [float(v) for v in maxs],
        "center_xyz_mm": [float(v) for v in center],
        "size_xyz_mm": [float(v) for v in maxs - mins],
    }


def stl_name_for_occurrence(name: str) -> str:
    if name == "AK45-36_trL (1)":
        return "AK45-36_trL (1).stl"
    return f"{name}.stl"


def add_pair_delta(rows: list[dict[str, object]], left_name: str, right_name: str) -> dict[str, object]:
    by_name = {str(row["name"]): row for row in rows}
    left = np.array(by_name[left_name]["target_origin_mm"], dtype=float)
    right = np.array(by_name[right_name]["target_origin_mm"], dtype=float)
    delta = left - right
    return {
        "left": left_name,
        "right": right_name,
        "target_delta_left_minus_right_mm": [float(v) for v in delta],
        "target_distance_mm": float(np.linalg.norm(delta)),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description="Audit STEP assembly transforms and compare them with STL bounds.")
    parser.add_argument("--step", type=Path, default=Path("URDF_F_.step"))
    parser.add_argument("--link-dir", type=Path, default=Path("link"))
    parser.add_argument("--out-dir", type=Path, default=Path("outputs/analysis/step_assembly"))
    parser.add_argument("--doc-dir", type=Path, default=Path("docs/hardware_validation"))
    args = parser.parse_args()

    out_dir = args.out_dir.resolve()
    doc_dir = args.doc_dir.resolve()
    out_dir.mkdir(parents=True, exist_ok=True)
    doc_dir.mkdir(parents=True, exist_ok=True)

    parsed = parse_step(args.step.read_text(encoding="utf-8", errors="replace"))
    occurrence_ids = sorted(parsed["occurrences"])
    transform_ids = sorted(parsed["transforms"])
    if len(occurrence_ids) != len(transform_ids):
        raise ValueError(f"Occurrence/transform count mismatch: {len(occurrence_ids)} vs {len(transform_ids)}")

    stl_by_name = {path.name: read_binary_stl_bounds(path) for path in sorted(args.link_dir.glob("*.stl"))}
    rows: list[dict[str, object]] = []
    for occurrence_id, transform_id in zip(occurrence_ids, transform_ids):
        occurrence = parsed["occurrences"][occurrence_id]
        transform = parsed["transforms"][transform_id]
        source = parsed["placements"][transform["source_placement"]]
        target = parsed["placements"][transform["target_placement"]]
        stl_name = stl_name_for_occurrence(str(occurrence["name"]))
        stl = stl_by_name.get(stl_name)
        target_origin = target["origin_mm"]
        stl_center = stl["center_xyz_mm"] if stl else None
        target_to_stl_center = None
        if target_origin is not None and stl_center is not None:
            target_to_stl_center = [float(v) for v in (np.array(stl_center) - np.array(target_origin))]
        rows.append(
            {
                "occurrence_id": occurrence_id,
                "transform_id": transform_id,
                "name": occurrence["name"],
                "stl_file": stl_name if stl else "",
                "source_placement": transform["source_placement"],
                "source_origin_mm": source["origin_mm"],
                "source_axis": source["axis"],
                "source_ref_direction": source["ref_direction"],
                "target_placement": transform["target_placement"],
                "target_origin_mm": target_origin,
                "target_axis": target["axis"],
                "target_ref_direction": target["ref_direction"],
                "stl_center_xyz_mm": stl_center,
                "stl_size_xyz_mm": stl["size_xyz_mm"] if stl else None,
                "stl_center_minus_target_origin_mm": target_to_stl_center,
            }
        )

    pair_rows = [
        add_pair_delta(rows, "foot_L", "foot_R"),
        add_pair_delta(rows, "footJ_L", "footJ_R"),
        add_pair_delta(rows, "thighJ_L", "thighJ_R"),
        add_pair_delta(rows, "calf_L", "calf_R"),
    ]

    csv_path = out_dir / "step_assembly_transforms.csv"
    with csv_path.open("w", newline="", encoding="utf-8") as handle:
        fieldnames = list(rows[0].keys())
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)

    pair_csv = out_dir / "step_left_right_pair_deltas.csv"
    with pair_csv.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(pair_rows[0].keys()))
        writer.writeheader()
        writer.writerows(pair_rows)

    payload = {
        "step": str(args.step.resolve()),
        "link_dir": str(args.link_dir.resolve()),
        "mapping_assumption": "NEXT_ASSEMBLY_USAGE_OCCURRENCE order is paired with ITEM_DEFINED_TRANSFORMATION order in this STEP export.",
        "rows": rows,
        "left_right_pairs": pair_rows,
        "outputs": {
            "csv": str(csv_path),
            "pair_csv": str(pair_csv),
        },
    }
    json_path = out_dir / "step_assembly_transforms_summary.json"
    json_path.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")

    foot_rows = [row for row in rows if str(row["name"]).startswith("foot")]
    md_path = doc_dir / "33_step_assembly_transform_audit.md"
    lines = [
        "# STEP Assembly Transform 감사",
        "",
        "## 목적",
        "",
        "`URDF_F_.step` 안의 assembly occurrence와 transform을 읽어 각 부품이 CAD assembly에서 어디에 놓였는지 확인한다. 28번에서 확인한 foot collision 겹침이 STL 파일 자체 문제인지, URDF/MJCF의 geom local offset 문제인지 분리하기 위한 자료다.",
        "",
        "## 실행 명령",
        "",
        "```bash",
        "python3 scripts/audit_step_assembly_transforms.py",
        "```",
        "",
        "## 파싱 기준",
        "",
        "- STEP 안의 `NEXT_ASSEMBLY_USAGE_OCCURRENCE` 수: `23`",
        "- STEP 안의 `ITEM_DEFINED_TRANSFORMATION` 수: `23`",
        "- 이 Autodesk STEP export에서는 occurrence와 transform이 같은 순서로 나열되어 있으므로 같은 index로 매칭했다.",
        "- 좌표 단위는 STEP/STL export 기준 `mm`로 기록했다.",
        "",
        "## Foot 관련 Transform",
        "",
        "| name | target origin mm | target axis | target refdir | STL center mm | STL size mm |",
        "| --- | --- | --- | --- | --- | --- |",
    ]
    for row in foot_rows:
        lines.append(
            f"| `{row['name']}` | `{row['target_origin_mm']}` | `{row['target_axis']}` | `{row['target_ref_direction']}` | `{row['stl_center_xyz_mm']}` | `{row['stl_size_xyz_mm']}` |"
        )
    lines.extend(
        [
            "",
            "## 좌우 Target Origin Delta",
            "",
            "| left | right | distance mm | delta L-R mm |",
            "| --- | --- | ---: | --- |",
        ]
    )
    for pair in pair_rows:
        lines.append(
            f"| `{pair['left']}` | `{pair['right']}` | {pair['target_distance_mm']:.6f} | `{pair['target_delta_left_minus_right_mm']}` |"
        )
    lines.extend(
        [
            "",
            "## 산출물",
            "",
            f"- JSON: `{json_path}`",
            f"- transform CSV: `{csv_path}`",
            f"- 좌우 delta CSV: `{pair_csv}`",
            "",
            "## 해석",
            "",
            "STEP assembly 기준으로 좌우 foot target origin은 서로 분리되어 있다. 따라서 좌우 foot body가 분리되어 있다는 28번 감사 결과와 일치한다.",
            "",
            "반면 기존 MJCF의 sole collision local offset은 좌우 foot body separation을 상쇄해서 두 sole collision이 같은 world 위치로 모이게 했다. 즉 현재까지의 증거로는 STL 파일 자체가 완전히 잘못된 것이라기보다, URDF/MJCF에서 foot visual/collision local origin을 잡는 방식이 문제일 가능성이 높다.",
            "",
            "다만 STEP transform만으로 실제 발바닥 패드 접촉 중심과 standing pose를 확정할 수는 없다. 이 값은 Fusion 360에서 joint origin, foot link local frame, sole pad 위치를 확인해 최종 입력값으로 확정해야 한다.",
            "",
        ]
    )
    md_path.write_text("\n".join(lines), encoding="utf-8")

    print(f"Wrote {json_path}")
    print(f"Wrote {csv_path}")
    print(f"Wrote {pair_csv}")
    print(f"Wrote {md_path}")
    print(json.dumps({"rows": len(rows), "pairs": pair_rows}, indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
