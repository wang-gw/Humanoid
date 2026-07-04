from __future__ import annotations

import argparse
import json
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco


def load_mapping(path: Path) -> dict[str, str]:
    payload = json.loads(path.read_text(encoding="utf-8"))
    return {
        str(old_name): str(info["new_name"])
        for old_name, info in payload["joints"].items()
    }


def rename_model(root: ET.Element, mapping: dict[str, str]) -> dict[str, int]:
    renamed_joints = 0
    renamed_actuators = 0
    renamed_references = 0

    for joint in root.iter("joint"):
        old = joint.attrib.get("name")
        if old in mapping:
            joint.attrib["name"] = mapping[old]
            renamed_joints += 1

    for actuator in root.iter("motor"):
        old_joint = actuator.attrib.get("joint")
        if old_joint in mapping:
            new_joint = mapping[old_joint]
            actuator.attrib["joint"] = new_joint
            actuator.attrib["name"] = f"motor_{new_joint}"
            renamed_actuators += 1
            renamed_references += 1

    return {
        "renamed_joints": renamed_joints,
        "renamed_actuators": renamed_actuators,
        "renamed_references": renamed_references,
    }


def transform_pose_config(source: Path, out: Path, mapping: dict[str, str]) -> bool:
    if not source.exists():
        return False
    payload = json.loads(source.read_text(encoding="utf-8"))
    targets = payload.get("joint_targets")
    if not isinstance(targets, dict):
        return False
    payload["joint_targets"] = {
        mapping.get(str(name), str(name)): value for name, value in targets.items()
    }
    payload["source_pose_config"] = str(source.resolve())
    payload["mapping_applied"] = True
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    return True


def main() -> int:
    parser = argparse.ArgumentParser(description="임시 joint mapping을 MJCF 모델에 적용한다.")
    parser.add_argument("--source", type=Path, default=Path("envs/robots/urdf_f/URDF_F_contact.xml"))
    parser.add_argument("--mapping", type=Path, default=Path("configs/joint_mapping_provisional.json"))
    parser.add_argument("--out", type=Path, default=Path("envs/robots/urdf_f/URDF_F_named.xml"))
    parser.add_argument("--pose-source", type=Path, default=Path("configs/standing_pose_candidate.json"))
    parser.add_argument("--pose-out", type=Path, default=Path("configs/standing_pose_candidate_named.json"))
    parser.add_argument("--doc-dir", type=Path, default=Path("docs/hardware_validation"))
    args = parser.parse_args()

    source = args.source.resolve()
    mapping_path = args.mapping.resolve()
    out = args.out.resolve()
    doc_dir = args.doc_dir.resolve()
    doc_dir.mkdir(parents=True, exist_ok=True)

    mapping = load_mapping(mapping_path)
    root = ET.parse(source).getroot()
    stats = rename_model(root, mapping)
    ET.indent(root, space="  ")
    ET.ElementTree(root).write(out, encoding="utf-8", xml_declaration=False)

    model = mujoco.MjModel.from_xml_path(str(out))
    pose_written = transform_pose_config(args.pose_source.resolve(), args.pose_out.resolve(), mapping)

    report = {
        "source": str(source),
        "mapping": str(mapping_path),
        "out": str(out),
        "pose_out": str(args.pose_out.resolve()) if pose_written else None,
        "stats": stats,
        "compiled": {
            "nq": int(model.nq),
            "nv": int(model.nv),
            "nu": int(model.nu),
            "njnt": int(model.njnt),
            "ngeom": int(model.ngeom)
        },
        "joint_names": [
            mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, idx) or f"joint_{idx}"
            for idx in range(model.njnt)
        ],
        "actuator_names": [
            mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, idx) or f"actuator_{idx}"
            for idx in range(model.nu)
        ]
    }

    report_path = doc_dir / "named_model_report.json"
    report_path.write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")

    md_path = doc_dir / "14_named_model_mapping.md"
    lines = [
        "# 사람이 읽을 수 있는 조인트 이름 적용",
        "",
        "## 목적",
        "",
        "CAD export 이름인 `Revolute XX`를 그대로 사용하면 torque plot, action vector, policy output을 해석하기 어렵다. 이 단계에서는 임시 joint mapping config를 적용해 검증용 MJCF의 joint/actuator 이름을 사람이 읽을 수 있는 이름으로 바꾼다.",
        "",
        "이 이름은 아직 최종 CAD 확인 전의 임시 이름이다.",
        "",
        "## 실행 명령",
        "",
        "```bash",
        "python3 scripts/apply_joint_mapping.py",
        "```",
        "",
        "## 입력과 산출물",
        "",
        f"- 입력 모델: `{source}`",
        f"- mapping config: `{mapping_path}`",
        f"- 출력 모델: `{out}`",
        f"- 변환된 pose config: `{args.pose_out.resolve() if pose_written else '생성 안 됨'}`",
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
        "## Joint 이름",
        "",
    ]
    lines.extend(f"- `{name}`" for name in report["joint_names"])
    lines.extend(["", "## Actuator 이름", ""])
    lines.extend(f"- `{name}`" for name in report["actuator_names"])
    lines.extend([
        "",
        "## 판단",
        "",
        "이제 로그와 그래프에서 `left_knee_pitch`, `right_ankle_roll` 같은 이름을 사용할 수 있다. 다만 positive direction과 정확한 관절 역할은 아직 CAD 확인이 필요하다.",
        ""
    ])
    md_path.write_text("\n".join(lines), encoding="utf-8")

    print(f"Wrote {out}")
    print(f"Wrote {report_path}")
    print(f"Wrote {md_path}")
    if pose_written:
        print(f"Wrote {args.pose_out.resolve()}")
    print(json.dumps(report["compiled"], indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
