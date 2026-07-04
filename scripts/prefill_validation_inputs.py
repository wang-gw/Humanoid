from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import mujoco


EXPORT_BY_NAMED = {
    "left_hip_roll": "Revolute 26",
    "left_hip_pitch": "Revolute 19",
    "left_knee_pitch": "Revolute 21",
    "left_ankle_pitch": "Revolute 23",
    "left_ankle_roll": "Revolute 25",
    "right_hip_roll": "Revolute 51",
    "right_hip_pitch": "Revolute 44",
    "right_knee_pitch": "Revolute 46",
    "right_ankle_pitch": "Revolute 48",
    "right_ankle_roll": "Revolute 53",
}


def fmt_vec(values) -> str:
    return " ".join(f"{float(v):.9g}" for v in values)


def write_joint_prefill(model: mujoco.MjModel, mapping_path: Path, out: Path) -> None:
    mapping = json.loads(mapping_path.read_text(encoding="utf-8"))
    old_to_info = mapping["joints"]
    rows = []
    for old_name, info in old_to_info.items():
        new_name = info["new_name"]
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, new_name)
        lower = upper = ""
        if joint_id >= 0:
            lower = f"{float(model.jnt_range[joint_id][0]):.6f}"
            upper = f"{float(model.jnt_range[joint_id][1]):.6f}"
        rows.append(
            {
                "exported_joint": old_name,
                "provisional_name": new_name,
                "cad_confirmed_name": "",
                "side": info["side"],
                "role_candidate": info["role"],
                "axis_in_model": info["axis"],
                "positive_direction_description": "모델 축 기준 +회전, CAD 실제 방향 확인 필요",
                "joint_zero_pose_description": "MJCF qpos0 기준 0 rad, CAD 조립 기준 자세 확인 필요",
                "lower_limit_rad": lower,
                "upper_limit_rad": upper,
                "cad_confirmed": "false",
                "notes": "모델에서 자동 prefill한 값. CAD 확인 전까지 확정값 아님.",
            }
        )
    out.parent.mkdir(parents=True, exist_ok=True)
    with out.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def write_mass_prefill(model: mujoco.MjModel, out: Path) -> None:
    rows = []
    for body_id in range(1, model.nbody):
        body_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, body_id) or f"body_{body_id}"
        rows.append(
            {
                "body_name": body_name,
                "cad_mass_kg": "",
                "mjcf_mass_kg": f"{float(model.body_mass[body_id]):.9g}",
                "cad_com_x": "",
                "cad_com_y": "",
                "cad_com_z": "",
                "mjcf_inertial_pos": fmt_vec(model.body_ipos[body_id]),
                "cad_ixx": "",
                "cad_iyy": "",
                "cad_izz": "",
                "cad_ixy": "",
                "cad_ixz": "",
                "cad_iyz": "",
                "mjcf_diagonal_inertia": fmt_vec(model.body_inertia[body_id]),
                "cad_confirmed": "false",
                "notes": "MJCF 값 prefill. CAD mass/COM/inertia 확인 필요.",
            }
        )
    out.parent.mkdir(parents=True, exist_ok=True)
    with out.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def write_pose_prefill(pose_path: Path, out: Path) -> None:
    pose = json.loads(pose_path.read_text(encoding="utf-8"))
    targets = pose["joint_targets"]
    named_targets = {
        name: targets[name] for name in [
            "left_hip_roll",
            "left_hip_pitch",
            "left_knee_pitch",
            "left_ankle_pitch",
            "left_ankle_roll",
            "right_hip_roll",
            "right_hip_pitch",
            "right_knee_pitch",
            "right_ankle_pitch",
            "right_ankle_roll",
        ]
        if name in targets
    }
    payload = {
        "status": "prefilled_from_geometry_search_failed_dynamic_probe",
        "description": "기존 geometry search pose 후보를 named joint 기준으로 옮긴 값이다. 동역학 PD standing은 실패했으므로 실제 standing pose로 확정하면 안 된다.",
        "base": {
            "x": 0.0,
            "y": 0.0,
            "z": pose.get("base_z"),
            "quat_wxyz": [1.0, 0.0, 0.0, 0.0],
        },
        "joint_targets_rad": named_targets,
        "expected_feet": {
            "left_foot_center_xyz": None,
            "right_foot_center_xyz": None,
            "stance_width_m": None,
            "toe_direction_description": "미확정",
        },
        "source": {
            "method": "geometry_search_candidate_only",
            "cad_file": None,
            "notes": "COM support AABB 조건은 만족했지만 forward dynamics probe에서 실패함.",
        },
    }
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")


def write_contact_prefill(model: mujoco.MjModel, out: Path) -> None:
    def geom_record(name: str, body: str) -> dict[str, object]:
        geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name)
        if geom_id < 0:
            return {"body": body, "center_xyz_in_body": None, "halfsize_xyz_m": None, "contact_type": "box"}
        return {
            "body": body,
            "center_xyz_in_body": [float(v) for v in model.geom_pos[geom_id]],
            "halfsize_xyz_m": [float(v) for v in model.geom_size[geom_id]],
            "contact_type": "box",
        }

    floor_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "floor")
    friction = model.geom_friction[floor_id].tolist() if floor_id >= 0 else [None, None, None]
    payload = {
        "status": "prefilled_from_current_placeholder_contact_model",
        "description": "현재 URDF_F_named.xml에 들어간 임시 foot box collision 값을 채운 파일이다. 실제 발바닥 치수로 확정하면 안 된다.",
        "friction": {
            "sliding": friction[0],
            "torsional": friction[1],
            "rolling": friction[2],
            "material_notes": "floor friction 기준. 실제 sole material 확인 필요.",
        },
        "left_sole": geom_record("foot_L_1_sole_collision", "foot_L_1"),
        "right_sole": geom_record("foot_R_v1_1_sole_collision", "foot_R_v1_1"),
        "notes": [
            "현재 값은 mesh bbox에서 만든 임시 collision이다.",
            "standing probe 실패 결과가 있으므로 실제 설계값으로 교체해야 한다.",
        ],
    }
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(description="현재 모델에서 확정 가능한 검증 입력값만 prefill한다.")
    parser.add_argument("--model", type=Path, default=Path("envs/robots/urdf_f/URDF_F_named.xml"))
    parser.add_argument("--mapping", type=Path, default=Path("configs/joint_mapping_provisional.json"))
    parser.add_argument("--pose", type=Path, default=Path("configs/standing_pose_candidate_named.json"))
    parser.add_argument("--out-dir", type=Path, default=Path("configs/prefilled"))
    parser.add_argument("--doc-dir", type=Path, default=Path("docs/hardware_validation"))
    args = parser.parse_args()

    model = mujoco.MjModel.from_xml_path(str(args.model.resolve()))
    out_dir = args.out_dir.resolve()
    doc_dir = args.doc_dir.resolve()
    out_dir.mkdir(parents=True, exist_ok=True)
    doc_dir.mkdir(parents=True, exist_ok=True)

    joint_out = out_dir / "cad_joint_confirmation_prefilled.csv"
    mass_out = out_dir / "cad_mass_properties_prefilled.csv"
    pose_out = out_dir / "standing_pose_prefilled_from_failed_candidate.json"
    contact_out = out_dir / "foot_contact_prefilled_placeholder.json"

    write_joint_prefill(model, args.mapping.resolve(), joint_out)
    write_mass_prefill(model, mass_out)
    write_pose_prefill(args.pose.resolve(), pose_out)
    write_contact_prefill(model, contact_out)

    md_path = doc_dir / "22_prefilled_validation_inputs.md"
    md_path.write_text(
        "\n".join(
            [
                "# 제공된 정보 기반 입력값 Prefill",
                "",
                "## 목적",
                "",
                "현재 저장소와 시뮬레이션 모델에서 확정적으로 알 수 있는 값만 검증 입력 파일에 미리 채운다. CAD에서만 알 수 있는 값은 비워 두거나 `확인 필요`로 표시한다.",
                "",
                "## 생성 파일",
                "",
                f"- joint 확인 prefill: `{joint_out}`",
                f"- mass property prefill: `{mass_out}`",
                f"- standing pose 후보 prefill: `{pose_out}`",
                f"- foot contact placeholder prefill: `{contact_out}`",
                "",
                "## 채운 값",
                "",
                "- joint provisional name",
                "- model axis",
                "- model joint limit",
                "- MJCF body mass",
                "- MJCF inertial position",
                "- MJCF diagonal inertia",
                "- 기존 geometry search pose 후보",
                "- 현재 placeholder foot box collision 값",
                "",
                "## 일부러 채우지 않은 값",
                "",
                "- CAD에서 확인한 실제 joint 이름",
                "- positive direction의 실제 기구적 의미",
                "- CAD mass/COM/inertia",
                "- 실제 standing pose",
                "- 실제 발바닥 접촉 패드 치수",
                "- 실제 모터/감속기 사양",
                "",
                "## 해석",
                "",
                "prefill 파일은 작업 시간을 줄이기 위한 초안이다. 특히 standing pose와 foot contact prefill은 이미 동역학 probe에서 실패한 값이므로, 실제 설계값으로 확정하면 안 된다.",
                "",
            ]
        ),
        encoding="utf-8",
    )

    print(f"Wrote {joint_out}")
    print(f"Wrote {mass_out}")
    print(f"Wrote {pose_out}")
    print(f"Wrote {contact_out}")
    print(f"Wrote {md_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
