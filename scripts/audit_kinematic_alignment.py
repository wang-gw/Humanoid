from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import mujoco
import numpy as np


BODY_NAMES = [
    "thighJ_L_1",
    "thighJ_R_1",
    "footJ_L_1",
    "footJ_R_1",
    "foot_L_1",
    "foot_R_v1_1",
]

GEOM_NAMES = [
    "foot_L_1_sole_collision",
    "foot_R_v1_1_sole_collision",
]

PAIR_NAMES = [
    ("thighJ_L_1", "thighJ_R_1"),
    ("footJ_L_1", "footJ_R_1"),
    ("foot_L_1", "foot_R_v1_1"),
    ("foot_L_1_sole_collision", "foot_R_v1_1_sole_collision"),
]


def vec(values: np.ndarray) -> list[float]:
    return [float(v) for v in values]


def body_record(model: mujoco.MjModel, data: mujoco.MjData, name: str) -> dict[str, object]:
    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
    if body_id < 0:
        raise ValueError(f"Body not found: {name}")
    parent_id = int(model.body_parentid[body_id])
    parent_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, parent_id) if parent_id >= 0 else None
    return {
        "type": "body",
        "name": name,
        "parent": parent_name,
        "local_pos": vec(model.body_pos[body_id]),
        "world_pos": vec(data.xpos[body_id]),
    }


def geom_record(model: mujoco.MjModel, data: mujoco.MjData, name: str) -> dict[str, object]:
    geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name)
    if geom_id < 0:
        raise ValueError(f"Geom not found: {name}")
    body_id = int(model.geom_bodyid[geom_id])
    body_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, body_id)
    return {
        "type": "geom",
        "name": name,
        "body": body_name,
        "local_pos": vec(model.geom_pos[geom_id]),
        "world_pos": vec(data.geom_xpos[geom_id]),
        "size": vec(model.geom_size[geom_id]),
    }


def pair_delta(records: dict[str, dict[str, object]], left: str, right: str) -> dict[str, object]:
    left_pos = np.asarray(records[left]["world_pos"], dtype=float)
    right_pos = np.asarray(records[right]["world_pos"], dtype=float)
    delta = left_pos - right_pos
    return {
        "left": left,
        "right": right,
        "delta_left_minus_right": vec(delta),
        "distance_m": float(np.linalg.norm(delta)),
        "abs_dx_m": float(abs(delta[0])),
        "abs_dy_m": float(abs(delta[1])),
        "abs_dz_m": float(abs(delta[2])),
    }


def joint_records(model: mujoco.MjModel, data: mujoco.MjData) -> list[dict[str, object]]:
    records = []
    for joint_id in range(model.njnt):
        if model.jnt_type[joint_id] != mujoco.mjtJoint.mjJNT_HINGE:
            continue
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, joint_id) or f"joint_{joint_id}"
        body_id = int(model.jnt_bodyid[joint_id])
        records.append(
            {
                "name": name,
                "body": mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, body_id),
                "local_axis": vec(model.jnt_axis[joint_id]),
                "world_axis": vec(data.xaxis[joint_id]),
                "qposadr": int(model.jnt_qposadr[joint_id]),
                "dofadr": int(model.jnt_dofadr[joint_id]),
            }
        )
    return records


def write_csv(path: Path, rows: list[dict[str, object]]) -> None:
    fieldnames = sorted({key for row in rows for key in row})
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def main() -> int:
    parser = argparse.ArgumentParser(description="Audit left/right body and foot collision alignment.")
    parser.add_argument("--model", type=Path, default=Path("envs/robots/urdf_f_link/URDF_F_link_named.xml"))
    parser.add_argument("--out-dir", type=Path, default=Path("outputs/analysis/kinematic_alignment_link"))
    parser.add_argument("--doc-dir", type=Path, default=Path("docs/hardware_validation"))
    args = parser.parse_args()

    model_path = args.model.resolve()
    out_dir = args.out_dir.resolve()
    doc_dir = args.doc_dir.resolve()
    out_dir.mkdir(parents=True, exist_ok=True)
    doc_dir.mkdir(parents=True, exist_ok=True)

    model = mujoco.MjModel.from_xml_path(str(model_path))
    data = mujoco.MjData(model)
    data.qpos[:] = model.qpos0
    if model.nq >= 7:
        data.qpos[0:3] = np.array([0.0, 0.0, 0.0])
        data.qpos[3:7] = np.array([1.0, 0.0, 0.0, 0.0])
    mujoco.mj_forward(model, data)

    records: dict[str, dict[str, object]] = {}
    for name in BODY_NAMES:
        records[name] = body_record(model, data, name)
    for name in GEOM_NAMES:
        records[name] = geom_record(model, data, name)

    pairs = [pair_delta(records, left, right) for left, right in PAIR_NAMES]
    joints = joint_records(model, data)

    body_csv = out_dir / "body_and_geom_alignment.csv"
    pair_csv = out_dir / "left_right_pair_deltas.csv"
    joint_csv = out_dir / "joint_axes_world.csv"
    write_csv(body_csv, list(records.values()))
    write_csv(pair_csv, pairs)
    write_csv(joint_csv, joints)

    payload = {
        "model_path": str(model_path),
        "records": records,
        "left_right_pairs": pairs,
        "joints": joints,
        "outputs": {
            "body_csv": str(body_csv),
            "pair_csv": str(pair_csv),
            "joint_csv": str(joint_csv),
        },
    }
    json_path = out_dir / "kinematic_alignment_summary.json"
    json_path.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")

    sole_pair = next(pair for pair in pairs if pair["left"] == "foot_L_1_sole_collision")
    foot_body_pair = next(pair for pair in pairs if pair["left"] == "foot_L_1")
    md_path = doc_dir / "28_kinematic_alignment_audit.md"
    lines = [
        "# 좌우 Kinematic Alignment 감사",
        "",
        "## 목적",
        "",
        "`URDF_F_link_named.xml`의 좌우 발 collision이 중립 자세에서 같은 world 위치에 겹치는 원인을 확인한다. 이 단계는 controller나 RL 문제가 아니라 모델 좌표계와 mesh origin이 올바른지 확인하는 감사다.",
        "",
        "## 실행 명령",
        "",
        "```bash",
        "python3 scripts/audit_kinematic_alignment.py --model envs/robots/urdf_f_link/URDF_F_link_named.xml",
        "```",
        "",
        "## 핵심 결과",
        "",
        f"- 모델: `{model_path}`",
        f"- 좌우 foot body 거리: `{foot_body_pair['distance_m']:.6f}` m",
        f"- 좌우 foot body delta L-R: `{foot_body_pair['delta_left_minus_right']}`",
        f"- 좌우 sole collision 거리: `{sole_pair['distance_m']:.6f}` m",
        f"- 좌우 sole collision delta L-R: `{sole_pair['delta_left_minus_right']}`",
        "",
        "## Body/Geom 위치",
        "",
        "| 항목 | 타입 | parent/body | local pos | world pos |",
        "| --- | --- | --- | --- | --- |",
    ]
    for record in records.values():
        owner = record.get("parent") or record.get("body")
        lines.append(
            f"| `{record['name']}` | `{record['type']}` | `{owner}` | `{record['local_pos']}` | `{record['world_pos']}` |"
        )
    lines.extend(
        [
            "",
            "## 좌우 Pair Delta",
            "",
            "| left | right | distance m | delta L-R |",
            "| --- | --- | ---: | --- |",
        ]
    )
    for pair in pairs:
        lines.append(
            f"| `{pair['left']}` | `{pair['right']}` | {pair['distance_m']:.6f} | `{pair['delta_left_minus_right']}` |"
        )
    lines.extend(
        [
            "",
            "## 산출물",
            "",
            f"- JSON: `{json_path}`",
            f"- body/geom CSV: `{body_csv}`",
            f"- pair delta CSV: `{pair_csv}`",
            f"- joint axis CSV: `{joint_csv}`",
            "",
            "## 판단",
            "",
            "좌우 foot body는 서로 떨어져 있지만, foot mesh와 sole collision의 local offset이 그 차이를 상쇄해서 최종 접촉 중심이 같은 world 위치로 겹친다. 따라서 현재 standing 실패는 단순히 RL 학습 부족으로 볼 수 없다.",
            "",
            "다음 단계에서는 CAD/Fusion 기준으로 foot link origin과 visual/collision origin이 body local 좌표인지, 또는 assembly absolute 좌표가 섞여 들어온 것인지 확인해야 한다. 확인 전에는 contact box를 임의로 옮겨서 최종 설계 검증에 사용하지 않는다.",
            "",
        ]
    )
    md_path.write_text("\n".join(lines), encoding="utf-8")

    print(f"Wrote {json_path}")
    print(f"Wrote {body_csv}")
    print(f"Wrote {pair_csv}")
    print(f"Wrote {joint_csv}")
    print(f"Wrote {md_path}")
    print(json.dumps({"foot_body_distance_m": foot_body_pair["distance_m"], "sole_collision_distance_m": sole_pair["distance_m"]}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
