from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import mujoco
import numpy as np


def main() -> int:
    parser = argparse.ArgumentParser(description="Run a short zero-control MuJoCo smoke probe.")
    parser.add_argument("--model", type=Path, default=Path("envs/robots/urdf_f/URDF_F_mujoco.xml"))
    parser.add_argument("--duration", type=float, default=1.0)
    parser.add_argument("--base-z", type=float, default=0.0)
    parser.add_argument("--out-dir", type=Path, default=Path("outputs/analysis/smoke_probe"))
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
        data.qpos[2] = float(args.base_z)
        data.qpos[3:7] = np.array([1.0, 0.0, 0.0, 0.0])
    mujoco.mj_forward(model, data)

    steps = int(max(args.duration, 0.0) / model.opt.timestep)
    rows: list[dict[str, float]] = []
    for step in range(steps + 1):
        rows.append(
            {
                "step": float(step),
                "time": float(data.time),
                "base_x": float(data.qpos[0]) if model.nq >= 1 else 0.0,
                "base_y": float(data.qpos[1]) if model.nq >= 2 else 0.0,
                "base_z": float(data.qpos[2]) if model.nq >= 3 else 0.0,
                "contacts": float(data.ncon),
                "qvel_norm": float(np.linalg.norm(data.qvel)),
                "ctrl_norm": float(np.linalg.norm(data.ctrl)) if model.nu else 0.0,
                "warning": float(data.warning[0].number) if len(data.warning) else 0.0,
            }
        )
        if step < steps:
            data.ctrl[:] = 0.0
            mujoco.mj_step(model, data)

    csv_path = out_dir / "zero_control_smoke.csv"
    with csv_path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)

    summary = {
        "model_path": str(model_path),
        "duration": args.duration,
        "timestep": float(model.opt.timestep),
        "steps": steps,
        "initial_base_z": rows[0]["base_z"],
        "final_base_z": rows[-1]["base_z"],
        "max_contacts": max(row["contacts"] for row in rows),
        "max_qvel_norm": max(row["qvel_norm"] for row in rows),
        "csv_path": str(csv_path),
    }
    json_path = out_dir / "zero_control_smoke_summary.json"
    json_path.write_text(json.dumps(summary, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")

    md_path = doc_dir / "03_zero_control_smoke_probe.md"
    md_path.write_text(
        "\n".join(
            [
                "# Zero-Control Smoke Probe",
                "",
                "## Purpose",
                "",
                "Actuator가 달린 MJCF가 짧은 MuJoCo rollout에서 수치적으로 실행되는지 확인한다. 이 probe는 보행 가능성 판단이 아니라, standing/squat controller를 만들기 전의 최소 실행성 확인이다.",
                "",
                "## Command",
                "",
                "```bash",
                "python3 scripts/probe_mujoco_smoke.py",
                "```",
                "",
                "## Summary",
                "",
                f"- Model: `{model_path}`",
                f"- Duration: `{args.duration}` sec",
                f"- Timestep: `{model.opt.timestep}`",
                f"- Steps: `{steps}`",
                f"- Initial base z: `{summary['initial_base_z']:.6f}`",
                f"- Final base z: `{summary['final_base_z']:.6f}`",
                f"- Max contacts: `{summary['max_contacts']:.0f}`",
                f"- Max qvel norm: `{summary['max_qvel_norm']:.6f}`",
                "",
                "## Outputs",
                "",
                f"- CSV: `{csv_path}`",
                f"- JSON: `{json_path}`",
                "",
                "## Interpretation",
                "",
                "Zero-control 상태에서 로봇이 균형을 잡는 것은 기대하지 않는다. 다음 단계에서는 neutral standing pose와 PD controller를 정의해 넘어짐이 제어 문제인지 형상 문제인지 분리한다.",
                "",
            ]
        ),
        encoding="utf-8",
    )

    print(f"Wrote {csv_path}")
    print(f"Wrote {json_path}")
    print(f"Wrote {md_path}")
    print(json.dumps(summary, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
