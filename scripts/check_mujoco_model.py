from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

import mujoco


def obj_name(model: mujoco.MjModel, obj_type: int, idx: int, fallback: str) -> str:
    name = mujoco.mj_id2name(model, obj_type, idx)
    return name if name else fallback


def main() -> int:
    parser = argparse.ArgumentParser(description="Compile a MuJoCo model and document key readiness checks.")
    parser.add_argument("--model", type=Path, default=Path("envs/robots/urdf_f/URDF_F_mujoco.urdf"))
    parser.add_argument("--out-dir", type=Path, default=Path("docs/hardware_validation"))
    args = parser.parse_args()

    model_path = args.model.resolve()
    out_dir = args.out_dir.resolve()
    out_dir.mkdir(parents=True, exist_ok=True)

    model = mujoco.MjModel.from_xml_path(str(model_path))
    joint_names = [
        obj_name(model, mujoco.mjtObj.mjOBJ_JOINT, idx, f"joint_{idx}") for idx in range(model.njnt)
    ]
    body_names = [
        obj_name(model, mujoco.mjtObj.mjOBJ_BODY, idx, f"body_{idx}") for idx in range(model.nbody)
    ]
    actuator_names = [
        obj_name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, idx, f"actuator_{idx}") for idx in range(model.nu)
    ]

    findings: list[str] = []
    if model.nu == 0:
        findings.append("No MuJoCo actuators are present. RL cannot command this model yet.")
    if model.nq == model.nv == model.njnt:
        findings.append(
            "Model loaded as a fixed-base mechanism. Walking RL needs a floating base/freejoint model."
        )
    if model.ngeom > 0 and model.ngeom == len(body_names) * 2:
        findings.append("Geometry count suggests visual/collision meshes may still be duplicated.")

    payload: dict[str, Any] = {
        "model_path": str(model_path),
        "summary": {
            "nq": int(model.nq),
            "nv": int(model.nv),
            "nu": int(model.nu),
            "nbody": int(model.nbody),
            "njnt": int(model.njnt),
            "ngeom": int(model.ngeom),
            "timestep": float(model.opt.timestep),
        },
        "findings": findings,
        "joint_names": joint_names,
        "body_names": body_names,
        "actuator_names": actuator_names,
    }

    json_path = out_dir / "mujoco_load_report.json"
    md_path = out_dir / "mujoco_load_report.md"
    json_path.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")

    lines = [
        "# MuJoCo Load Report",
        "",
        "## Source",
        "",
        f"- Model file: `{model_path}`",
        "",
        "## Summary",
        "",
        f"- nq: `{model.nq}`",
        f"- nv: `{model.nv}`",
        f"- nu: `{model.nu}`",
        f"- bodies: `{model.nbody}`",
        f"- joints: `{model.njnt}`",
        f"- geoms: `{model.ngeom}`",
        f"- timestep: `{model.opt.timestep}`",
        "",
        "## Findings",
        "",
    ]
    lines.extend(f"- {finding}" for finding in findings) if findings else lines.append("- No load findings.")
    lines.extend(["", "## Joint Names", ""])
    lines.extend(f"- `{name}`" for name in joint_names)
    md_path.write_text("\n".join(lines) + "\n", encoding="utf-8")

    print(f"Wrote {json_path}")
    print(f"Wrote {md_path}")
    print(json.dumps(payload["summary"], indent=2))
    if findings:
        print("Findings:")
        for finding in findings:
            print(f"- {finding}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
