from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import mujoco
import numpy as np

from render_pd_standing import hinge_joint_info, load_targets
from search_right_foot_lift_trajectory import simulate


FULLBODY_JOINT_RANGES = {
    "left_hip_roll": (-0.10, 0.10),
    "left_hip_pitch": (-0.08, 0.08),
    "left_knee_pitch": (-0.08, 0.08),
    "left_ankle_pitch": (-0.08, 0.08),
    "left_ankle_roll": (-0.10, 0.10),
    "right_hip_roll": (-0.16, 0.16),
    "right_hip_pitch": (-0.18, 0.10),
    "right_knee_pitch": (-0.25, 0.04),
    "right_ankle_pitch": (-0.05, 0.25),
    "right_ankle_roll": (-0.16, 0.16),
}


SEED_PATTERNS = [
    {
        "left_hip_roll": 0.04,
        "left_ankle_roll": -0.04,
        "right_hip_roll": -0.10,
        "right_hip_pitch": -0.04,
        "right_knee_pitch": -0.08,
        "right_ankle_pitch": 0.07,
        "right_ankle_roll": 0.14,
    },
    {
        "left_hip_roll": 0.06,
        "left_ankle_roll": -0.06,
        "right_hip_roll": -0.12,
        "right_hip_pitch": -0.02,
        "right_knee_pitch": -0.10,
        "right_ankle_pitch": 0.09,
        "right_ankle_roll": 0.12,
    },
    {
        "left_hip_pitch": 0.04,
        "left_knee_pitch": -0.04,
        "left_ankle_pitch": 0.04,
        "right_hip_roll": -0.08,
        "right_hip_pitch": -0.08,
        "right_knee_pitch": -0.14,
        "right_ankle_pitch": 0.12,
        "right_ankle_roll": 0.10,
    },
]


def filled_offsets(values: dict[str, float]) -> dict[str, float]:
    return {name: float(values.get(name, 0.0)) for name in FULLBODY_JOINT_RANGES}


def main() -> int:
    parser = argparse.ArgumentParser(description="Search full-body coordinated right-foot lift trajectory.")
    parser.add_argument("--model", type=Path, required=True)
    parser.add_argument("--seed-pose", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--out-dir", type=Path, required=True)
    parser.add_argument("--samples", type=int, default=240)
    parser.add_argument("--seed", type=int, default=701)
    parser.add_argument("--duration", type=float, default=8.0)
    parser.add_argument("--lift-ramp", type=float, default=2.0)
    parser.add_argument("--lift-hold", type=float, default=2.0)
    parser.add_argument("--return-ramp", type=float, default=2.0)
    parser.add_argument("--target-clearance", type=float, default=0.002)
    parser.add_argument("--target-clearance-time", type=float, default=0.5)
    parser.add_argument("--target-left-ratio", type=float, default=0.78)
    parser.add_argument("--right-force-gate", type=float, default=15.0)
    parser.add_argument("--right-contact-gate", type=float, default=1.0)
    parser.add_argument("--max-roll-gate", type=float, default=0.18)
    parser.add_argument("--joint-kp", type=float, default=20.0)
    parser.add_argument("--joint-kd", type=float, default=12.0)
    parser.add_argument("--torque-limit", type=float, default=30.0)
    parser.add_argument("--kp-att", type=float, default=0.0)
    parser.add_argument("--kd-att", type=float, default=1.0)
    parser.add_argument("--kcom", type=float, default=1.0)
    parser.add_argument("--roll-sign", type=float, default=-1.0)
    parser.add_argument("--pitch-sign", type=float, default=1.0)
    parser.add_argument("--kforce", type=float, default=5.0)
    parser.add_argument("--force-sign", type=float, default=1.0)
    args = parser.parse_args()

    out_dir = args.out_dir.resolve()
    out_dir.mkdir(parents=True, exist_ok=True)
    model = mujoco.MjModel.from_xml_path(str(args.model.resolve()))
    joints = hinge_joint_info(model)
    joint_names = [str(joint["name"]) for joint in joints]
    actuator_names = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, idx) or f"actuator_{idx}" for idx in range(model.nu)]
    base_z, seed_q = load_targets(model, joints, args.seed_pose.resolve())
    rng = np.random.default_rng(args.seed)

    candidate_offsets = [filled_offsets({}), *[filled_offsets(pattern) for pattern in SEED_PATTERNS]]
    for _ in range(args.samples):
        candidate_offsets.append({name: float(rng.uniform(lo, hi)) for name, (lo, hi) in FULLBODY_JOINT_RANGES.items()})

    rows = []
    best = None
    for offsets in candidate_offsets:
        lift_q = seed_q.copy()
        for name, value in offsets.items():
            if name in joint_names:
                lift_q[joint_names.index(name)] += float(value)
        metrics = simulate(model, joints, actuator_names, base_z, seed_q, lift_q, args)
        stable_time_error = max(0.0, args.target_clearance_time - metrics["stable_clearance_time_s"])
        score = (
            500.0 * stable_time_error
            - 55.0 * metrics["stable_clearance_time_s"]
            - 6.0 * metrics["low_force_time_s"]
            - 2.0 * metrics["low_contact_time_s"]
            + 180.0 * max(0.0, metrics["max_abs_roll"] - args.max_roll_gate)
            + 100.0 * metrics["saturation_fraction"]
            + 0.04 * metrics["max_abs_tau"]
            + 25.0 * max(0.0, metrics["final_right_force"] - 25.0) / 90.0
        )
        row = {"score": float(score), **offsets, **metrics}
        rows.append(row)
        if best is None or row["score"] < best["score"]:
            best = row

    assert best is not None
    best_q = seed_q.copy()
    for name in FULLBODY_JOINT_RANGES:
        if name in joint_names:
            best_q[joint_names.index(name)] += float(best[name])
    seed_payload = json.loads(args.seed_pose.read_text(encoding="utf-8"))
    targets = dict(seed_payload.get("joint_targets", {}))
    for idx, name in enumerate(joint_names):
        targets[name] = float(best_q[idx])

    search_args = {key: (str(value) if isinstance(value, Path) else value) for key, value in vars(args).items()}
    out_payload = {
        "base_z": float(base_z),
        "joint_targets": targets,
        "source": {
            "seed_pose": str(args.seed_pose.resolve()),
            "model": str(args.model.resolve()),
            "search": search_args,
            "best": best,
        },
    }
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(out_payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")

    csv_path = out_dir / "right_foot_lift_fullbody_trajectory_candidates.csv"
    with csv_path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(sorted(rows, key=lambda row: row["score"]))
    summary = {"best": best, "out_pose": str(args.out.resolve()), "csv": str(csv_path), "samples": len(rows)}
    (out_dir / "right_foot_lift_fullbody_trajectory_search_summary.json").write_text(
        json.dumps(summary, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    print(json.dumps(summary, indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
