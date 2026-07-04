from __future__ import annotations

import argparse
import csv
import json
import subprocess
from pathlib import Path


def run_json(command: list[str]) -> dict[str, object]:
    completed = subprocess.run(command, check=True, capture_output=True, text=True)
    return json.loads(completed.stdout)


def tag_float(value: float) -> str:
    return f"{value:.2f}".replace(".", "p")


def main() -> int:
    parser = argparse.ArgumentParser(description="Sweep virtual foot support dimensions against one-leg support gate.")
    parser.add_argument("--source-model", type=Path, required=True)
    parser.add_argument("--pose", type=Path, required=True)
    parser.add_argument("--out-dir", type=Path, required=True)
    parser.add_argument("--duration", type=float, default=6.0)
    parser.add_argument("--target-gate-time", type=float, default=0.5)
    parser.add_argument("--variants", choices=("coarse", "focused", "xscan"), default="coarse")
    args = parser.parse_args()

    out_dir = args.out_dir.resolve()
    out_dir.mkdir(parents=True, exist_ok=True)
    model_dir = Path("envs/robots/urdf_f_link").resolve()

    if args.variants == "xscan":
        variants = [
            ("long_x115", 1.15, 1.0),
            ("long_x120", 1.20, 1.0),
            ("long_x125", 1.25, 1.0),
            ("long_x130", 1.30, 1.0),
            ("long_x135", 1.35, 1.0),
            ("long_x140", 1.40, 1.0),
        ]
    elif args.variants == "focused":
        variants = [
            ("baseline", 1.0, 1.0),
            ("wide_y125", 1.0, 1.25),
            ("wide_y150", 1.0, 1.50),
            ("long_x125", 1.25, 1.0),
            ("long_x150", 1.50, 1.0),
            ("both_x125_y125", 1.25, 1.25),
            ("both_x150_y150", 1.50, 1.50),
        ]
    else:
        variants = [
            ("baseline", 1.0, 1.0),
            ("wide_y125", 1.0, 1.25),
            ("wide_y150", 1.0, 1.50),
            ("wide_y175", 1.0, 1.75),
            ("long_x125", 1.25, 1.0),
            ("long_x150", 1.50, 1.0),
            ("both_x125_y125", 1.25, 1.25),
            ("both_x150_y150", 1.50, 1.50),
        ]

    rows: list[dict[str, object]] = []
    for label, scale_x, scale_y in variants:
        model_path = model_dir / f"URDF_F_link_virtual_design_gate_{label}.xml"
        build_report = out_dir / f"{label}_build_report.json"
        audit_dir = out_dir / f"{label}_com_support"
        gate_dir = out_dir / f"{label}_one_leg_gate"

        build = run_json(
            [
                "python3",
                "scripts/build_virtual_design_variant.py",
                "--source",
                str(args.source_model),
                "--out",
                str(model_path),
                "--report",
                str(build_report),
                "--sole-scale-x",
                str(scale_x),
                "--sole-scale-y",
                str(scale_y),
            ]
        )
        audit = run_json(
            [
                "python3",
                "scripts/audit_com_support_polygon.py",
                "--model",
                str(model_path),
                "--pose-json",
                str(args.pose),
                "--out-dir",
                str(audit_dir),
            ]
        )
        gate = run_json(
            [
                "python3",
                "scripts/search_one_leg_support_gate.py",
                "--model",
                str(model_path),
                "--pose",
                str(args.pose),
                "--out-dir",
                str(gate_dir),
                "--duration",
                str(args.duration),
                "--target-gate-time",
                str(args.target_gate_time),
                "--right-force-gate",
                "5.0",
                "--right-contact-gate",
                "0.0",
                "--max-roll-gate",
                "0.12",
                "--joint-kp",
                "20",
                "--joint-kd",
                "12",
                "--torque-limit",
                "30",
                "--kp-att",
                "0",
                "--kd-att",
                "1",
                "--kcom",
                "1",
                "--roll-sign",
                "-1",
                "--pitch-sign",
                "1",
                "--force-sign",
                "1",
                "--narrow",
            ]
        )
        best = gate["best"]
        left_support = audit["support"]["left"]
        row = {
            "label": label,
            "scale_x": scale_x,
            "scale_y": scale_y,
            "model": str(model_path),
            "left_area_m2": left_support["area_m2"],
            "left_area_cm2": float(left_support["area_m2"]) * 10000.0,
            "left_margin_mm": float(left_support["polygon_margin"]["signed_margin"]) * 1000.0,
            "gate_time_s": best["gate_time_s"],
            "low_force_time_s": best["low_force_time_s"],
            "no_contact_time_s": best["no_contact_time_s"],
            "max_abs_roll": best["max_abs_roll"],
            "max_abs_pitch": best["max_abs_pitch"],
            "max_abs_tau": best["max_abs_tau"],
            "saturation_fraction": best["saturation_fraction"],
            "target_left_ratio": best["target_left_ratio"],
            "kforce": best["kforce"],
            "final_left_ratio": best["final_left_ratio"],
            "final_right_force": best["final_right_force"],
            "final_right_contacts": best["final_right_contacts"],
            "gate_pass": float(best["gate_time_s"]) >= args.target_gate_time,
            "build_report": str(build_report),
            "audit_json": str(audit_dir / "com_support_polygon_audit.json"),
            "gate_summary": str(gate_dir / "one_leg_support_gate_summary.json"),
            "compiled_ngeom": build["compiled"]["ngeom"],
        }
        rows.append(row)

    csv_path = out_dir / "virtual_support_design_gate_sweep.csv"
    with csv_path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)

    best_row = max(rows, key=lambda row: float(row["gate_time_s"]))
    first_pass = next((row for row in rows if row["gate_pass"]), None)
    summary = {
        "source_model": str(args.source_model.resolve()),
        "pose": str(args.pose.resolve()),
        "duration": args.duration,
        "target_gate_time": args.target_gate_time,
        "rows": rows,
        "best": best_row,
        "first_pass": first_pass,
        "csv": str(csv_path),
    }
    summary_path = out_dir / "virtual_support_design_gate_sweep_summary.json"
    summary_path.write_text(json.dumps(summary, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(json.dumps(summary, indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
