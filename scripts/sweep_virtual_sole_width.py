from __future__ import annotations

import argparse
import csv
import json
import subprocess
from pathlib import Path


def run_json(command: list[str]) -> dict[str, object]:
    completed = subprocess.run(command, check=True, capture_output=True, text=True)
    return json.loads(completed.stdout)


def main() -> int:
    parser = argparse.ArgumentParser(description="Sweep virtual sole Y scale and collect unload gate metrics.")
    parser.add_argument("--source-model", type=Path, required=True)
    parser.add_argument("--pose", type=Path, required=True)
    parser.add_argument("--out-dir", type=Path, required=True)
    parser.add_argument("--scales", nargs="+", type=float, required=True)
    parser.add_argument("--target-left-ratio", type=float, default=0.78)
    parser.add_argument("--kforce", type=float, default=5.0)
    parser.add_argument("--duration", type=float, default=8.0)
    parser.add_argument("--ramp", type=float, default=3.0)
    parser.add_argument("--hold", type=float, default=5.0)
    args = parser.parse_args()

    out_dir = args.out_dir.resolve()
    out_dir.mkdir(parents=True, exist_ok=True)
    model_dir = Path("envs/robots/urdf_f_link").resolve()
    rows: list[dict[str, object]] = []

    for scale in args.scales:
        tag = f"y{scale:.2f}".replace(".", "p")
        model_path = model_dir / f"URDF_F_link_virtual_sweep_sole_{tag}.xml"
        build_report = out_dir / f"sole_{tag}_build_report.json"
        audit_dir = out_dir / f"com_support_{tag}"
        render_dir = out_dir / f"force_ratio_{tag}_target{args.target_left_ratio:.2f}".replace(".", "p")

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
                "--sole-scale-y",
                str(scale),
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
        render = run_json(
            [
                "python3",
                "scripts/render_force_ratio_sequence.py",
                "--model",
                str(model_path),
                "--poses",
                str(args.pose),
                "--target-left-ratios",
                str(args.target_left_ratio),
                "--out-dir",
                str(render_dir),
                "--duration",
                str(args.duration),
                "--ramp",
                str(args.ramp),
                "--hold",
                str(args.hold),
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
                "--kforce",
                str(args.kforce),
                "--force-sign",
                "1",
                "--force-role",
                "both",
                "--force-on",
                "all",
                "--force-side-mode",
                "same",
                "--fps",
                "10",
                "--azimuth",
                "135",
                "--elevation",
                "-12",
                "--distance",
                "1.45",
            ]
        )

        final = render["final"]
        left_margin = audit["support"]["left"]["polygon_margin"]["signed_margin"]
        gate_pass = (
            final["left_force_ratio"] >= 0.70
            and final["right_force"] <= 25.0
            and render["max_abs_roll"] <= 0.22
            and abs(final["roll"]) <= 0.16
            and render["max_abs_tau"] < 30.0
            and render["saturation_fraction"] == 0.0
        )
        rows.append(
            {
                "scale_y": scale,
                "model": str(model_path),
                "left_support_margin_m": left_margin,
                "left_support_margin_mm": left_margin * 1000.0,
                "final_left_ratio": final["left_force_ratio"],
                "final_right_force_n": final["right_force"],
                "final_roll_rad": final["roll"],
                "max_roll_rad": render["max_abs_roll"],
                "max_abs_tau_nm": render["max_abs_tau"],
                "saturation_fraction": render["saturation_fraction"],
                "max_qvel_norm": render["max_qvel_norm"],
                "gate_pass": gate_pass,
                "mp4": render["mp4"],
                "audit_json": str(audit_dir / "com_support_polygon_audit.json"),
                "build_report": str(build_report),
                "compiled_ngeom": build["compiled"]["ngeom"],
            }
        )

    csv_path = out_dir / "virtual_sole_width_sweep.csv"
    with csv_path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)

    summary = {
        "source_model": str(args.source_model.resolve()),
        "pose": str(args.pose.resolve()),
        "target_left_ratio": args.target_left_ratio,
        "kforce": args.kforce,
        "rows": rows,
        "first_gate_pass": next((row for row in rows if row["gate_pass"]), None),
        "csv": str(csv_path),
    }
    summary_path = out_dir / "virtual_sole_width_sweep_summary.json"
    summary_path.write_text(json.dumps(summary, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(json.dumps(summary, indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
