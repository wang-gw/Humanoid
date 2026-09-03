#!/usr/bin/env python3
"""Build comparable tables and plots from frequency-sweep diagnostics."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


def mean_metric(episodes: list[dict], key: str, statistic: str) -> float | None:
    if not episodes:
        return None
    return float(np.mean([row["reward_metrics"][key][statistic] for row in episodes]))


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("root", type=Path)
    parser.add_argument("--frequencies", type=float, nargs="+", default=[0.6, 0.8, 1.0, 1.2, 1.4])
    args = parser.parse_args()
    rows = []
    for frequency in args.frequencies:
        label = f"{frequency:.1f}".replace(".", "p") + "hz"
        path = args.root / label / "diagnostics/diagnostics_summary.json"
        data = json.loads(path.read_text())
        episodes = data["episodes"]
        successful = [episode for episode in episodes if episode["success"]]
        joints = data["aggregate"]["joints"]
        duration = [episode["duration_s"] for episode in successful]
        distance = [episode["forward_distance_m"] for episode in successful]
        rows.append({
            "frequency_hz": frequency,
            "runs": len(episodes),
            "successes": len(successful),
            "success_rate": len(successful) / len(episodes),
            "falls": sum(bool(episode["terminated"]) for episode in episodes),
            "mean_forward_distance_success_m": float(np.mean(distance)) if distance else None,
            "mean_forward_speed_success_m_s": float(np.mean(np.asarray(distance) / np.asarray(duration))) if distance else None,
            "lateral_tilt_rms_success_rad": mean_metric(successful, "lateral_tilt_rad", "rms"),
            "lateral_sway_p2p_success_m": mean_metric(successful, "lateral_position_m", "peak_to_peak"),
            "lateral_tilt_rate_rms_success_rad_s": mean_metric(successful, "lateral_tilt_rate_rad_s", "rms"),
            "sagittal_tilt_rms_success_rad": mean_metric(successful, "sagittal_tilt_rad", "rms"),
            "mean_action_saturation_fraction": float(np.mean([
                joint["action_saturation_fraction"] for joint in joints.values()
            ])),
            "mean_torque_clip_fraction": float(np.mean([
                joint["torque_clip_fraction"] for joint in joints.values()
            ])),
        })

    output_json = args.root / "frequency_sweep_summary.json"
    output_csv = args.root / "frequency_sweep_summary.csv"
    output_json.write_text(json.dumps(rows, indent=2))
    with output_csv.open("w", newline="") as file:
        writer = csv.DictWriter(file, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)

    frequencies = np.asarray([row["frequency_hz"] for row in rows])
    figure, axes = plt.subplots(2, 2, figsize=(12, 8), sharex=True)
    axes[0, 0].plot(frequencies, [row["success_rate"] for row in rows], marker="o")
    axes[0, 0].set_ylabel("Success rate")
    axes[0, 0].set_ylim(-0.05, 1.05)
    metrics = (
        (axes[0, 1], "lateral_tilt_rms_success_rad", "Lateral tilt RMS [rad]"),
        (axes[1, 0], "lateral_sway_p2p_success_m", "Lateral sway p-p [m]"),
        (axes[1, 1], "lateral_tilt_rate_rms_success_rad_s", "Lateral tilt-rate RMS [rad/s]"),
    )
    for axis, key, label in metrics:
        values = [np.nan if row[key] is None else row[key] for row in rows]
        axis.plot(frequencies, values, marker="o")
        axis.set_ylabel(label)
        axis.set_xlabel("Gait frequency [Hz]")
    for axis in axes.flat:
        axis.grid(alpha=0.3)
    figure.suptitle("V1 gait-frequency sweep — 30 deterministic seeds")
    figure.tight_layout(rect=(0, 0, 1, 0.96))
    figure.savefig(args.root / "frequency_sweep_summary.png", dpi=180)
    plt.close(figure)
    print(json.dumps(rows, indent=2))


if __name__ == "__main__":
    main()
