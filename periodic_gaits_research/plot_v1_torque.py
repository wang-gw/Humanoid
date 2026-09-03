#!/usr/bin/env python3
"""Plot applied and unclipped joint torque from a diagnose_v1 rollout CSV."""

from __future__ import annotations

import argparse
import csv
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


JOINTS = (
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
)
TORQUE_LIMITS = dict(zip(JOINTS, [24.0, 24.0, 24.0, 24.0, 7.0] * 2))


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("csv", type=Path)
    parser.add_argument("--output", type=Path, default=None)
    args = parser.parse_args()
    output = args.output or args.csv.with_name("torque_timeseries.png")

    with args.csv.open(newline="") as file:
        rows = list(csv.DictReader(file))
    time = np.asarray([float(row["time_s"]) for row in rows])

    figure, axes = plt.subplots(5, 2, figsize=(15, 14), sharex=True)
    for index, joint in enumerate(JOINTS):
        column = 0 if joint.startswith("left") else 1
        row_index = index if column == 0 else index - 5
        axis = axes[row_index, column]
        applied = np.asarray([float(row[f"{joint}_applied_torque_nm"]) for row in rows])
        unclipped = np.asarray([float(row[f"{joint}_unclipped_torque_nm"]) for row in rows])
        limit = TORQUE_LIMITS[joint]
        axis.plot(time, applied, color="#1565c0", linewidth=1.0, label="applied")
        axis.plot(time, unclipped, color="#ef6c00", linewidth=0.8, alpha=0.65, linestyle="--", label="unclipped")
        axis.axhline(limit, color="#c62828", linewidth=0.8, linestyle=":", label="limit")
        axis.axhline(-limit, color="#c62828", linewidth=0.8, linestyle=":")
        axis.set_title(joint.replace("_", " "))
        axis.set_ylabel("Torque [Nm]")
        axis.grid(alpha=0.25)
        if row_index == 0:
            axis.legend(loc="upper right", ncols=3, fontsize=8)
    axes[-1, 0].set_xlabel("Time [s]")
    axes[-1, 1].set_xlabel("Time [s]")
    figure.suptitle("V1 joint torque over time — deterministic rollout, seed 17", fontsize=15)
    figure.tight_layout(rect=(0, 0, 1, 0.98))
    output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(output, dpi=180)
    plt.close(figure)
    print(output.resolve())


if __name__ == "__main__":
    main()
