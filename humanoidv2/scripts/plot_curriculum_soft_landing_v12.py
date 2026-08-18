#!/usr/bin/env python3
"""Plot V12 checkpoint validation degradation."""

from __future__ import annotations

import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
OUTPUT = ROOT / "results/khr3hv_v12_curriculum"


def main() -> None:
    summary = json.loads((OUTPUT / "training_summary.json").read_text())
    scores = summary["candidate_scores"]
    labels = ["V10 initial", "stage 0", "stage 1", "stage 2"]
    successes = [row["total_successes"] for row in scores]
    falls = [row["falls"] for row in scores]
    nominal = [row["nominal_successes"] for row in scores]
    x = np.arange(len(labels))
    width = 0.35
    fig, ax = plt.subplots(figsize=(8, 5))
    success_bars = ax.bar(x - width / 2, successes, width, label="Strict successes / 40", color="#2f9e44")
    fall_bars = ax.bar(x + width / 2, falls, width, label="Falls / 40", color="#e03131")
    ax.bar_label(success_bars, padding=3)
    ax.bar_label(fall_bars, padding=3)
    for index, value in enumerate(nominal):
        ax.text(index, 43.5, f"nominal {value}/2", ha="center", va="top", fontsize=9)
    ax.set_xticks(x, labels)
    ax.set_ylim(0, 46)
    ax.set_ylabel("Validation runs")
    ax.set_title("V12 curriculum PPO checkpoint validation")
    ax.legend(loc="upper center", ncols=2)
    ax.grid(axis="y", alpha=0.25)
    fig.tight_layout()
    fig.savefig(OUTPUT / "checkpoint_validation.png", dpi=160)
    plt.close(fig)


if __name__ == "__main__":
    main()
