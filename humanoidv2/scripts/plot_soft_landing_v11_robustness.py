#!/usr/bin/env python3
"""Plot V11 scenario and asymmetry robustness summaries."""

from __future__ import annotations

import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
OUTPUT = ROOT / "results/khr3hv_v11_robustness"


def main() -> None:
    robustness = json.loads((OUTPUT / "robustness_summary.json").read_text())
    scenarios = robustness["scenario_aggregates"]
    names = [row["scenario"].replace("_", "\n") for row in scenarios]
    rates = [100.0 * row["success_rate"] for row in scenarios]
    colors = ["#2f9e44" if rate == 100.0 else "#f08c00" if rate >= 50.0 else "#e03131" for rate in rates]
    fig, ax = plt.subplots(figsize=(12, 5.5))
    bars = ax.bar(np.arange(len(names)), rates, color=colors)
    ax.bar_label(bars, fmt="%.0f%%", padding=3)
    ax.set_xticks(np.arange(len(names)), names, fontsize=8)
    ax.set_ylim(0, 110)
    ax.set_ylabel("Strict V10 success rate (%)")
    ax.set_title("V11 robustness by perturbation scenario")
    ax.grid(axis="y", alpha=0.25)
    fig.tight_layout()
    fig.savefig(OUTPUT / "robustness_success_rates.png", dpi=160)
    plt.close(fig)

    asymmetry = json.loads((OUTPUT / "asymmetry_sweep_summary.json").read_text())
    aggregates = asymmetry["aggregates"]
    x = np.array(asymmetry["asymmetries"]) * 100.0
    no_noise = [
        100.0 * next(
            row["success_rate"]
            for row in aggregates
            if row["leg_mass_asymmetry"] == value and row["noise_label"] == "none"
        )
        for value in asymmetry["asymmetries"]
    ]
    mild = [
        100.0 * next(
            row["success_rate"]
            for row in aggregates
            if row["leg_mass_asymmetry"] == value and row["noise_label"] == "mild"
        )
        for value in asymmetry["asymmetries"]
    ]
    fig, ax = plt.subplots(figsize=(8, 5))
    ax.plot(x, no_noise, "o-", label="No initial noise")
    ax.plot(x, mild, "s-", label="Mild initial noise")
    ax.axvline(0.0, color="black", linewidth=1, alpha=0.4)
    ax.set_ylim(-5, 105)
    ax.set_xlabel("Leg mass asymmetry a (%) — positive means left leg heavier")
    ax.set_ylabel("Strict V10 success rate (%)")
    ax.set_title("Directional sensitivity to left/right mass mismatch")
    ax.grid(alpha=0.25)
    ax.legend()
    fig.tight_layout()
    fig.savefig(OUTPUT / "asymmetry_success_rates.png", dpi=160)
    plt.close(fig)


if __name__ == "__main__":
    main()

