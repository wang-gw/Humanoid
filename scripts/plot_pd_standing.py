from __future__ import annotations

import argparse
import csv
from pathlib import Path

import matplotlib.pyplot as plt


def read_csv(path: Path) -> list[dict[str, float]]:
    with path.open("r", newline="", encoding="utf-8") as handle:
        return [{key: float(value) for key, value in row.items()} for row in csv.DictReader(handle)]


def main() -> int:
    parser = argparse.ArgumentParser(description="Plot PD standing probe logs.")
    parser.add_argument("--csv", type=Path, default=Path("outputs/analysis/pd_standing/neutral_pd_standing.csv"))
    parser.add_argument("--out-dir", type=Path, default=Path("outputs/analysis/pd_standing"))
    args = parser.parse_args()

    rows = read_csv(args.csv.resolve())
    out_dir = args.out_dir.resolve()
    out_dir.mkdir(parents=True, exist_ok=True)
    time = [row["time"] for row in rows]
    tau_keys = sorted(key for key in rows[0] if key.startswith("tau_"))

    fig, axes = plt.subplots(3, 1, figsize=(11, 9), sharex=True)
    for key in tau_keys:
        axes[0].plot(time, [row[key] for row in rows], linewidth=1.0, label=key)
    axes[0].axhline(100.0, color="black", linewidth=0.8, linestyle="--")
    axes[0].axhline(-100.0, color="black", linewidth=0.8, linestyle="--")
    axes[0].set_ylabel("Torque (Nm)")
    axes[0].set_title("Joint Torque")
    axes[0].legend(ncol=5, fontsize=7)

    axes[1].plot(time, [row["base_z"] for row in rows], label="base_z")
    axes[1].plot(time, [row["base_roll"] for row in rows], label="roll_rad")
    axes[1].plot(time, [row["base_pitch"] for row in rows], label="pitch_rad")
    axes[1].set_ylabel("Base state")
    axes[1].set_title("Base Pose")
    axes[1].legend(fontsize=8)

    axes[2].plot(time, [row["contact_normal_force"] for row in rows], label="contact normal force")
    axes[2].plot(time, [row["qvel_norm"] for row in rows], label="qvel norm")
    axes[2].set_ylabel("Contact / velocity")
    axes[2].set_xlabel("Time (s)")
    axes[2].set_title("Contact and Velocity")
    axes[2].legend(fontsize=8)

    fig.tight_layout()
    plot_path = out_dir / "neutral_pd_standing_plot.png"
    fig.savefig(plot_path, dpi=160)
    plt.close(fig)
    print(f"Wrote {plot_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
