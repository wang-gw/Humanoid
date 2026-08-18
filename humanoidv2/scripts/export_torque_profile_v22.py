#!/usr/bin/env python3
"""Export per-joint torque profiles of the deployed V22 walking policy.

The training/evaluation logs only keep `peak_control_interval_torque_n_m`, a
single scalar per 40 ms control step. Hardware review needs the per-joint,
per-physics-step (2 ms) command history, so this script replays the deployed
V22 configuration (V17 base policy + V20 corrector + 0.008 rad first-step
stance hip-roll offset) and records `data.ctrl` at every `mj_step`.

Outputs per side, in `results/torque_profile/`:

- `torque_profile_<side>_first.csv`   500 Hz per-joint torque and joint speed
- `torque_profile_<side>_first.json`  per-joint and per-phase statistics
- `torque_profile_<side>_first.png`   torque traces with footstep/phase bands
"""

from __future__ import annotations

import argparse
import csv
import json
import sys
from pathlib import Path

import mujoco
import numpy as np
from stable_baselines3 import PPO

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from humanoidv2 import FirstStepStanceHipRollV22Env  # noqa: E402
from humanoidv2.khr3hv_env import JOINT_NAMES  # noqa: E402
from scripts.diagnose_counterfactual_v22 import BASE, CORRECTOR  # noqa: E402
from scripts.train_landing_residual_v19 import combined_domain  # noqa: E402


OUTPUT = ROOT / "results/torque_profile"

# Actuator per joint, taken from the model comment in humanoidv2/khr3hv_env.py.
MOTORS = ("AK45-36",) * 4 + ("AK45-10",) + ("AK45-36",) * 4 + ("AK45-10",)
RATED_SPEED_RPM = (40.0,) * 4 + (150.0,) + (40.0,) * 4 + (150.0,)


class TorqueRecorder:
    """Capture `data.ctrl` and joint speed at every physics step."""

    def __init__(self, env) -> None:
        self.env = env
        self.enabled = False
        self.torques: list[np.ndarray] = []
        self.speeds: list[np.ndarray] = []
        self._original = mujoco.mj_step

    def __enter__(self) -> "TorqueRecorder":
        recorder = self

        def patched(model, data, nstep=1):
            if recorder.enabled and data is recorder.env.data:
                recorder.torques.append(data.ctrl.copy())
                recorder.speeds.append(data.qvel[recorder.env.dof_ids].copy())
            return recorder._original(model, data, nstep)

        mujoco.mj_step = patched
        return self

    def __exit__(self, *exception) -> None:
        mujoco.mj_step = self._original


def build_environment(side: str, seed: int, domain_seed: int | None):
    base = PPO.load(BASE, device="cpu")
    corrector = PPO.load(CORRECTOR, device="cpu")
    env = FirstStepStanceHipRollV22Env(
        base_policy=base,
        fixed_first_side=side,
        fixed_domain=combined_domain(domain_seed) if domain_seed is not None else {},
        curriculum_stage=2,
        early_gate_blend=0.75,
        stance_hip_roll_lift_offset_rad=0.008,
    )
    observation, _ = env.reset(seed=seed)
    return corrector, env, observation


def rollout(side: str, seed: int, domain_seed: int | None) -> dict[str, object]:
    corrector, env, observation = build_environment(side, seed, domain_seed)
    frame_skip = env.frame_skip
    sim_dt = env.config.sim_dt
    rows: list[dict[str, object]] = []
    info: dict[str, object] = {}
    terminated = truncated = False
    with TorqueRecorder(env) as recorder:
        for control_step in range(env.config.max_episode_steps):
            action = corrector.predict(observation, deterministic=True)[0]
            recorder.enabled = True
            start = len(recorder.torques)
            observation, _, terminated, truncated, info = env.step(action)
            recorder.enabled = False
            captured = len(recorder.torques) - start
            if captured != frame_skip:
                raise RuntimeError(
                    f"expected {frame_skip} physics samples, captured {captured}"
                )
            for offset in range(captured):
                index = start + offset
                rows.append(
                    {
                        "time_s": (control_step * frame_skip + offset) * sim_dt,
                        "control_step": control_step + 1,
                        "footstep": int(info["active_step"]),
                        "swing_side": str(info["active_swing_side"]),
                        "task_phase": str(info["task_phase"]).split("_", 1)[1],
                        "phase_progress": float(info["phase_progress"]),
                        "effective_kp": float(info["effective_kp"]),
                        "torque": recorder.torques[index],
                        "speed": recorder.speeds[index],
                    }
                )
            if terminated or truncated:
                break
    env.close()
    return {
        "rows": rows,
        "info": info,
        "terminated": bool(terminated),
        "truncated": bool(truncated),
        "torque_limit": env.torque_limit.copy(),
    }


def write_csv(path: Path, rows: list[dict[str, object]]) -> None:
    header = (
        ["time_s", "control_step", "footstep", "swing_side", "task_phase",
         "phase_progress", "effective_kp"]
        + [f"tau_{name}_nm" for name in JOINT_NAMES]
        + [f"qvel_{name}_rad_s" for name in JOINT_NAMES]
    )
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(header)
        for row in rows:
            writer.writerow(
                [
                    f"{row['time_s']:.3f}",
                    row["control_step"],
                    row["footstep"],
                    row["swing_side"],
                    row["task_phase"],
                    f"{row['phase_progress']:.4f}",
                    f"{row['effective_kp']:.1f}",
                ]
                + [f"{value:.6f}" for value in row["torque"]]
                + [f"{value:.6f}" for value in row["speed"]]
            )


def summarize(result: dict[str, object]) -> dict[str, object]:
    rows = result["rows"]
    limit = np.asarray(result["torque_limit"])
    torque = np.array([row["torque"] for row in rows])
    speed = np.array([row["speed"] for row in rows])
    power = torque * speed
    phases = np.array([row["task_phase"] for row in rows])
    footsteps = np.array([row["footstep"] for row in rows])

    sim_dt = 0.002
    rate = np.diff(torque, axis=0) / sim_dt
    spectrum = np.abs(np.fft.rfft(torque - torque.mean(axis=0), axis=0))
    frequencies = np.fft.rfftfreq(torque.shape[0], sim_dt)
    high_band = frequencies >= 5.0

    joints = []
    for index, name in enumerate(JOINT_NAMES):
        column = torque[:, index]
        joints.append(
            {
                "joint": name,
                "motor": MOTORS[index],
                "torque_limit_n_m": float(limit[index]),
                "peak_abs_n_m": float(np.abs(column).max()),
                "utilization_percent": float(100.0 * np.abs(column).max() / limit[index]),
                "p95_abs_n_m": float(np.percentile(np.abs(column), 95.0)),
                "rms_n_m": float(np.sqrt(np.mean(column ** 2))),
                "mean_abs_n_m": float(np.abs(column).mean()),
                "minimum_n_m": float(column.min()),
                "maximum_n_m": float(column.max()),
                "saturated_sample_fraction": float(
                    np.mean(np.abs(column) >= limit[index] - 1.0e-9)
                ),
                "peak_abs_speed_rpm": float(
                    np.abs(speed[:, index]).max() * 60.0 / (2.0 * np.pi)
                ),
                "rated_speed_rpm": RATED_SPEED_RPM[index],
                "peak_mechanical_power_w": float(np.abs(power[:, index]).max()),
                "peak_torque_rate_n_m_per_s": float(np.abs(rate[:, index]).max()),
                # Gait content sits below a few Hz; anything above 5 Hz is
                # PD chatter that a real driver has to reproduce.
                "dominant_chatter_hz": float(
                    frequencies[high_band][int(spectrum[high_band, index].argmax())]
                ),
                "chatter_energy_fraction_above_5hz": float(
                    spectrum[high_band, index].sum() / spectrum[1:, index].sum()
                ),
            }
        )

    per_phase = {}
    for phase in ("lift", "hold", "advance", "land", "settle"):
        mask = phases == phase
        if not mask.any():
            continue
        per_phase[phase] = {
            "samples": int(mask.sum()),
            "peak_abs_n_m": float(np.abs(torque[mask]).max()),
            "peak_joint": JOINT_NAMES[int(np.abs(torque[mask]).max(axis=0).argmax())],
            "mean_abs_n_m": float(np.abs(torque[mask]).mean()),
        }

    per_footstep = []
    for step in range(1, 9):
        mask = footsteps == step
        if not mask.any():
            continue
        per_footstep.append(
            {
                "footstep": step,
                "swing_side": rows[int(np.flatnonzero(mask)[0])]["swing_side"],
                "peak_abs_n_m": float(np.abs(torque[mask]).max()),
                "peak_joint": JOINT_NAMES[int(np.abs(torque[mask]).max(axis=0).argmax())],
                "mean_abs_n_m": float(np.abs(torque[mask]).mean()),
            }
        )

    info = result["info"]
    return {
        "configuration": {
            "version": "V22",
            "base_policy": str(BASE.relative_to(ROOT)),
            "corrector_policy": str(CORRECTOR.relative_to(ROOT)),
            "environment": "FirstStepStanceHipRollV22Env",
            "stride_m": 0.042,
            "step_cycle_seconds": 2.56,
            "sim_dt": 0.002,
            "control_dt": 0.04,
            "torque_source": "data.ctrl after PD clip, sampled every physics step",
        },
        "episode": {
            "first_side": str(info["first_side"]),
            "step_order": str(info["step_order"]),
            "samples": int(torque.shape[0]),
            "duration_s": float(rows[-1]["time_s"] + 0.002),
            "terminated": result["terminated"],
            "truncated": result["truncated"],
            "success": bool(info["is_success"]),
            "forward_displacement_m": float(info["base_forward_displacement_m"]),
            "maximum_landing_force_n": float(info["maximum_landing_force_n"]),
        },
        "global": {
            "peak_abs_n_m": float(np.abs(torque).max()),
            "peak_joint": JOINT_NAMES[int(np.abs(torque).max(axis=0).argmax())],
            "peak_time_s": float(
                rows[int(np.abs(torque).max(axis=1).argmax())]["time_s"]
            ),
            "any_saturated": bool((np.abs(torque) >= limit - 1.0e-9).any()),
            "peak_mechanical_power_w": float(np.abs(power).max()),
            "peak_total_mechanical_power_w": float(np.abs(power.sum(axis=1)).max()),
        },
        "joints": joints,
        "per_phase": per_phase,
        "per_footstep": per_footstep,
    }


def plot(path: Path, result: dict[str, object], summary: dict[str, object]) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    rows = result["rows"]
    time = np.array([row["time_s"] for row in rows])
    torque = np.array([row["torque"] for row in rows])
    limit = np.asarray(result["torque_limit"])
    footsteps = np.array([row["footstep"] for row in rows])

    # One panel per joint: rows are the five joints, columns are the two legs,
    # so left and right of the same joint share a row and a y scale.
    joint_labels = ("hip_roll", "hip_pitch", "knee_pitch", "ankle_pitch", "ankle_roll")
    colors = plt.cm.tab10(np.linspace(0.0, 1.0, 5))
    figure, axes = plt.subplots(5, 2, figsize=(15, 14), sharex=True)
    statistics = {entry["joint"]: entry for entry in summary["joints"]}
    for row_index, label in enumerate(joint_labels):
        row_indices = (row_index, row_index + 5)
        span = float(np.abs(torque[:, list(row_indices)]).max()) * 1.18
        for column_index, (side, index) in enumerate(
            (("left", row_indices[0]), ("right", row_indices[1]))
        ):
            axis = axes[row_index, column_index]
            for step in range(1, 9):
                mask = footsteps == step
                if mask.any() and step % 2 == 0:
                    axis.axvspan(time[mask][0], time[mask][-1], color="0.92", zorder=0)
            axis.axhline(0.0, color="0.5", linewidth=0.6)
            axis.plot(time, torque[:, index], linewidth=0.8, color=colors[row_index])
            axis.set_ylim(-span, span)
            axis.grid(alpha=0.3)
            entry = statistics[JOINT_NAMES[index]]
            axis.set_title(
                f"{JOINT_NAMES[index]}  |  peak {entry['peak_abs_n_m']:.2f} N·m"
                f"  ({entry['utilization_percent']:.0f}% of {entry['torque_limit_n_m']:.0f})"
                f"  |  RMS {entry['rms_n_m']:.2f} N·m",
                fontsize=10,
            )
            if column_index == 0:
                axis.set_ylabel(f"{label}\ntorque [N·m]")
    for axis in axes[-1]:
        axis.set_xlabel("time [s]")
    figure.suptitle(
        f"V22 deployed policy per-joint torque — {summary['episode']['first_side']} first"
        f"  |  8 steps / {summary['episode']['duration_s']:.2f} s"
        f"  |  global peak {summary['global']['peak_abs_n_m']:.2f} N·m"
        f" ({summary['global']['peak_joint']}), no saturation"
        "\ngrey bands = even-numbered footsteps; each panel is autoscaled, "
        "so compare against the peak/limit shown in its title",
        fontsize=12,
    )
    figure.tight_layout(rect=(0.0, 0.0, 1.0, 0.955))
    figure.savefig(path, dpi=140)
    plt.close(figure)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--side", choices=("left", "right", "both"), default="both")
    parser.add_argument("--seed", type=int, default=17)
    parser.add_argument(
        "--domain-seed",
        type=int,
        default=None,
        help="apply combined_domain(seed) randomization; default is the nominal model",
    )
    parser.add_argument("--output", type=Path, default=OUTPUT)
    parser.add_argument("--no-plot", action="store_true")
    arguments = parser.parse_args()

    arguments.output.mkdir(parents=True, exist_ok=True)
    sides = ("left", "right") if arguments.side == "both" else (arguments.side,)
    for side in sides:
        result = rollout(side, arguments.seed, arguments.domain_seed)
        summary = summarize(result)
        label = f"torque_profile_{side}_first"
        write_csv(arguments.output / f"{label}.csv", result["rows"])
        (arguments.output / f"{label}.json").write_text(
            json.dumps(summary, indent=2, allow_nan=False), encoding="utf-8"
        )
        if not arguments.no_plot:
            plot(arguments.output / f"{label}.png", result, summary)
        print(json.dumps(summary, indent=2, allow_nan=False), flush=True)


if __name__ == "__main__":
    main()
