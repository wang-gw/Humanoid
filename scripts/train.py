from __future__ import annotations

import argparse
import json
import os
import platform
import subprocess
import sys
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from envs import UnitreeEnv
from rewards import ReferenceWalkingReward, StandingReward, WalkingReward


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Train standing policy with SB3 PPO")
    parser.add_argument("--total-timesteps", type=int, default=500_000)
    parser.add_argument("--n-envs", type=int, default=8)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--save-dir", type=str, default="outputs/train")
    parser.add_argument("--resume-model", type=str, default="")
    parser.add_argument("--resume-vecnorm", type=str, default="")
    parser.add_argument(
        "--run-name",
        type=str,
        default="",
        help="Optional run folder name. Default: timestamp (YYYYmmdd_HHMMSS).",
    )
    parser.add_argument(
        "--device",
        type=str,
        default="auto",
        help="Torch device: auto|mps|cpu|cuda. On Apple Silicon, auto prefers mps.",
    )

    parser.add_argument("--max-episode-steps", type=int, default=1000)
    parser.add_argument("--robot", type=str, default="go1", choices=["go1", "g1"])
    parser.add_argument("--frame-skip", type=int, default=5)
    parser.add_argument("--stage", type=str, default="stand", choices=["stand", "walk"])
    parser.add_argument("--reward", type=str, default="", help="Default: standing for stand, walking for walk.")
    parser.add_argument("--motion-path", type=str, default="", help="Reference motion .npz for reference_walking reward.")

    parser.add_argument(
        "--residual-action",
        dest="residual_action",
        action="store_true",
        help="Use residual policy action: ctrl = base + scale * delta",
    )
    parser.add_argument(
        "--no-residual-action",
        dest="residual_action",
        action="store_false",
        help="Disable residual action mode (policy outputs absolute ctrl)",
    )
    parser.set_defaults(residual_action=True)
    parser.add_argument("--residual-scale", type=float, default=1.0)
    parser.add_argument("--residual-limit-ratio", type=float, default=0.25)
    parser.add_argument("--residual-action-mask", type=str, default="all")
    parser.add_argument("--residual-ramp-time", type=float, default=0.0)
    parser.add_argument("--nominal-ramp-time", type=float, default=0.0)
    parser.add_argument(
        "--base-controller",
        type=str,
        default="",
        choices=[
            "",
            "home",
            "trot",
            "humanoid_walk",
            "reference_motion",
            "reference_left_forward",
            "reference_commanded",
            "reference_stabilized",
            "reference_capture",
            "reference_capture_soft",
            "reference_yaw_stable",
            "reference_yaw_stable_softroll",
            "reference_yaw_stable_midroll",
            "reference_yaw_stable_velroll",
            "reference_yaw_stable_grounded",
            "reference_yaw_stable_footcap",
            "reference_yaw_stable_footcap_rev",
            "reference_yaw_stable_footcap_revsoft",
            "reference_yaw_stable_footcap_revsoft_softroll",
            "reference_yaw_stable_footcap_scheduled",
            "reference_yaw_stable_supportcap",
            "reference_yaw_stable_supportcap_soft",
            "reference_yaw_stable_supportcap_tiny",
            "reference_yaw_stable_supportcap_late",
            "reference_yaw_stable_supportcap_rev",
            "reference_yaw_stable_stanceguard",
            "reference_yaw_stable_stanceguard_latecap",
            "reference_yaw_stable_stanceguard_revcap",
            "reference_yaw_stable_stanceguard_revcap_soft",
            "reference_yaw_stable_stanceguard_width",
            "reference_yaw_stable_stanceguard_width_lift",
            "reference_yaw_stable_stanceguard_width_rightlift",
            "reference_yaw_stable_stanceguard_width_rightstance",
            "reference_yaw_stable_stanceguard_width_rightstance_tight",
            "reference_yaw_stable_stanceguard_width_rightstance_lat",
            "reference_yaw_stable_stanceguard_width_rightstance_speed",
            "reference_yaw_stable_stanceguard_width_rightstance_governed",
            "reference_yaw_stable_stanceguard_width_rightstance_decoupled",
            "reference_yaw_stable_stanceguard_width_rightstance_ground",
            "reference_yaw_stable_stanceguard_width_rightstance_plant",
            "reference_yaw_stable_stanceguard_width_soft",
            "reference_yaw_stable_stanceguard_width_pitch",
            "reference_yaw_stable_stanceguard_width_speed",
            "reference_yaw_stable_stanceguard_width_vel",
            "reference_yaw_stable_speedbrake",
            "reference_yaw_stable_pitchcatch",
            "reference_yaw_stable_revgovernor",
            "reference_yaw_stable_revroll",
            "reference_lift_only",
            "reference_clearance",
            "reference_footplace",
            "reference_latecatch",
            "reference_speedgoverned",
            "reference_landing",
            "reference_hipgoverned",
            "reference_support_stable",
            "reference_pelvis_stable",
            "footstep_ik",
            "footstep_ik_capture",
            "footstep_ik_landing",
            "footstep_ik_landing_governed",
            "footstep_ik_landing_governed_revbrake",
            "footstep_ik_lipm_capture",
            "footstep_ik_forward",
            "footstep_ik_walkform",
            "footstep_ik_landing_strong",
            "footstep_ik_landing_upright",
            "footstep_ik_landing_lateral",
            "footstep_ik_landing_lateral_soft",
            "footstep_ik_landing_lateral_mid",
            "footstep_cartesian",
            "footstep_capture",
            "footstep_capture_aggressive",
            "footstep_capture_earlycatch",
            "footstep_capture_swingcatch",
            "footstep_capture_speedbrake",
            "footstep_capture_placement",
            "footstep_capture_pitchcatch",
            "footstep_capture_toelift",
            "footstep_capture_highclearance",
            "footstep_march",
            "footstep_capture_lipm",
            "footstep_capture_lipm_swingcatch",
            "footstep_capture_lipm_aggressive",
            "footstep_bounded",
            "reference_phase_footstep",
        ],
    )
    parser.add_argument("--gait-frequency", type=float, default=1.5)
    parser.add_argument("--gait-phase-offset", type=float, default=0.0)
    parser.add_argument("--gait-thigh-amp", type=float, default=0.25)
    parser.add_argument("--gait-calf-amp", type=float, default=0.35)
    parser.add_argument("--gait-command-scale", type=float, default=1.0)
    parser.add_argument("--gait-command-reference-speed", type=float, default=0.3)
    parser.add_argument("--gait-command-max-scale", type=float, default=1.0)
    parser.add_argument("--gait-yaw-turn-scale", type=float, default=0.0)
    parser.add_argument("--gait-yaw-turn-direction", type=float, default=1.0)
    parser.add_argument("--gait-yaw-forward-compensation", type=float, default=0.0)
    parser.add_argument("--gait-yaw-hip-amp", type=float, default=0.0)
    parser.add_argument("--fall-height-threshold", type=float, default=0.18)
    parser.add_argument("--fall-height-ratio", type=float, default=0.85)
    parser.add_argument(
        "--soft-termination-prob",
        type=float,
        default=1.0,
        help="Probability of terminating on fall. Set below 1.0 to keep some unstable/fallen states alive for recovery learning.",
    )
    parser.add_argument(
        "--soft-termination-max-grace-steps",
        type=int,
        default=0,
        help="Maximum consecutive fall steps that can be kept alive by soft termination. 0 means no explicit cap.",
    )
    parser.add_argument(
        "--failure-state-path",
        type=str,
        default="",
        help="Optional npz with qpos/qvel/reference_time/command rows for failure-state reset curriculum.",
    )
    parser.add_argument(
        "--failure-state-reset-prob",
        type=float,
        default=0.0,
        help="Probability of resetting from a sampled failure-state row.",
    )
    parser.add_argument(
        "--eval-use-failure-curriculum",
        action="store_true",
        help="Apply failure-state reset and soft termination to EvalCallback env as well. Default keeps eval clean.",
    )
    parser.add_argument(
        "--max-forward-speed",
        type=float,
        default=0.0,
        help="Terminate if base forward speed exceeds this value. Use <=0 to disable.",
    )
    parser.add_argument("--obs-noise-std", type=float, default=0.0)
    parser.add_argument("--foot-state-obs", action="store_true")
    parser.add_argument("--action-noise-std", type=float, default=0.0)
    parser.add_argument("--action-delay-steps", type=int, default=0)
    parser.add_argument("--randomize-dynamics", action="store_true")
    parser.add_argument("--friction-scale-range", type=float, nargs=2, default=(1.0, 1.0))
    parser.add_argument("--trunk-mass-scale-range", type=float, nargs=2, default=(1.0, 1.0))
    parser.add_argument("--command-low", type=float, nargs=3, default=None, metavar=("VX", "VY", "YAW"))
    parser.add_argument("--command-high", type=float, nargs=3, default=None, metavar=("VX", "VY", "YAW"))
    parser.add_argument("--command-generator", choices=("random", "waypoint"), default="random")
    parser.add_argument("--waypoint-count", type=int, default=0)
    parser.add_argument("--waypoint-first-distance", type=float, default=1.0)
    parser.add_argument("--waypoint-forward-range", type=float, nargs=2, default=(0.8, 1.4))
    parser.add_argument("--waypoint-lateral-range", type=float, nargs=2, default=(-0.35, 0.35))
    parser.add_argument("--waypoint-radius", type=float, default=0.35)
    parser.add_argument("--waypoint-target-speed", type=float, default=0.20)
    parser.add_argument("--waypoint-yaw-kp", type=float, default=1.2)
    parser.add_argument("--waypoint-yaw-rate-max", type=float, default=0.35)
    parser.add_argument("--waypoint-vx-min", type=float, default=0.05)
    parser.add_argument("--waypoint-vx-max", type=float, default=0.25)

    parser.add_argument("--learning-rate", type=float, default=3e-4)
    parser.add_argument("--n-steps", type=int, default=2048)
    parser.add_argument("--batch-size", type=int, default=256)
    parser.add_argument("--n-epochs", type=int, default=10)
    parser.add_argument("--gamma", type=float, default=0.99)
    parser.add_argument("--gae-lambda", type=float, default=0.95)
    parser.add_argument("--ent-coef", type=float, default=0.0)
    parser.add_argument("--clip-range", type=float, default=0.2)
    parser.add_argument(
        "--target-kl",
        type=float,
        default=0.0,
        help="SB3 PPO target_kl. Use 0 to disable early stopping by KL.",
    )
    parser.add_argument(
        "--log-std-init",
        type=float,
        default=0.0,
        help="Initial Gaussian policy log std. Lower values reduce early residual exploration.",
    )
    parser.add_argument(
        "--policy-net-arch",
        type=int,
        nargs="+",
        default=None,
        metavar="H",
        help="Hidden layer sizes for SB3 MlpPolicy. Default uses SB3's 64 64.",
    )
    parser.add_argument(
        "--activation-fn",
        choices=("tanh", "relu", "elu"),
        default="tanh",
        help="Activation function for a newly created policy network.",
    )

    parser.add_argument("--eval-freq", type=int, default=20_000)
    parser.add_argument("--eval-episodes", type=int, default=5)
    parser.add_argument(
        "--checkpoint-freq",
        type=int,
        default=0,
        help="Save model checkpoints every N training timesteps. Use 0 to disable.",
    )

    parser.add_argument("--no-vecnorm", action="store_true")
    parser.add_argument("--progress-bar", action="store_true")
    parser.add_argument(
        "--reset-num-timesteps",
        action="store_true",
        help="Reset SB3 timestep counter when resuming. Default keeps the previous counter.",
    )

    parser.add_argument(
        "--torch-threads",
        type=int,
        default=0,
        help="If >0, call torch.set_num_threads(value).",
    )
    parser.add_argument(
        "--torch-interop-threads",
        type=int,
        default=0,
        help="If >0, call torch.set_num_interop_threads(value).",
    )
    parser.add_argument(
        "--disable-mps-fallback",
        action="store_true",
        help="Disable PYTORCH_ENABLE_MPS_FALLBACK=1.",
    )

    # StandingReward weights
    parser.add_argument("--alive-bonus", type=float, default=1.0)
    parser.add_argument("--w-upright", type=float, default=1.0)
    parser.add_argument("--w-ang", type=float, default=0.05)
    parser.add_argument("--w-height", type=float, default=0.0)
    parser.add_argument("--w-dact", type=float, default=0.01)
    parser.add_argument("--w-joint-dev", type=float, default=0.02)
    parser.add_argument("--w-nominal-action", type=float, default=0.0)
    parser.add_argument("--terminal-penalty", type=float, default=1.0)
    parser.add_argument("--w-lin-vel", type=float, default=1.5)
    parser.add_argument("--w-lateral-vel", type=float, default=0.0)
    parser.add_argument("--w-yaw-rate", type=float, default=0.75)
    parser.add_argument("--w-contact", type=float, default=0.05)
    parser.add_argument("--w-phase-contact", type=float, default=0.0)
    parser.add_argument("--w-swing-foot-clearance", type=float, default=0.0)
    parser.add_argument("--swing-foot-clearance-target", type=float, default=0.12)
    parser.add_argument(
        "--clearance-requires-air",
        action="store_true",
        help="Only count swing-foot clearance reward when the phase swing foot is not in contact.",
    )
    parser.add_argument("--w-swing-contact-penalty", type=float, default=0.0)
    parser.add_argument("--w-double-support-penalty", type=float, default=0.0)
    parser.add_argument("--w-lin-overspeed", type=float, default=0.0)
    parser.add_argument("--w-abs-overspeed", type=float, default=0.0)
    parser.add_argument("--abs-overspeed-threshold", type=float, default=0.85)
    parser.add_argument("--w-backward-vel", type=float, default=0.0)
    parser.add_argument("--w-lateral-position", type=float, default=0.0)
    parser.add_argument("--lateral-position-grace", type=float, default=0.06)
    parser.add_argument("--w-lateral-capture", type=float, default=0.0)
    parser.add_argument("--lateral-capture-margin", type=float, default=0.08)
    parser.add_argument("--lateral-capture-clip", type=float, default=0.20)
    parser.add_argument("--lateral-capture-vy-threshold", type=float, default=0.0)
    parser.add_argument("--w-roll-bias", type=float, default=0.0)
    parser.add_argument("--roll-bias-grace", type=float, default=0.04)
    parser.add_argument("--roll-bias-decay", type=float, default=0.995)
    parser.add_argument("--w-yaw-position", type=float, default=0.0)
    parser.add_argument("--yaw-position-grace", type=float, default=0.20)
    parser.add_argument(
        "--w-forward-progress",
        type=float,
        default=0.0,
        help="Signed world-x progress reward. Positive x is forward for the current G1/Go1 assets.",
    )
    parser.add_argument(
        "--forward-progress-clip",
        type=float,
        default=1.0,
        help="Clip signed progress velocity before normalizing the forward progress reward.",
    )
    parser.add_argument("--w-waypoint-progress", type=float, default=0.0)
    parser.add_argument("--waypoint-progress-clip", type=float, default=0.35)
    parser.add_argument("--w-waypoint-distance", type=float, default=0.0)
    parser.add_argument("--waypoint-distance-sigma", type=float, default=0.45)
    parser.add_argument("--w-heading-alignment", type=float, default=0.0)
    parser.add_argument("--w-path-lateral", type=float, default=0.0)
    parser.add_argument("--path-lateral-grace", type=float, default=0.08)
    parser.add_argument("--path-lateral-clip", type=float, default=0.40)
    parser.add_argument(
        "--w-cumulative-backward",
        type=float,
        default=0.0,
        help="Penalty weight for being behind the reset x position.",
    )
    parser.add_argument(
        "--w-forward-debt",
        type=float,
        default=0.0,
        help="Penalty weight for lagging behind command_vx * elapsed_time.",
    )
    parser.add_argument(
        "--forward-debt-grace",
        type=float,
        default=0.03,
        help="Meters of allowed cumulative progress shortfall before forward-debt penalty starts.",
    )
    parser.add_argument("--w-recovery-upright", type=float, default=0.0)
    parser.add_argument("--recovery-upright-threshold", type=float, default=0.10)
    parser.add_argument("--recovery-upright-sigma", type=float, default=0.04)
    parser.add_argument("--w-recovery-upright-progress", type=float, default=0.0)
    parser.add_argument("--recovery-upright-progress-clip", type=float, default=1.0)
    parser.add_argument("--w-recovery-foot-support", type=float, default=0.0)
    parser.add_argument("--lin-overspeed-margin", type=float, default=0.02)
    parser.add_argument("--lin-vel-sigma", type=float, default=0.25)
    parser.add_argument("--yaw-rate-sigma", type=float, default=0.25)
    parser.add_argument("--w-ref-joint", type=float, default=0.5)
    parser.add_argument("--w-ref-joint-mse", type=float, default=0.0)
    parser.add_argument("--ref-joint-sigma", type=float, default=0.05)
    parser.add_argument("--w-foot-slip", type=float, default=0.0)
    parser.add_argument("--w-foot-behind", type=float, default=0.0)
    parser.add_argument("--foot-behind-threshold", type=float, default=-0.42)
    parser.add_argument("--w-double-foot-behind", type=float, default=0.0)
    parser.add_argument("--double-foot-behind-threshold", type=float, default=-0.36)
    parser.add_argument("--w-footstep-forward", type=float, default=0.0)
    parser.add_argument("--w-footstep-backstep", type=float, default=0.0)
    parser.add_argument("--w-footstep-lateral", type=float, default=0.0)
    parser.add_argument("--w-landing-relx", type=float, default=0.0)
    parser.add_argument("--landing-relx-target", type=float, default=-0.04)
    parser.add_argument("--landing-relx-scale", type=float, default=0.16)
    parser.add_argument("--footstep-lateral-target", type=float, default=0.24)
    parser.add_argument("--footstep-forward-clip", type=float, default=0.20)
    parser.add_argument("--footstep-backstep-scale", type=float, default=1.0)
    parser.add_argument("--w-foot-width", type=float, default=0.0)
    parser.add_argument("--foot-width-target", type=float, default=0.24)
    parser.add_argument("--foot-width-tolerance", type=float, default=0.08)
    parser.add_argument("--w-foot-crossing", type=float, default=0.0)
    parser.add_argument("--foot-crossing-margin", type=float, default=0.02)
    parser.add_argument("--w-no-flight", type=float, default=0.0)
    parser.add_argument("--w-body-forward-z", type=float, default=0.0)
    parser.add_argument("--body-forward-z-threshold", type=float, default=0.20)
    parser.add_argument("--w-pelvis-lateral", type=float, default=0.0)
    parser.add_argument("--w-stance-lateral", type=float, default=0.0)
    parser.add_argument("--stance-lateral-margin", type=float, default=0.08)
    parser.add_argument("--w-ref-contact", type=float, default=0.0)
    parser.add_argument("--w-ref-stance-miss", type=float, default=0.0)
    parser.add_argument("--w-ref-swing-extra-contact", type=float, default=0.0)
    parser.add_argument("--w-ref-right-stance-miss", type=float, default=0.0)
    parser.add_argument("--w-ref-right-swing-clearance", type=float, default=0.0)
    parser.add_argument("--ref-right-swing-clearance-target", type=float, default=0.09)
    parser.add_argument("--w-ref-root-vel", type=float, default=0.0)
    parser.add_argument("--ref-root-vel-sigma", type=float, default=0.05)
    parser.add_argument("--w-ref-action", type=float, default=0.0)
    parser.add_argument("--w-ref-action-mse", type=float, default=0.0)
    parser.add_argument("--ref-action-sigma", type=float, default=0.05)
    parser.add_argument(
        "--ref-joint-scope",
        type=str,
        default="lower_body",
        choices=["all", "lower_body", "legs_only"],
        help="Subset of reference joints used by reference_walking.",
    )
    parser.add_argument("--reference-obs", action="store_true")
    parser.add_argument("--reference-obs-horizon", type=int, default=2)
    parser.add_argument("--reference-obs-dt", type=float, default=0.10)
    parser.add_argument(
        "--reference-obs-scope",
        type=str,
        default="lower_body",
        choices=["all", "lower_body", "legs_only"],
    )
    parser.add_argument("--reference-random-start", action="store_true")
    parser.add_argument("--reference-start-time-range", type=float, nargs=2, default=None, metavar=("LOW", "HIGH"))
    parser.add_argument("--reference-start-time", type=float, default=None)
    parser.add_argument("--reference-init-pose-on-reset", action="store_true")
    parser.add_argument("--reference-init-pose-blend", type=float, default=None)
    parser.add_argument("--reference-error-obs", action="store_true")
    parser.add_argument("--reference-time-scale", type=float, default=1.0)
    parser.add_argument("--reference-action-blend", type=float, default=1.0)

    return parser.parse_args()


def resolve_device(requested: str, torch: object) -> str:
    req = requested.strip().lower()
    if req != "auto":
        return req
    if hasattr(torch.backends, "mps") and torch.backends.mps.is_available():
        return "mps"
    if torch.cuda.is_available():
        return "cuda"
    return "cpu"


def get_git_revision() -> str | None:
    try:
        result = subprocess.run(
            ["git", "rev-parse", "--short", "HEAD"],
            check=True,
            capture_output=True,
            text=True,
        )
    except Exception:
        return None
    return result.stdout.strip() or None


def write_json(path: Path, payload: dict[str, Any]) -> None:
    path.write_text(
        json.dumps(payload, indent=2, ensure_ascii=False, sort_keys=True),
        encoding="utf-8",
    )


def classify_artifact(path: Path) -> str:
    suffix = path.suffix.lower()
    name = path.name.lower()
    if suffix == ".zip":
        return "model"
    if suffix == ".pkl":
        return "normalization"
    if name.startswith("events.out.tfevents"):
        return "tensorboard"
    if suffix == ".npz":
        return "evaluation_archive"
    if suffix == ".json":
        return "metadata"
    if suffix == ".csv":
        return "metrics_table"
    if suffix == ".png":
        return "plot"
    return "other"


def write_artifact_manifest(run_dir: Path) -> None:
    artifacts: list[dict[str, Any]] = []
    for path in sorted(p for p in run_dir.rglob("*") if p.is_file()):
        if path.name == "artifact_manifest.json":
            continue
        rel = path.relative_to(run_dir)
        artifacts.append(
            {
                "path": str(rel),
                "kind": classify_artifact(path),
                "size_bytes": path.stat().st_size,
            }
        )

    write_json(
        run_dir / "artifact_manifest.json",
        {
            "generated_at": "not_recorded",
            "run_dir": str(run_dir),
            "artifact_count": len(artifacts),
            "artifacts": artifacts,
        },
    )


def resolve_reward_name(stage: str, requested: str) -> str:
    if requested.strip():
        return requested.strip()
    return "walking" if stage == "walk" else "standing"


def resolve_base_controller(robot: str, stage: str, requested: str) -> str:
    if requested.strip():
        return requested.strip()
    if stage != "walk":
        return "home"
    return "trot" if robot == "go1" else "humanoid_walk"


def build_policy_kwargs(args: argparse.Namespace, torch: Any) -> dict[str, Any]:
    activation_map = {
        "tanh": torch.nn.Tanh,
        "relu": torch.nn.ReLU,
        "elu": torch.nn.ELU,
    }
    kwargs: dict[str, Any] = {"log_std_init": args.log_std_init}
    if args.policy_net_arch:
        arch = [int(v) for v in args.policy_net_arch]
        kwargs["net_arch"] = {"pi": arch, "vf": arch}
    kwargs["activation_fn"] = activation_map[args.activation_fn]
    return kwargs


def make_reward_module(args: argparse.Namespace, reward_name: str) -> object:
    reward_key = reward_name.strip().lower()
    if reward_key in {"standing", "stand"}:
        return StandingReward(
            alive_bonus=args.alive_bonus,
            w_upright=args.w_upright,
            w_ang=args.w_ang,
            w_height=args.w_height,
            w_dact=args.w_dact,
            w_joint_dev=args.w_joint_dev,
            terminal_penalty=args.terminal_penalty,
        )
    if reward_key in {"walking", "walk", "locomotion"}:
        return WalkingReward(
            alive_bonus=args.alive_bonus,
            w_lin_vel=args.w_lin_vel,
            w_lateral_vel=args.w_lateral_vel,
            w_yaw_rate=args.w_yaw_rate,
            w_upright=args.w_upright,
            w_ang=args.w_ang,
            w_height=args.w_height,
            w_dact=args.w_dact,
            w_joint_dev=args.w_joint_dev,
            w_nominal_action=args.w_nominal_action,
            w_contact=args.w_contact,
            w_phase_contact=args.w_phase_contact,
            w_swing_foot_clearance=args.w_swing_foot_clearance,
            swing_foot_clearance_target=args.swing_foot_clearance_target,
            clearance_requires_air=args.clearance_requires_air,
            w_swing_contact_penalty=args.w_swing_contact_penalty,
            w_double_support_penalty=args.w_double_support_penalty,
            w_foot_slip=args.w_foot_slip,
            w_foot_behind=args.w_foot_behind,
            foot_behind_threshold=args.foot_behind_threshold,
            w_double_foot_behind=args.w_double_foot_behind,
            double_foot_behind_threshold=args.double_foot_behind_threshold,
            w_footstep_forward=args.w_footstep_forward,
            w_footstep_backstep=args.w_footstep_backstep,
            w_footstep_lateral=args.w_footstep_lateral,
            w_landing_relx=args.w_landing_relx,
            landing_relx_target=args.landing_relx_target,
            landing_relx_scale=args.landing_relx_scale,
            footstep_lateral_target=args.footstep_lateral_target,
            footstep_forward_clip=args.footstep_forward_clip,
            footstep_backstep_scale=args.footstep_backstep_scale,
            w_foot_width=args.w_foot_width,
            foot_width_target=args.foot_width_target,
            foot_width_tolerance=args.foot_width_tolerance,
            w_foot_crossing=args.w_foot_crossing,
            foot_crossing_margin=args.foot_crossing_margin,
            w_no_flight=args.w_no_flight,
            w_body_forward_z=args.w_body_forward_z,
            body_forward_z_threshold=args.body_forward_z_threshold,
            w_lin_overspeed=args.w_lin_overspeed,
            w_abs_overspeed=args.w_abs_overspeed,
            abs_overspeed_threshold=args.abs_overspeed_threshold,
            w_backward_vel=args.w_backward_vel,
            w_lateral_position=args.w_lateral_position,
            lateral_position_grace=args.lateral_position_grace,
            w_lateral_capture=args.w_lateral_capture,
            lateral_capture_margin=args.lateral_capture_margin,
            lateral_capture_clip=args.lateral_capture_clip,
            lateral_capture_vy_threshold=args.lateral_capture_vy_threshold,
            w_roll_bias=args.w_roll_bias,
            roll_bias_grace=args.roll_bias_grace,
            roll_bias_decay=args.roll_bias_decay,
            w_yaw_position=args.w_yaw_position,
            yaw_position_grace=args.yaw_position_grace,
            w_forward_progress=args.w_forward_progress,
            forward_progress_clip=args.forward_progress_clip,
            w_waypoint_progress=args.w_waypoint_progress,
            waypoint_progress_clip=args.waypoint_progress_clip,
            w_waypoint_distance=args.w_waypoint_distance,
            waypoint_distance_sigma=args.waypoint_distance_sigma,
            w_heading_alignment=args.w_heading_alignment,
            w_path_lateral=args.w_path_lateral,
            path_lateral_grace=args.path_lateral_grace,
            path_lateral_clip=args.path_lateral_clip,
            w_cumulative_backward=args.w_cumulative_backward,
            w_forward_debt=args.w_forward_debt,
            forward_debt_grace=args.forward_debt_grace,
            w_recovery_upright=args.w_recovery_upright,
            recovery_upright_threshold=args.recovery_upright_threshold,
            recovery_upright_sigma=args.recovery_upright_sigma,
            w_recovery_upright_progress=args.w_recovery_upright_progress,
            recovery_upright_progress_clip=args.recovery_upright_progress_clip,
            w_recovery_foot_support=args.w_recovery_foot_support,
            lin_overspeed_margin=args.lin_overspeed_margin,
            terminal_penalty=args.terminal_penalty,
            lin_vel_sigma=args.lin_vel_sigma,
            yaw_rate_sigma=args.yaw_rate_sigma,
        )
    if reward_key in {"reference_walking", "reference_walk", "imitation_walk"}:
        if not args.motion_path.strip():
            raise ValueError("--motion-path is required for reference_walking reward")
        return ReferenceWalkingReward(
            motion_path=args.motion_path,
            w_ref_joint=args.w_ref_joint,
            w_ref_joint_mse=args.w_ref_joint_mse,
            ref_joint_sigma=args.ref_joint_sigma,
            ref_joint_scope=args.ref_joint_scope,
            w_foot_slip=args.w_foot_slip,
            w_pelvis_lateral=args.w_pelvis_lateral,
            w_stance_lateral=args.w_stance_lateral,
            stance_lateral_margin=args.stance_lateral_margin,
            w_ref_contact=args.w_ref_contact,
            w_ref_stance_miss=args.w_ref_stance_miss,
            w_ref_swing_extra_contact=args.w_ref_swing_extra_contact,
            w_ref_right_stance_miss=args.w_ref_right_stance_miss,
            w_ref_right_swing_clearance=args.w_ref_right_swing_clearance,
            ref_right_swing_clearance_target=args.ref_right_swing_clearance_target,
            w_ref_root_vel=args.w_ref_root_vel,
            ref_root_vel_sigma=args.ref_root_vel_sigma,
            w_ref_action=args.w_ref_action,
            w_ref_action_mse=args.w_ref_action_mse,
            ref_action_sigma=args.ref_action_sigma,
            alive_bonus=args.alive_bonus,
            w_lin_vel=args.w_lin_vel,
            w_lateral_vel=args.w_lateral_vel,
            w_yaw_rate=args.w_yaw_rate,
            w_upright=args.w_upright,
            w_ang=args.w_ang,
            w_height=args.w_height,
            w_dact=args.w_dact,
            w_joint_dev=args.w_joint_dev,
            w_nominal_action=args.w_nominal_action,
            w_contact=args.w_contact,
            w_phase_contact=args.w_phase_contact,
            w_swing_foot_clearance=args.w_swing_foot_clearance,
            swing_foot_clearance_target=args.swing_foot_clearance_target,
            clearance_requires_air=args.clearance_requires_air,
            w_swing_contact_penalty=args.w_swing_contact_penalty,
            w_double_support_penalty=args.w_double_support_penalty,
            w_foot_behind=args.w_foot_behind,
            foot_behind_threshold=args.foot_behind_threshold,
            w_double_foot_behind=args.w_double_foot_behind,
            double_foot_behind_threshold=args.double_foot_behind_threshold,
            w_footstep_forward=args.w_footstep_forward,
            w_footstep_backstep=args.w_footstep_backstep,
            w_footstep_lateral=args.w_footstep_lateral,
            w_landing_relx=args.w_landing_relx,
            landing_relx_target=args.landing_relx_target,
            landing_relx_scale=args.landing_relx_scale,
            footstep_lateral_target=args.footstep_lateral_target,
            footstep_forward_clip=args.footstep_forward_clip,
            footstep_backstep_scale=args.footstep_backstep_scale,
            w_foot_width=args.w_foot_width,
            foot_width_target=args.foot_width_target,
            foot_width_tolerance=args.foot_width_tolerance,
            w_foot_crossing=args.w_foot_crossing,
            foot_crossing_margin=args.foot_crossing_margin,
            w_no_flight=args.w_no_flight,
            w_body_forward_z=args.w_body_forward_z,
            body_forward_z_threshold=args.body_forward_z_threshold,
            w_lin_overspeed=args.w_lin_overspeed,
            w_abs_overspeed=args.w_abs_overspeed,
            abs_overspeed_threshold=args.abs_overspeed_threshold,
            w_backward_vel=args.w_backward_vel,
            w_lateral_position=args.w_lateral_position,
            lateral_position_grace=args.lateral_position_grace,
            w_lateral_capture=args.w_lateral_capture,
            lateral_capture_margin=args.lateral_capture_margin,
            lateral_capture_clip=args.lateral_capture_clip,
            lateral_capture_vy_threshold=args.lateral_capture_vy_threshold,
            w_roll_bias=args.w_roll_bias,
            roll_bias_grace=args.roll_bias_grace,
            roll_bias_decay=args.roll_bias_decay,
            w_yaw_position=args.w_yaw_position,
            yaw_position_grace=args.yaw_position_grace,
            w_forward_progress=args.w_forward_progress,
            forward_progress_clip=args.forward_progress_clip,
            w_waypoint_progress=args.w_waypoint_progress,
            waypoint_progress_clip=args.waypoint_progress_clip,
            w_waypoint_distance=args.w_waypoint_distance,
            waypoint_distance_sigma=args.waypoint_distance_sigma,
            w_heading_alignment=args.w_heading_alignment,
            w_path_lateral=args.w_path_lateral,
            path_lateral_grace=args.path_lateral_grace,
            path_lateral_clip=args.path_lateral_clip,
            w_cumulative_backward=args.w_cumulative_backward,
            w_forward_debt=args.w_forward_debt,
            forward_debt_grace=args.forward_debt_grace,
            w_recovery_upright=args.w_recovery_upright,
            recovery_upright_threshold=args.recovery_upright_threshold,
            recovery_upright_sigma=args.recovery_upright_sigma,
            w_recovery_upright_progress=args.w_recovery_upright_progress,
            recovery_upright_progress_clip=args.recovery_upright_progress_clip,
            w_recovery_foot_support=args.w_recovery_foot_support,
            lin_overspeed_margin=args.lin_overspeed_margin,
            terminal_penalty=args.terminal_penalty,
            lin_vel_sigma=args.lin_vel_sigma,
            yaw_rate_sigma=args.yaw_rate_sigma,
        )
    return None


def main() -> int:
    args = parse_args()
    if args.robot == "g1" and args.fall_height_ratio == 0.85:
        args.fall_height_ratio = 0.55
    reward_name = resolve_reward_name(args.stage, args.reward)
    base_controller = resolve_base_controller(args.robot, args.stage, args.base_controller)
    if (
        reward_name in {"reference_walking", "reference_walk", "imitation_walk"}
        and args.reference_obs
        and args.ref_joint_scope != args.reference_obs_scope
    ):
        print(
            "[WARN] ref_joint_scope and reference_obs_scope differ: "
            f"reward tracks {args.ref_joint_scope}, env reference uses {args.reference_obs_scope}. "
            "This can make the policy optimize a different pose than the action/reference signal."
        )

    try:
        import torch
        from stable_baselines3 import PPO
        from stable_baselines3.common.callbacks import CheckpointCallback, EvalCallback
        from stable_baselines3.common.env_util import make_vec_env
        from stable_baselines3.common.monitor import Monitor
        from stable_baselines3.common.utils import FloatSchedule
        from stable_baselines3.common.vec_env import VecNormalize
    except ImportError as exc:
        raise SystemExit(
            "[ERROR] Required packages are missing. Install with: pip install torch stable-baselines3"
        ) from exc

    if not args.disable_mps_fallback:
        os.environ.setdefault("PYTORCH_ENABLE_MPS_FALLBACK", "1")

    if args.torch_threads > 0:
        torch.set_num_threads(args.torch_threads)
    if args.torch_interop_threads > 0:
        torch.set_num_interop_threads(args.torch_interop_threads)

    selected_device = resolve_device(args.device, torch)
    print(
        f"[INFO] device request={args.device} selected={selected_device} "
        f"mps_available={getattr(torch.backends, 'mps', None) and torch.backends.mps.is_available()}"
    )

    save_root = Path(args.save_dir)
    save_root.mkdir(parents=True, exist_ok=True)
    run_name = args.run_name.strip() or "run"
    save_dir = save_root / run_name
    idx = 1
    while save_dir.exists():
        save_dir = save_root / f"{run_name}_{idx:02d}"
        idx += 1
    save_dir.mkdir(parents=True, exist_ok=False)
    print(f"[INFO] run dir: {save_dir}")

    run_config: dict[str, Any] = {
        "created_at": "not_recorded",
        "command": sys.argv,
        "args": vars(args),
        "git_revision": get_git_revision(),
        "platform": {
            "system": platform.system(),
            "release": platform.release(),
            "machine": platform.machine(),
            "python": platform.python_version(),
        },
        "data_policy": {
            "run_dir": str(save_dir),
            "tracked_in_git": [
                "source code",
                "README.md",
                "OPS_README.md",
                "DATA_MANAGEMENT.md",
                ".gitignore",
            ],
            "kept_out_of_git": [
                "runs/",
                "TensorBoard event files",
                "trained model zip files",
                "VecNormalize pickle files",
                "large rollout media",
            ],
        },
    }
    write_json(save_dir / "run_config.json", run_config)

    tb_log_dir = None
    try:
        import tensorboard  # noqa: F401

        tb_log_dir = str(save_dir / "tb")
    except ImportError:
        print("[WARN] tensorboard is not installed; tensorboard logging is disabled.")

    reward_cfg = {
        "reward_name": reward_name,
        "stage": args.stage,
        "alive_bonus": args.alive_bonus,
        "w_upright": args.w_upright,
        "w_ang": args.w_ang,
        "w_height": args.w_height,
        "w_dact": args.w_dact,
        "w_joint_dev": args.w_joint_dev,
        "w_nominal_action": args.w_nominal_action,
        "terminal_penalty": args.terminal_penalty,
        "w_lin_vel": args.w_lin_vel,
        "w_lateral_vel": args.w_lateral_vel,
        "w_yaw_rate": args.w_yaw_rate,
        "w_contact": args.w_contact,
        "w_phase_contact": args.w_phase_contact,
        "w_swing_foot_clearance": args.w_swing_foot_clearance,
        "swing_foot_clearance_target": args.swing_foot_clearance_target,
        "clearance_requires_air": args.clearance_requires_air,
        "w_swing_contact_penalty": args.w_swing_contact_penalty,
        "w_double_support_penalty": args.w_double_support_penalty,
        "w_lin_overspeed": args.w_lin_overspeed,
        "w_abs_overspeed": args.w_abs_overspeed,
        "abs_overspeed_threshold": args.abs_overspeed_threshold,
        "w_backward_vel": args.w_backward_vel,
        "w_lateral_position": args.w_lateral_position,
        "lateral_position_grace": args.lateral_position_grace,
        "w_lateral_capture": args.w_lateral_capture,
        "lateral_capture_margin": args.lateral_capture_margin,
        "lateral_capture_clip": args.lateral_capture_clip,
        "lateral_capture_vy_threshold": args.lateral_capture_vy_threshold,
        "w_roll_bias": args.w_roll_bias,
        "roll_bias_grace": args.roll_bias_grace,
        "roll_bias_decay": args.roll_bias_decay,
        "w_yaw_position": args.w_yaw_position,
        "yaw_position_grace": args.yaw_position_grace,
        "w_forward_progress": args.w_forward_progress,
        "forward_progress_clip": args.forward_progress_clip,
        "w_waypoint_progress": args.w_waypoint_progress,
        "waypoint_progress_clip": args.waypoint_progress_clip,
        "w_waypoint_distance": args.w_waypoint_distance,
        "waypoint_distance_sigma": args.waypoint_distance_sigma,
        "w_heading_alignment": args.w_heading_alignment,
        "w_path_lateral": args.w_path_lateral,
        "path_lateral_grace": args.path_lateral_grace,
        "path_lateral_clip": args.path_lateral_clip,
        "w_cumulative_backward": args.w_cumulative_backward,
        "w_forward_debt": args.w_forward_debt,
        "forward_debt_grace": args.forward_debt_grace,
        "w_recovery_upright": args.w_recovery_upright,
        "recovery_upright_threshold": args.recovery_upright_threshold,
        "recovery_upright_sigma": args.recovery_upright_sigma,
        "w_recovery_upright_progress": args.w_recovery_upright_progress,
        "recovery_upright_progress_clip": args.recovery_upright_progress_clip,
        "w_recovery_foot_support": args.w_recovery_foot_support,
        "lin_overspeed_margin": args.lin_overspeed_margin,
        "lin_vel_sigma": args.lin_vel_sigma,
        "yaw_rate_sigma": args.yaw_rate_sigma,
        "motion_path": args.motion_path,
        "w_ref_joint": args.w_ref_joint,
        "w_ref_joint_mse": args.w_ref_joint_mse,
        "ref_joint_sigma": args.ref_joint_sigma,
        "ref_joint_scope": args.ref_joint_scope,
        "w_foot_slip": args.w_foot_slip,
        "w_foot_behind": args.w_foot_behind,
        "foot_behind_threshold": args.foot_behind_threshold,
        "w_double_foot_behind": args.w_double_foot_behind,
        "double_foot_behind_threshold": args.double_foot_behind_threshold,
        "w_footstep_forward": args.w_footstep_forward,
        "w_footstep_backstep": args.w_footstep_backstep,
        "w_footstep_lateral": args.w_footstep_lateral,
        "w_landing_relx": args.w_landing_relx,
        "landing_relx_target": args.landing_relx_target,
        "landing_relx_scale": args.landing_relx_scale,
        "footstep_lateral_target": args.footstep_lateral_target,
        "footstep_forward_clip": args.footstep_forward_clip,
        "footstep_backstep_scale": args.footstep_backstep_scale,
        "w_foot_width": args.w_foot_width,
        "foot_width_target": args.foot_width_target,
        "foot_width_tolerance": args.foot_width_tolerance,
        "w_foot_crossing": args.w_foot_crossing,
        "foot_crossing_margin": args.foot_crossing_margin,
        "w_no_flight": args.w_no_flight,
        "w_body_forward_z": args.w_body_forward_z,
        "body_forward_z_threshold": args.body_forward_z_threshold,
        "w_pelvis_lateral": args.w_pelvis_lateral,
        "w_stance_lateral": args.w_stance_lateral,
        "stance_lateral_margin": args.stance_lateral_margin,
        "w_ref_contact": args.w_ref_contact,
        "w_ref_stance_miss": args.w_ref_stance_miss,
        "w_ref_swing_extra_contact": args.w_ref_swing_extra_contact,
        "w_ref_right_stance_miss": args.w_ref_right_stance_miss,
        "w_ref_right_swing_clearance": args.w_ref_right_swing_clearance,
        "ref_right_swing_clearance_target": args.ref_right_swing_clearance_target,
        "w_ref_root_vel": args.w_ref_root_vel,
        "ref_root_vel_sigma": args.ref_root_vel_sigma,
        "w_ref_action": args.w_ref_action,
        "w_ref_action_mse": args.w_ref_action_mse,
        "ref_action_sigma": args.ref_action_sigma,
        "reference_obs": args.reference_obs,
        "reference_obs_horizon": args.reference_obs_horizon,
        "reference_obs_dt": args.reference_obs_dt,
        "reference_obs_scope": args.reference_obs_scope,
        "reference_random_start": args.reference_random_start,
        "reference_start_time_range": args.reference_start_time_range,
        "reference_start_time": args.reference_start_time,
        "reference_init_pose_on_reset": args.reference_init_pose_on_reset,
        "reference_init_pose_blend": args.reference_init_pose_blend,
        "reference_error_obs": args.reference_error_obs,
        "foot_state_obs": args.foot_state_obs,
        "reference_time_scale": args.reference_time_scale,
        "reference_action_blend": args.reference_action_blend,
    }
    print(f"[INFO] reward config: {reward_cfg}")
    print(
        f"[INFO] residual config: enabled={args.residual_action} "
        f"scale={args.residual_scale} limit_ratio={args.residual_limit_ratio} "
        f"mask={args.residual_action_mask} "
        f"residual_ramp={args.residual_ramp_time} nominal_ramp={args.nominal_ramp_time} "
        f"base_controller={base_controller}"
    )
    print(
        f"[INFO] robustness config: obs_noise={args.obs_noise_std} "
        f"action_noise={args.action_noise_std} action_delay={args.action_delay_steps} "
        f"randomize_dynamics={args.randomize_dynamics} "
        f"friction_range={tuple(args.friction_scale_range)} "
        f"trunk_mass_range={tuple(args.trunk_mass_scale_range)}"
    )
    print(
        f"[INFO] command config: generator={args.command_generator} "
        f"command_low={args.command_low} command_high={args.command_high} "
        f"waypoints={args.waypoint_count} first={args.waypoint_first_distance} "
        f"forward_range={tuple(args.waypoint_forward_range)} "
        f"lateral_range={tuple(args.waypoint_lateral_range)} "
        f"radius={args.waypoint_radius} speed={args.waypoint_target_speed} "
        f"yaw_kp={args.waypoint_yaw_kp} yaw_max={args.waypoint_yaw_rate_max}"
    )

    def make_env(*, eval_mode: bool = False) -> UnitreeEnv:
        use_failure_curriculum = (not eval_mode) or bool(args.eval_use_failure_curriculum)
        env = UnitreeEnv(
            render_mode=None,
            robot=args.robot,
            stage=args.stage,
            reward_name=reward_name,
            reward_module=make_reward_module(args, reward_name),
            frame_skip=args.frame_skip,
            sleep_sec=0.0,
            max_episode_steps=args.max_episode_steps,
            fall_height_threshold=args.fall_height_threshold,
            fall_height_ratio=args.fall_height_ratio,
            soft_termination_prob=args.soft_termination_prob if use_failure_curriculum else 1.0,
            soft_termination_max_grace_steps=(
                args.soft_termination_max_grace_steps if use_failure_curriculum else 0
            ),
            failure_state_path=args.failure_state_path if use_failure_curriculum else "",
            failure_state_reset_prob=args.failure_state_reset_prob if use_failure_curriculum else 0.0,
            max_forward_speed=args.max_forward_speed,
            residual_action=args.residual_action,
            residual_scale=args.residual_scale,
            residual_limit_ratio=args.residual_limit_ratio,
            residual_action_mask=args.residual_action_mask,
            residual_ramp_time=args.residual_ramp_time,
            nominal_ramp_time=args.nominal_ramp_time,
            base_controller=base_controller,
            gait_frequency=args.gait_frequency,
            gait_phase_offset=args.gait_phase_offset,
            gait_thigh_amp=args.gait_thigh_amp,
            gait_calf_amp=args.gait_calf_amp,
            gait_command_scale=args.gait_command_scale,
            gait_command_reference_speed=args.gait_command_reference_speed,
            gait_command_max_scale=args.gait_command_max_scale,
            gait_yaw_turn_scale=args.gait_yaw_turn_scale,
            gait_yaw_turn_direction=args.gait_yaw_turn_direction,
            gait_yaw_forward_compensation=args.gait_yaw_forward_compensation,
            gait_yaw_hip_amp=args.gait_yaw_hip_amp,
            obs_noise_std=args.obs_noise_std,
            foot_state_obs=args.foot_state_obs,
            action_noise_std=args.action_noise_std,
            action_delay_steps=args.action_delay_steps,
            randomize_dynamics=args.randomize_dynamics,
            friction_scale_range=tuple(args.friction_scale_range),
            trunk_mass_scale_range=tuple(args.trunk_mass_scale_range),
            command_low=tuple(args.command_low) if args.command_low is not None else None,
            command_high=tuple(args.command_high) if args.command_high is not None else None,
            command_generator=args.command_generator,
            waypoint_count=args.waypoint_count,
            waypoint_first_distance=args.waypoint_first_distance,
            waypoint_forward_range=tuple(args.waypoint_forward_range),
            waypoint_lateral_range=tuple(args.waypoint_lateral_range),
            waypoint_radius=args.waypoint_radius,
            waypoint_target_speed=args.waypoint_target_speed,
            waypoint_yaw_kp=args.waypoint_yaw_kp,
            waypoint_yaw_rate_max=args.waypoint_yaw_rate_max,
            waypoint_vx_min=args.waypoint_vx_min,
            waypoint_vx_max=args.waypoint_vx_max,
            reference_obs_path=args.motion_path,
            reference_obs=args.reference_obs,
            reference_obs_horizon=args.reference_obs_horizon,
            reference_obs_dt=args.reference_obs_dt,
            reference_obs_scope=args.reference_obs_scope,
            reference_random_start=args.reference_random_start,
            reference_start_time_range=tuple(args.reference_start_time_range)
            if args.reference_start_time_range is not None
            else None,
            reference_start_time=args.reference_start_time,
            reference_init_pose_on_reset=args.reference_init_pose_on_reset,
            reference_init_pose_blend=args.reference_init_pose_blend,
            reference_error_obs=args.reference_error_obs,
            reference_time_scale=args.reference_time_scale,
            reference_action_blend=args.reference_action_blend,
        )
        return Monitor(env)

    train_env = make_vec_env(lambda: make_env(eval_mode=False), n_envs=args.n_envs, seed=args.seed)
    eval_env = make_vec_env(lambda: make_env(eval_mode=True), n_envs=1, seed=args.seed + 1)

    resume_vecnorm = Path(args.resume_vecnorm) if args.resume_vecnorm.strip() else None
    if not args.no_vecnorm:
        if resume_vecnorm is not None:
            train_env = VecNormalize.load(str(resume_vecnorm), train_env)
            train_env.training = True
            train_env.norm_reward = True
            eval_env = VecNormalize.load(str(resume_vecnorm), eval_env)
            eval_env.training = False
            eval_env.norm_reward = False
            print(f"[INFO] resumed vecnormalize: {resume_vecnorm}")
        else:
            train_env = VecNormalize(
                train_env,
                norm_obs=True,
                norm_reward=True,
                clip_obs=10.0,
                gamma=args.gamma,
            )
            eval_env = VecNormalize(
                eval_env,
                norm_obs=True,
                norm_reward=False,
                clip_obs=10.0,
                gamma=args.gamma,
                training=False,
            )
        eval_env.obs_rms = train_env.obs_rms

    eval_callback = EvalCallback(
        eval_env,
        best_model_save_path=str(save_dir / "best"),
        log_path=str(save_dir / "eval"),
        eval_freq=max(args.eval_freq // args.n_envs, 1),
        n_eval_episodes=args.eval_episodes,
        deterministic=True,
        render=False,
    )
    callbacks: list[Any] = [eval_callback]
    if args.checkpoint_freq > 0:
        checkpoint_callback = CheckpointCallback(
            save_freq=max(args.checkpoint_freq // args.n_envs, 1),
            save_path=str(save_dir / "checkpoints"),
            name_prefix="ppo_checkpoint",
            save_replay_buffer=False,
            save_vecnormalize=not args.no_vecnorm,
        )
        callbacks.append(checkpoint_callback)

    if args.resume_model.strip():
        model = PPO.load(
            args.resume_model,
            env=train_env,
            device=selected_device,
            tensorboard_log=tb_log_dir,
        )
        model.learning_rate = args.learning_rate
        model.lr_schedule = FloatSchedule(args.learning_rate)
        model.clip_range = FloatSchedule(args.clip_range)
        model.target_kl = args.target_kl if args.target_kl > 0.0 else None
        model.n_epochs = args.n_epochs
        model.batch_size = args.batch_size
        model.ent_coef = args.ent_coef
        model.gamma = args.gamma
        model.gae_lambda = args.gae_lambda
        print(f"[INFO] resumed model: {args.resume_model}")
        print(
            "[INFO] override resumed PPO settings: "
            f"learning_rate={args.learning_rate} clip_range={args.clip_range} "
            f"target_kl={model.target_kl} n_epochs={args.n_epochs} batch_size={args.batch_size}"
        )
    else:
        model = PPO(
            policy="MlpPolicy",
            env=train_env,
            learning_rate=args.learning_rate,
            n_steps=args.n_steps,
            batch_size=args.batch_size,
            n_epochs=args.n_epochs,
            gamma=args.gamma,
            gae_lambda=args.gae_lambda,
            ent_coef=args.ent_coef,
            clip_range=args.clip_range,
            target_kl=args.target_kl if args.target_kl > 0.0 else None,
            policy_kwargs=build_policy_kwargs(args, torch),
            verbose=1,
            seed=args.seed,
            device=selected_device,
            tensorboard_log=tb_log_dir,
        )

    model.learn(
        total_timesteps=args.total_timesteps,
        callback=callbacks,
        progress_bar=args.progress_bar,
        reset_num_timesteps=args.reset_num_timesteps,
    )

    model_path = save_dir / "ppo_stand_final"
    model.save(str(model_path))
    print(f"[OK] saved model: {model_path}.zip")

    if not args.no_vecnorm:
        vecnorm_path = save_dir / "vecnormalize.pkl"
        train_env.save(str(vecnorm_path))
        print(f"[OK] saved vecnormalize: {vecnorm_path}")

    write_artifact_manifest(save_dir)
    print(f"[OK] saved artifact manifest: {save_dir / 'artifact_manifest.json'}")

    train_env.close()
    eval_env.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
