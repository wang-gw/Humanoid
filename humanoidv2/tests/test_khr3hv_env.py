from dataclasses import replace
from pathlib import Path

import mujoco
import numpy as np
import pytest

from humanoidv2 import (
    KHR3HVEnv,
    KHRConfig,
    KHR3HVV2Env,
    KHR3HVV21Env,
    KHR3HVV22Env,
    KHR3HVV3Env,
    KHR3HVV31Env,
    KHR3HVV32Env,
    KHR3HVV33Env,
    KHR3HVV34Env,
    SingleSupportV4Env,
    SingleStepV5Env,
    TwoStepV6Env,
    FourStepV7Env,
    EightStepV8Env,
    FastEightStepV9Env,
    SoftLandingEightStepV10Env,
    CorrectedForwardV13Env,
    DynamicForwardV14Env,
    DynamicForwardV15Env,
    DynamicForwardV16Env,
    ForwardMarginV17Env,
    RobustForwardMarginV18Env,
    LandingResidualV19Env,
    ContactAwareResidualV20Env,
    PhaseSplitResidualV21Env,
    CounterfactualProbeV22Env,
    FirstStepStanceHipRollV22Env,
    RobustSoftLandingV11Env,
    CurriculumSoftLandingV12Env,
)

ROOT = Path(__file__).resolve().parents[1]
SYMMETRIC_MODEL = ROOT / "models/urdf_f_v2/URDF_F_v2_footprint_contact_symmetric_inertia.xml"


def test_environment_shapes_and_finite_step():
    env = KHR3HVEnv()
    observation, info = env.reset(seed=1)
    assert observation.shape == (82,)
    assert np.isfinite(observation).all()
    assert env.reference.shape == (50, 10)
    assert np.isfinite(env.reference).all()
    assert np.all(env.reference >= env.joint_ranges[:, 0])
    assert np.all(env.reference <= env.joint_ranges[:, 1])
    assert info["torso_height"] > env.fall_height
    observation, reward, terminated, truncated, step_info = env.step(np.zeros(10, dtype=np.float32))
    assert observation.shape == (82,)
    assert np.isfinite(observation).all()
    assert np.isfinite(reward)
    assert not truncated
    assert "forward_velocity" in step_info
    env.close()


def test_v21_uses_cycle_velocity_and_mirrored_joint_coordinates():
    env = KHR3HVV21Env()
    assert env._velocity_history.shape == (50, 5)
    assert env.config.mirrored_roll_symmetry
    observation, _ = env.reset(seed=1)
    observation, reward, terminated, truncated, info = env.step(np.zeros(10, dtype=np.float32))
    assert np.isfinite(reward)
    assert not info["is_success"]
    env.close()


def test_v22_penalizes_fall_and_rewards_survival():
    env = KHR3HVV22Env()
    assert env.config.alive_weight == 0.20
    assert env.config.fall_penalty == -10.0
    env.close()


def test_v3_adds_fore_aft_reference_stride():
    v22 = KHR3HVV22Env()
    v3 = KHR3HVV3Env()
    assert v3.config.stride_half_length == 0.025
    assert np.isclose(v3.config.foot_height - v3.config.foot_height_offset, 0.010)
    assert not np.allclose(v22.reference, v3.reference)
    assert np.isfinite(v3.reference).all()
    assert np.all(v3.reference >= v3.joint_ranges[:, 0])
    assert np.all(v3.reference <= v3.joint_ranges[:, 1])
    v22.close()
    v3.close()


def test_v31_swaps_swing_phase_without_changing_stride_size():
    v3 = KHR3HVV3Env()
    v31 = KHR3HVV31Env()
    assert v31.config.swap_swing_legs
    assert v31.config.stride_half_length == v3.config.stride_half_length
    assert not np.allclose(v31.reference, v3.reference)
    v3.close()
    v31.close()


def test_v32_uses_quadrature_stride_and_reference_reset():
    env = KHR3HVV32Env()
    assert env.config.phased_stride
    assert env.config.reset_to_reference
    assert np.allclose(env.home_qpos[env.qpos_ids], env.reference[0])
    assert np.isfinite(env.reference).all()
    env.close()


def test_v33_decouples_lift_and_forward_leg_mapping():
    env = KHR3HVV33Env()
    assert not env.config.swap_swing_legs
    assert env.config.swap_forward_legs
    assert env.config.phased_stride
    env.close()


def test_v34_adds_contact_schedule_reward():
    env = KHR3HVV34Env()
    assert env.config.contact_schedule_reward_weight == 0.15
    assert np.isclose(env.config.sway_width, 0.035)
    env.close()


def test_v2_has_strict_reward_collision_proxies_and_success_metric():
    env = KHR3HVV2Env()
    observation, _ = env.reset(seed=1)
    collision_names = {env.model.geom(i).name for i in range(env.model.ngeom)}
    assert "v2_torso_collision" in collision_names
    assert "v2_left_thigh_collision" in collision_names
    proxy_ids = [env.model.geom(name).id for name in collision_names if name.startswith("v2_")]
    assert all(env.model.geom_contype[geom_id] == 2 for geom_id in proxy_ids)
    assert all(env.model.geom_conaffinity[geom_id] == 0 for geom_id in proxy_ids)
    observation, reward, terminated, truncated, info = env.step(np.zeros(10, dtype=np.float32))
    assert np.isfinite(observation).all()
    assert np.isfinite(reward)
    assert not terminated
    assert not truncated
    assert not info["is_success"]
    assert "mean_episode_forward_velocity" in info
    env.close()


def test_v4_observation_identifies_requested_support_task():
    right = SingleSupportV4Env(fixed_side="right")
    left = SingleSupportV4Env(fixed_side="left")
    right_observation, right_info = right.reset(seed=7)
    left_observation, left_info = left.reset(seed=7)
    assert right_observation.shape == (82,)
    assert left_observation.shape == (82,)
    assert right_observation[-2] == 1.0
    assert left_observation[-2] == -1.0
    assert right_info["swing_side"] == "right"
    assert left_info["swing_side"] == "left"
    right.close()
    left.close()


def test_v4_reference_completes_both_single_support_tasks():
    for side in ("right", "left"):
        env = SingleSupportV4Env(reference_only=True, fixed_side=side)
        observation, _ = env.reset(seed=7)
        forces = []
        for step in range(200):
            observation, reward, terminated, truncated, info = env.step(
                np.zeros(10, dtype=np.float32)
            )
            forces.append(info["swing_force_n"])
            assert np.isfinite(observation).all()
            assert np.isfinite(reward)
            if terminated or truncated:
                break
        assert step + 1 == 200
        assert not terminated
        assert truncated
        assert info["is_success"]
        assert np.mean(forces[-50:]) < 5.0
        assert info["stance_force_n"] > 30.0
        env.close()


def test_symmetric_inertia_model_preserves_mass_and_mirrors_com():
    original = mujoco.MjModel.from_xml_path(
        str(ROOT / "models/urdf_f_v2/URDF_F_v2_footprint_contact.xml")
    )
    symmetric = mujoco.MjModel.from_xml_path(str(SYMMETRIC_MODEL))
    assert np.isclose(original.body_mass.sum(), symmetric.body_mass.sum(), atol=1.0e-12)
    mirror = np.diag([-1.0, 1.0, 1.0])
    for left_name, right_name in (
        ("hipjoint1_L_1", "hipjoint1_R_1"),
        ("thigh_L_1", "thigh_R_1"),
        ("calf_L_1", "calf_R_1"),
        ("footJ_L_1", "footJ_R_1"),
        ("foot_L_1", "foot_R_1"),
    ):
        left = symmetric.body(left_name).id
        right = symmetric.body(right_name).id
        assert symmetric.body_mass[left] == symmetric.body_mass[right]
        assert np.allclose(symmetric.body_ipos[right], mirror @ symmetric.body_ipos[left], atol=1.0e-12)
    assert symmetric.body_ipos[symmetric.body("base_link").id, 0] == 0.0


def test_symmetric_v4_uses_identical_targets_and_completes_both_sides():
    summaries = []
    for side in ("right", "left"):
        env = SingleSupportV4Env(
            model_path=SYMMETRIC_MODEL,
            reference_only=True,
            fixed_side=side,
            task_profile="symmetric",
        )
        observation, _ = env.reset(seed=7)
        forces = []
        heights = []
        for _ in range(200):
            observation, reward, terminated, truncated, info = env.step(
                np.zeros(10, dtype=np.float32)
            )
            forces.append(info["swing_force_n"])
            heights.append(info["swing_height_m"])
        assert not terminated and truncated and info["is_success"]
        assert env._pd_gains() == (80.0, 0.32)
        summaries.append((np.mean(forces[-50:]), max(heights)))
        env.close()
    assert np.isclose(summaries[0][0], summaries[1][0], atol=3.0e-5)
    assert np.isclose(summaries[0][1], summaries[1][1], atol=2.0e-5)


def test_v5_reference_completes_lift_advance_land_on_both_sides():
    for side in ("left", "right"):
        env = SingleStepV5Env(reference_only=True, fixed_side=side)
        observation, info = env.reset(seed=7)
        assert observation.shape == (82,)
        phases = []
        advance_forces = []
        for step in range(env.config.max_episode_steps):
            observation, reward, terminated, truncated, info = env.step(
                np.zeros(10, dtype=np.float32)
            )
            phases.append(info["task_phase"])
            if info["task_phase"] == "advance":
                advance_forces.append(info["swing_force_n"])
        assert step + 1 == 250
        assert not terminated and truncated and info["is_success"]
        assert set(phases) == {"lift", "advance", "land", "settle"}
        assert np.mean(advance_forces) < 5.0
        assert info["final_both_contact_fraction"] >= 0.9
        assert info["step_length_m"] >= 0.020
        env.close()


def test_v6_reference_completes_both_alternating_orders_without_reset():
    expected_phases = {
        "first_lift": 100,
        "first_advance": 50,
        "first_land": 50,
        "first_settle": 50,
        "second_lift": 100,
        "second_advance": 50,
        "second_land": 50,
        "second_settle": 50,
    }
    for first_side in ("left", "right"):
        env = TwoStepV6Env(reference_only=True, fixed_first_side=first_side)
        observation, _ = env.reset(seed=7)
        phase_counts = {phase: 0 for phase in expected_phases}
        for step in range(env.config.max_episode_steps):
            observation, reward, terminated, truncated, info = env.step(
                np.zeros(10, dtype=np.float32)
            )
            phase_counts[info["task_phase"]] += 1
        assert step + 1 == 500
        assert phase_counts == expected_phases
        assert not terminated and truncated and info["is_success"]
        assert info["first_step_length_m"] >= 0.020
        assert info["active_step_length_m"] >= 0.020
        assert info["base_forward_displacement_m"] >= 0.035
        assert info["first_both_contact_fraction"] >= 0.9
        assert info["final_both_contact_fraction"] >= 0.9
        env.close()


def test_v7_reference_completes_four_alternating_steps_without_reset():
    expected_phases = {
        f"step{step}_{phase}": duration
        for step in range(1, 5)
        for phase, duration in (("lift", 100), ("advance", 50), ("land", 50), ("settle", 50))
    }
    for first_side in ("left", "right"):
        env = FourStepV7Env(reference_only=True, fixed_first_side=first_side)
        observation, _ = env.reset(seed=7)
        phase_counts = {phase: 0 for phase in expected_phases}
        for step in range(env.config.max_episode_steps):
            observation, reward, terminated, truncated, info = env.step(
                np.zeros(10, dtype=np.float32)
            )
            phase_counts[info["task_phase"]] += 1
        assert step + 1 == 1000
        assert phase_counts == expected_phases
        assert not terminated and truncated and info["is_success"]
        assert info["base_forward_displacement_m"] >= 0.070
        for index in range(1, 5):
            assert info[f"step_{index}_advance_mean_force_n"] < 5.0
            assert info[f"step_{index}_length_m"] >= 0.020
            assert info[f"step_{index}_both_contact_fraction"] >= 0.9
        env.close()


def test_v8_reference_completes_eight_forward_steps_with_reanchoring():
    expected_phases = {
        f"step{step}_{phase}": duration
        for step in range(1, 9)
        for phase, duration in (("lift", 100), ("advance", 50), ("land", 50), ("settle", 50))
    }
    for first_side in ("left", "right"):
        env = EightStepV8Env(reference_only=True, fixed_first_side=first_side)
        observation, _ = env.reset(seed=7)
        phase_counts = {phase: 0 for phase in expected_phases}
        for step in range(env.config.max_episode_steps):
            observation, reward, terminated, truncated, info = env.step(
                np.zeros(10, dtype=np.float32)
            )
            phase_counts[info["task_phase"]] += 1
        assert step + 1 == 2000
        assert phase_counts == expected_phases
        assert not terminated and truncated and info["is_success"]
        assert info["reanchor_mode"] == "joint"
        assert info["base_forward_displacement_m"] >= 0.180
        for index in range(1, 9):
            assert info[f"step_{index}_advance_mean_force_n"] < 5.0
            assert info[f"step_{index}_length_m"] >= 0.020
            assert info[f"step_{index}_both_contact_fraction"] >= 0.9
        env.close()


def test_v9_six_second_reference_is_safe_and_near_feasible_for_learning():
    expected_phases = {
        f"step{step}_{phase}": duration
        for step in range(1, 9)
        for phase, duration in (("lift", 60), ("advance", 30), ("land", 30), ("settle", 30))
    }
    for first_side in ("left", "right"):
        env = FastEightStepV9Env(reference_only=True, fixed_first_side=first_side)
        observation, _ = env.reset(seed=7)
        phase_counts = {phase: 0 for phase in expected_phases}
        for step in range(env.config.max_episode_steps):
            observation, reward, terminated, truncated, info = env.step(
                np.zeros(10, dtype=np.float32)
            )
            phase_counts[info["task_phase"]] += 1
        assert step + 1 == 1200
        assert phase_counts == expected_phases
        assert not terminated and truncated
        assert info["base_forward_displacement_m"] >= 0.180
        assert not info["is_success"]
        for index in range(1, 9):
            assert info[f"step_{index}_advance_mean_force_n"] < 5.0
            assert info[f"step_{index}_length_m"] >= 0.020
            assert info[f"step_{index}_both_contact_fraction"] >= 0.9
        env.close()


def test_v10_reallocates_landing_time_and_enforces_impact_limit():
    for first_side in ("left", "right"):
        env = SoftLandingEightStepV10Env(reference_only=True, fixed_first_side=first_side)
        observation, _ = env.reset(seed=7)
        phase_counts = {
            f"step{step}_{phase}": duration
            for step in range(1, 9)
            for phase, duration in (("lift", 60), ("advance", 30), ("land", 50), ("settle", 10))
        }
        observed_counts = {phase: 0 for phase in phase_counts}
        for step in range(env.config.max_episode_steps):
            observation, reward, terminated, truncated, info = env.step(
                np.zeros(10, dtype=np.float32)
            )
            observed_counts[info["task_phase"]] += 1
        assert step + 1 == 1200
        assert observed_counts == phase_counts
        assert not terminated and truncated
        assert info["maximum_landing_force_n"] <= 50.0
        assert info["impact_limit_satisfied"]
        assert not info["gait_success"]
        assert not info["is_success"]
        env.close()


def test_v13_uses_anatomical_minus_y_forward_without_changing_v10():
    legacy = SoftLandingEightStepV10Env(reference_only=True, fixed_first_side="left")
    corrected = CorrectedForwardV13Env(reference_only=True, fixed_first_side="left")
    assert legacy.forward_sign == 1.0
    assert corrected.forward_sign == -1.0
    assert legacy.residual_action_scale == 0.25
    assert corrected.residual_action_scale == 0.10
    assert not np.allclose(legacy._targets["left"][2], corrected._targets["left"][2])

    observation, info = corrected.reset(seed=7)
    start_world_y = float(corrected.data.qpos[1])
    phase_counts = {phase: 0 for phase in ("lift", "hold", "advance", "land", "settle")}
    for _ in range(corrected.config.max_episode_steps):
        observation, reward, terminated, truncated, info = corrected.step(
            np.zeros(10, dtype=np.float32)
        )
        phase_counts[info["task_phase"].split("_", 1)[1]] += 1
    assert not terminated and truncated
    assert info["forward_axis"] == "-Y"
    assert corrected.data.qpos[1] - start_world_y < -0.20
    assert info["base_forward_displacement_m"] > 0.20
    assert phase_counts == {
        "lift": 480,
        "hold": 160,
        "advance": 240,
        "land": 240,
        "settle": 80,
    }
    assert all(info[f"step_{index}_length_m"] >= 0.020 for index in range(1, 9))
    legacy.close()
    corrected.close()


def test_v14_compresses_v13_to_first_speed_curriculum_stage():
    v13 = CorrectedForwardV13Env(reference_only=True, fixed_first_side="left")
    v14 = DynamicForwardV14Env(reference_only=True, fixed_first_side="left")
    assert v13.step_cycle_steps == 150
    assert v13.config.max_episode_steps == 1200
    assert v14.step_cycle_steps == 113
    assert v14.config.max_episode_steps == 904
    assert np.isclose(v14.step_cycle_steps * v14.config.control_dt, 4.52)
    assert v14.forward_sign == -1.0
    assert v14.residual_action_scale == 0.10
    assert v13.kp == 90.0
    assert v14.kp == 110.0

    observation, info = v14.reset(seed=7)
    start_world_y = float(v14.data.qpos[1])
    phase_counts = {phase: 0 for phase in ("lift", "hold", "advance", "land", "settle")}
    for _ in range(v14.config.max_episode_steps):
        observation, reward, terminated, truncated, info = v14.step(
            np.zeros(10, dtype=np.float32)
        )
        phase_counts[info["task_phase"].split("_", 1)[1]] += 1
    assert not terminated and truncated
    assert info["speed_curriculum_stage"] == 1
    assert info["step_cycle_steps"] == 113
    assert np.isclose(info["step_cycle_seconds"], 4.52)
    assert np.isfinite(info["unload_excess_penalty"])
    assert np.isfinite(info["swing_clearance_reward"])
    assert v14.data.qpos[1] - start_world_y < 0.0
    assert info["base_forward_displacement_m"] > 0.0
    assert phase_counts == {
        "lift": 360,
        "hold": 120,
        "advance": 184,
        "land": 184,
        "settle": 56,
    }
    v13.close()
    v14.close()


def test_v15_compresses_v14_to_second_speed_curriculum_stage():
    v14 = DynamicForwardV14Env(reference_only=True, fixed_first_side="left")
    v15 = DynamicForwardV15Env(reference_only=True, fixed_first_side="left")
    assert v14.step_cycle_steps == 113
    assert v14.config.max_episode_steps == 904
    assert v15.step_cycle_steps == 85
    assert v15.config.max_episode_steps == 680
    assert np.isclose(v15.step_cycle_steps * v15.config.control_dt, 3.40)
    assert v15.forward_sign == -1.0
    assert v15.residual_action_scale == 0.10
    assert v15.kp == 130.0
    assert v15.landing_kp == 70.0
    assert v15._phase_pd_gains("advance", 0.5) == (130.0, 0.52)
    assert v15._phase_pd_gains("land", 0.5) == (70.0, 0.28)

    expected = {"lift": 32, "hold": 11, "advance": 15, "land": 21, "settle": 6}
    phase_counts = {phase: 0 for phase in expected}
    v15.reset(seed=7)
    for step_count in range(v15.step_cycle_steps):
        v15._step_count = step_count
        _, phase, _, _, _ = v15._reference_target()
        phase_counts[phase.split("_", 1)[1]] += 1
    assert phase_counts == expected
    _, info = v15.reset(seed=7)
    assert info["speed_curriculum_stage"] == 2
    assert info["step_cycle_steps"] == 85
    assert np.isclose(info["step_cycle_seconds"], 3.40)
    observation, reward, terminated, truncated, step_info = v15.step(
        np.zeros(10, dtype=np.float32)
    )
    assert step_info["peak_control_interval_torque_n_m"] <= 24.0
    assert 0.0 <= step_info["actuator_saturation_fraction"] <= 1.0
    v14.close()
    v15.close()


def test_v16_compresses_v15_and_smooths_gain_before_landing():
    v15 = DynamicForwardV15Env(reference_only=True, fixed_first_side="left")
    v16 = DynamicForwardV16Env(reference_only=True, fixed_first_side="left")
    assert v15.step_cycle_steps == 85
    assert v16.step_cycle_steps == 64
    assert v16.config.max_episode_steps == 512
    assert np.isclose(v16.step_cycle_steps * v16.config.control_dt, 2.56)
    assert v16.forward_sign == -1.0
    assert v16.residual_action_scale == 0.10
    assert v16._phase_pd_gains("advance", 0.50) == (130.0, 0.52)
    transition_kp, transition_kd = v16._phase_pd_gains("advance", 0.95)
    assert 80.0 < transition_kp < 130.0
    assert 0.32 < transition_kd < 0.52
    assert v16._phase_pd_gains("advance", 1.0) == (80.0, 0.32)
    assert v16._phase_pd_gains("land", 0.0) == (80.0, 0.32)

    expected = {"lift": 20, "hold": 8, "advance": 10, "land": 21, "settle": 5}
    phase_counts = {phase: 0 for phase in expected}
    v16.reset(seed=7)
    for step_count in range(v16.step_cycle_steps):
        v16._step_count = step_count
        _, phase, _, _, _ = v16._reference_target()
        phase_counts[phase.split("_", 1)[1]] += 1
    assert phase_counts == expected
    _, info = v16.reset(seed=7)
    assert info["speed_curriculum_stage"] == 3
    assert info["step_cycle_steps"] == 64
    assert np.isclose(info["step_cycle_seconds"], 2.56)
    assert info["gain_transition_fraction"] == 0.12
    v15.close()
    v16.close()


def test_v17_adds_forward_margin_target_without_changing_v16():
    v16 = DynamicForwardV16Env(reference_only=True, fixed_first_side="left")
    v17 = ForwardMarginV17Env(reference_only=True, fixed_first_side="left")
    assert v16.stride_m == 0.035
    assert v16.minimum_forward_m == 0.180
    assert v17.stride_m == 0.042
    assert v17.minimum_forward_m == 0.200
    assert v17.step_cycle_steps == v16.step_cycle_steps == 64
    assert v17._phase_body_fraction("lift", 1.0) == 0.0
    assert np.isclose(v17._phase_body_fraction("advance", 1.0), 0.70)
    assert np.isclose(v17._phase_body_fraction("land", 1.0), 1.0)

    observation, info = v17.reset(seed=17)
    assert info["desired_body_forward_m"] == 0.0
    observation, reward, terminated, truncated, info = v17.step(
        np.zeros(10, dtype=np.float32)
    )
    assert np.isfinite(reward)
    assert info["forward_margin_stage"] == 1
    assert info["body_progress_per_step_m"] == 0.0275
    assert info["minimum_forward_m"] == 0.200
    assert np.isfinite(info["body_progress_error_m"])
    assert info["body_progress_shortfall_m"] >= 0.0
    assert np.isfinite(info["body_progress_reward"])
    assert np.isfinite(info["body_stall_penalty"])
    v16.close()
    v17.close()


def test_v18_preserves_v17_targets_and_applies_seeded_perturbations():
    env = RobustForwardMarginV18Env(
        fixed_first_side="left",
        friction_scale=0.8,
        mass_scale=1.02,
        leg_mass_asymmetry=0.01,
        initial_joint_noise_rad=0.0035,
        initial_joint_velocity_noise_rad_s=0.01,
        motor_gain_scale=0.95,
        control_delay_steps=1,
    )
    observation_a, info_a = env.reset(seed=17)
    qpos_a = env.data.qpos[env.qpos_ids].copy()
    observation_b, _ = env.reset(seed=17)
    qpos_b = env.data.qpos[env.qpos_ids].copy()
    observation_c, _ = env.reset(seed=29)
    qpos_c = env.data.qpos[env.qpos_ids].copy()

    assert observation_a.shape == (82,)
    assert np.allclose(qpos_a, qpos_b)
    assert not np.allclose(qpos_a, qpos_c)
    assert env.stride_m == 0.042
    assert env.minimum_forward_m == 0.200
    assert env.step_cycle_steps == 64
    assert env.kp == 130.0 * 0.95
    assert env.landing_kp == 80.0 * 0.95
    assert info_a["robustness_stage"] == 1
    assert info_a["control_delay_steps"] == 1
    assert np.isclose(info_a["control_delay_seconds"], 0.04)
    assert np.isclose(
        info_a["perturbed_total_mass_kg"], env._nominal_total_mass_kg * 1.02,
        rtol=1.0e-5,
    )
    _, reward, _, _, step_info = env.step(np.ones(10, dtype=np.float32))
    assert np.isfinite(reward)
    assert step_info["friction_scale"] == 0.8
    env.close()


def test_v19_correction_is_bounded_and_landing_gated():
    class ConstantPolicy:
        def predict(self, observation, deterministic=True):
            return np.full(10, 0.25, dtype=np.float32), None

    env = LandingResidualV19Env(
        fixed_first_side="left",
        base_policy=ConstantPolicy(),
        fixed_domain={"mass_scale": 1.01},
        landing_correction_scale=0.025,
    )
    observation, info = env.reset(seed=17)
    assert observation.shape == (82,)
    assert info["mass_scale"] == 1.01
    assert env.stride_m == 0.042
    assert env.minimum_forward_m == 0.200
    assert env.step_cycle_steps == 64
    assert env._landing_gate("lift", 1.0, 0.5) == 0.0
    assert env._landing_gate("hold", 1.0, 0.5) == 0.0
    assert env._landing_gate("advance", 0.5, 0.5) == 0.0
    assert np.isclose(env._landing_gate("advance", 0.75, 0.5), 0.5)
    assert env._landing_gate("land", 0.1, 0.5) == 1.0
    assert np.isclose(env._landing_gate("settle", 0.5, 0.5), 0.5)

    _, reward, _, _, step_info = env.step(np.ones(10, dtype=np.float32))
    assert np.isfinite(reward)
    assert step_info["landing_correction_gate"] == 0.0
    assert step_info["applied_correction_max_abs"] == 0.0
    assert np.isclose(step_info["base_action_max_abs"], 0.25)

    env._step_count = env.lift_steps + env.lift_hold_steps + env.advance_steps
    env._current_observation = env._task_observation(advance_history=False)
    _, reward, _, _, step_info = env.step(np.ones(10, dtype=np.float32))
    assert np.isfinite(reward)
    assert step_info["landing_correction_gate"] == 1.0
    assert np.isclose(step_info["applied_correction_max_abs"], 0.025)
    env.close()


def test_v19_curriculum_sampling_is_seeded_and_stage_bounded():
    env = LandingResidualV19Env(curriculum_stage=2)
    _, info_a = env.reset(seed=29)
    domain_a = env.sampled_domain.copy()
    _, info_b = env.reset(seed=29)
    assert domain_a == env.sampled_domain
    assert 0.8 <= info_a["friction_scale"] <= 1.2
    assert 0.98 <= info_a["mass_scale"] <= 1.02
    assert -0.01 <= info_a["leg_mass_asymmetry"] <= 0.01
    assert 0.95 <= info_a["motor_gain_scale"] <= 1.05
    assert 0 <= info_a["control_delay_steps"] <= 2
    assert info_a["friction_scale"] == info_b["friction_scale"]
    env.set_curriculum_stage(3)
    _, info_c = env.reset(seed=43)
    assert 0.8 <= info_c["friction_scale"] <= 1.2
    env.close()


def test_v20_adds_contact_observation_and_starts_gate_in_late_hold():
    env = ContactAwareResidualV20Env(
        fixed_first_side="left", fixed_domain={}, curriculum_stage=1
    )
    observation, info = env.reset(seed=17)
    assert observation.shape == (90,)
    assert env.observation_space.shape == (90,)
    assert np.isfinite(observation).all()
    assert info["corrector_observation_size"] == 90
    assert env._landing_gate("lift", 1.0, 0.5) == 0.0
    assert env._landing_gate("hold", 0.5, 0.5) == 0.0
    assert np.isclose(env._landing_gate("hold", 0.75, 0.5), 0.5)
    assert env._landing_gate("advance", 0.1, 0.5) == 1.0
    assert env._landing_gate("land", 0.1, 0.5) == 1.0
    env.set_v20_stage(0)
    assert env.early_gate_blend == 0.0
    assert env.curriculum_stage == 1
    assert env._landing_gate("hold", 0.75, 0.5) == 0.0
    assert env._landing_gate("advance", 0.25, 0.5) == 0.0
    env.set_v20_stage(2)
    assert env.early_gate_blend == 1.0
    assert env.curriculum_stage == 3
    observation, reward, _, _, step_info = env.step(
        np.zeros(10, dtype=np.float32)
    )
    assert observation.shape == (90,)
    assert np.isfinite(observation).all()
    assert np.isfinite(reward)
    assert np.isfinite(observation[-8:]).all()
    assert "unload_excess_penalty_v20" in step_info
    assert "contact_recovery_penalty_v20" in step_info
    env.close()


def test_v21_splits_action_heads_and_adds_completed_step_memory():
    env = PhaseSplitResidualV21Env(
        fixed_first_side="left", fixed_domain={}, curriculum_stage=2
    )
    observation, info = env.reset(seed=17)
    assert observation.shape == (94,)
    assert env.observation_space.shape == (94,)
    assert env.action_space.shape == (20,)
    assert np.allclose(observation[-4:], 0.0)
    assert info["corrector_action_size"] == 20
    assert env._landing_head_mix("lift", 1.0) == 0.0
    assert env._landing_head_mix("advance", 0.5) == 0.0
    assert np.isclose(env._landing_head_mix("advance", 0.75), 0.5)
    assert env._landing_head_mix("land", 0.1) == 1.0

    action = np.concatenate(
        (np.full(10, 0.25, dtype=np.float32), np.full(10, -0.50, dtype=np.float32))
    )
    observation, reward, _, _, step_info = env.step(action)
    assert observation.shape == (94,)
    assert np.isfinite(observation).all()
    assert np.isfinite(reward)
    assert step_info["landing_head_mix"] == 0.0
    assert np.isclose(step_info["combined_correction_action_max_abs"], 0.25)

    env._step_count = env.step_cycle_steps - 1
    env._landing_peaks[0] = 48.0
    env._advance_forces[0] = [0.0] * 9 + [6.0]
    env._settle_left[0] = [20.0] * 4
    env._settle_right[0] = [20.0] * 4
    observation, _, _, _, step_info = env.step(np.zeros(20, dtype=np.float32))
    assert step_info["step_memory_updated"]
    assert np.isclose(observation[-4], 0.48)
    assert np.isclose(observation[-3], 0.9)
    assert np.isclose(observation[-2], 1.0)
    assert np.isfinite(observation[-1])
    env.close()


def test_v21_stage_enables_only_bounded_contact_conditioned_lift_gate():
    env = PhaseSplitResidualV21Env(
        fixed_first_side="left", fixed_domain={}, conditional_lift_gate_scale=0.0
    )
    env.reset(seed=17)
    assert env._landing_gate("lift", 1.0, 0.5) == 0.0
    env.set_v21_stage(2)
    assert env.curriculum_stage == 2
    assert env.early_gate_blend == 0.75
    assert env.conditional_lift_gate_scale == 0.35
    gate = env._landing_gate("lift", 1.0, 0.5)
    assert 0.0 <= gate <= 0.35
    env.set_v21_stage(0)
    assert env.conditional_lift_gate_scale == 0.0
    assert env._landing_gate("lift", 1.0, 0.5) == 0.0
    env.close()


def test_v22_probe_is_bounded_local_and_zero_at_phase_edges():
    env = CounterfactualProbeV22Env(
        fixed_first_side="left",
        fixed_domain={},
        probe_step=1,
        probe_phase="lift",
        probe_joint="left_knee_pitch",
        probe_offset_rad=0.01,
    )
    env.reset(seed=17)
    base = np.zeros(10, dtype=np.float64)
    edge = env._residual_joint_target_offset(base, "lift", 1.0)
    middle = env._residual_joint_target_offset(base, "lift", 0.5)
    wrong_phase = env._residual_joint_target_offset(base, "advance", 0.5)
    assert np.isclose(edge[2], 0.0)
    assert np.isclose(middle[2], 0.01)
    assert np.count_nonzero(np.abs(middle) > 1.0e-12) == 1
    assert np.allclose(wrong_phase, 0.0)
    env._step_count = env.step_cycle_steps
    wrong_step = env._residual_joint_target_offset(base, "lift", 0.5)
    assert np.allclose(wrong_step, 0.0)
    env.close()


def test_v22_stance_hip_roll_probe_is_mirrored_by_first_swing_side():
    left_first = FirstStepStanceHipRollV22Env(
        fixed_first_side="left", fixed_domain={}, stance_hip_roll_lift_offset_rad=0.006
    )
    left_first.reset(seed=17)
    offset = left_first._residual_joint_target_offset(
        np.zeros(10, dtype=np.float64), "lift", 0.5
    )
    assert np.isclose(offset[5], 0.006)
    assert np.count_nonzero(np.abs(offset) > 1.0e-12) == 1
    left_first.close()

    right_first = FirstStepStanceHipRollV22Env(
        fixed_first_side="right", fixed_domain={}, stance_hip_roll_lift_offset_rad=0.006
    )
    right_first.reset(seed=17)
    offset = right_first._residual_joint_target_offset(
        np.zeros(10, dtype=np.float64), "lift", 0.5
    )
    assert np.isclose(offset[0], -0.006)
    assert np.count_nonzero(np.abs(offset) > 1.0e-12) == 1
    right_first._step_count = right_first.step_cycle_steps
    assert np.allclose(
        right_first._residual_joint_target_offset(
            np.zeros(10, dtype=np.float64), "lift", 0.5
        ),
        0.0,
    )
    right_first.close()


def test_v11_applies_seeded_physics_and_initial_state_perturbations():
    env = RobustSoftLandingV11Env(
        fixed_first_side="left",
        friction_scale=0.7,
        mass_scale=1.05,
        leg_mass_asymmetry=0.03,
        initial_joint_noise_rad=0.0035,
        initial_joint_velocity_noise_rad_s=0.01,
    )
    observation_a, info_a = env.reset(seed=17)
    qpos_a = env.data.qpos[env.qpos_ids].copy()
    observation_b, _ = env.reset(seed=17)
    qpos_b = env.data.qpos[env.qpos_ids].copy()
    observation_c, _ = env.reset(seed=29)
    qpos_c = env.data.qpos[env.qpos_ids].copy()
    assert np.allclose(qpos_a, qpos_b)
    assert not np.allclose(qpos_a, qpos_c)
    assert observation_a.shape == (82,)
    assert info_a["friction_scale"] == 0.7
    assert info_a["mass_scale"] == 1.05
    assert info_a["leg_mass_asymmetry"] == 0.03
    assert np.isclose(env.model.body_mass.sum(), 1.05 * 8.359301, rtol=1.0e-5)
    env.close()


def test_v12_curriculum_sampling_is_seeded_and_stage_bounded():
    env = CurriculumSoftLandingV12Env(fixed_first_side="left", curriculum_stage=2)
    observation_a, info_a = env.reset(seed=29)
    qpos_a = env.data.qpos[env.qpos_ids].copy()
    observation_b, info_b = env.reset(seed=29)
    qpos_b = env.data.qpos[env.qpos_ids].copy()
    assert observation_a.shape == (82,)
    assert np.allclose(qpos_a, qpos_b)
    assert info_a["sampled_friction_scale"] == info_b["sampled_friction_scale"]
    assert 0.8 <= info_a["sampled_friction_scale"] <= 1.2
    assert 0.98 <= info_a["sampled_mass_scale"] <= 1.02
    assert -0.01 <= info_a["sampled_leg_mass_asymmetry"] <= 0.01
    env.set_curriculum_stage(0)
    _, info_c = env.reset(seed=43)
    assert 0.98 <= info_c["sampled_friction_scale"] <= 1.02
    assert 0.995 <= info_c["sampled_mass_scale"] <= 1.005
    assert -0.001 <= info_c["sampled_leg_mass_asymmetry"] <= 0.001
    env.close()


def test_world_fixed_render_camera_configuration_does_not_track_base():
    env = SoftLandingEightStepV10Env()
    env.configure_render_camera(
        mode="world_fixed",
        lookat=(0.0, 0.12, 0.18),
        distance=1.8,
        azimuth=160.0,
        elevation=-10.0,
    )
    assert env.render_camera_mode == "world_fixed"
    assert np.allclose(env.render_camera_lookat, [0.0, 0.12, 0.18])
    assert env.render_camera_distance == 1.8
    assert env.render_camera_azimuth == 160.0
    assert env.render_camera_elevation == -10.0
    env.close()


def test_actuator_limits_default_to_the_ak45_hardware_values():
    env = KHR3HVEnv()
    expected_torque = np.array([24.0, 24.0, 24.0, 24.0, 7.0] * 2)
    expected_speed = np.array([40.0, 40.0, 40.0, 40.0, 150.0] * 2) * 2.0 * np.pi / 60.0
    assert np.array_equal(env.torque_limit, expected_torque)
    assert np.array_equal(env.max_speed, expected_speed)
    assert np.allclose(env.action_scale, 0.5 * expected_speed * env.config.control_dt)
    env.close()


def test_actuator_limits_are_configurable_and_reach_the_task_environments():
    faster = replace(
        KHRConfig(),
        max_speed_rpm=(60.0,) * 4 + (200.0,) + (60.0,) * 4 + (200.0,),
        torque_limit_n_m=(30.0,) * 4 + (9.0,) + (30.0,) * 4 + (9.0,),
    )
    env = KHR3HVEnv(config=faster)
    assert env.torque_limit[0] == 30.0 and env.torque_limit[4] == 9.0
    assert np.isclose(env.max_speed[0], 60.0 * 2.0 * np.pi / 60.0)
    env.close()

    class FasterV22Env(FirstStepStanceHipRollV22Env):
        @classmethod
        def base_config(cls):
            return faster

    task_env = FasterV22Env()
    assert np.array_equal(task_env.torque_limit, np.array([30.0] * 4 + [9.0] + [30.0] * 4 + [9.0]))
    # Task settings the family sets on top of the base config must survive.
    assert task_env.config.enhanced_collisions
    assert task_env.config.torque_penalty_scale == 0.002
    task_env.close()


def test_actuator_limits_reject_wrong_length_and_non_positive_entries():
    for broken in (
        replace(KHRConfig(), torque_limit_n_m=(24.0,) * 9),
        replace(KHRConfig(), torque_limit_n_m=(24.0,) * 9 + (0.0,)),
        replace(KHRConfig(), max_speed_rpm=(40.0,) * 9 + (-150.0,)),
    ):
        with pytest.raises(ValueError):
            KHR3HVEnv(config=broken)
