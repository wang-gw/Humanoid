import numpy as np

from periodic_gaits import NaturalGaitConfig, PeriodicGaitEnv


def test_interface_and_absolute_action_mapping():
    env = PeriodicGaitEnv()
    observation, _ = env.reset(seed=3)
    assert observation.shape == (44,)
    assert env.action_space.shape == (10,)
    assert np.all(env.action_low >= env.joint_limits[:, 0])
    assert np.all(env.action_high <= env.joint_limits[:, 1])
    target = env.action_center + env.action_scale * np.ones(10)
    assert np.allclose(target, env.action_high)
    env.close()


def test_actor_observation_is_finite_and_step_is_100hz():
    env = PeriodicGaitEnv()
    observation, _ = env.reset(seed=5)
    observation, reward, terminated, truncated, info = env.step(np.zeros(10, dtype=np.float32))
    assert env.frame_skip == 5
    assert np.isclose(env.config.control_dt, 0.01)
    assert observation.shape == (44,)
    assert np.isfinite(observation).all()
    assert np.isfinite(reward)
    assert "left_force_n" in info  # reward/metrics only, never actor observation
    assert "angular_cost" in info
    assert "gait_pattern_satisfied" in info
    assert not truncated
    env.close()


def test_walking_clocks_are_half_cycle_opposed():
    env = PeriodicGaitEnv()
    env.reset(seed=7)
    left = env._clock(env.config.left_phase_offset)
    right = env._clock(env.config.right_phase_offset)
    assert np.isclose(left, -right)
    env.close()


def test_v2_keeps_interface_and_exposes_natural_swing_costs():
    env = PeriodicGaitEnv(config=NaturalGaitConfig())
    observation, _ = env.reset(seed=11)
    observation, reward, _, _, info = env.step(np.zeros(10, dtype=np.float32))
    assert observation.shape == (44,)
    assert np.isfinite(reward)
    assert info["natural_swing_cost"] > 0.0
    assert info["clearance_cost"] >= 0.0
    assert info["swing_forward_cost"] >= 0.0
    assert info["knee_flexion_cost"] >= 0.0
    assert info["knee_forward_cost"] >= 0.0
    assert np.isfinite(info["left_knee_forward_m"])
    env.close()
