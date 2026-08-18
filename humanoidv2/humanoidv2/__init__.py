"""Humanoidv2 reinforcement-learning environments."""

from .khr3hv_env import KHR3HVEnv
from .khr3hv_v2_env import KHR3HVV2Env, KHR3HVV21Env, KHR3HVV22Env
from .khr3hv_v3_env import KHR3HVV3Env, KHR3HVV31Env, KHR3HVV32Env, KHR3HVV33Env, KHR3HVV34Env
from .single_support_v4_env import SingleSupportV4Env
from .single_step_v5_env import SingleStepV5Env
from .two_step_v6_env import TwoStepV6Env
from .four_step_v7_env import FourStepV7Env
from .eight_step_v8_env import EightStepV8Env
from .fast_eight_step_v9_env import FastEightStepV9Env
from .soft_landing_v10_env import SoftLandingEightStepV10Env
from .robust_soft_landing_v11_env import RobustSoftLandingV11Env
from .curriculum_soft_landing_v12_env import CurriculumSoftLandingV12Env
from .corrected_forward_v13_env import CorrectedForwardV13Env
from .dynamic_forward_v14_env import DynamicForwardV14Env
from .dynamic_forward_v15_env import DynamicForwardV15Env
from .dynamic_forward_v16_env import DynamicForwardV16Env
from .forward_margin_v17_env import ForwardMarginV17Env
from .robust_forward_margin_v18_env import RobustForwardMarginV18Env
from .landing_residual_v19_env import LandingResidualV19Env, V19_DOMAIN_RANGES
from .contact_aware_residual_v20_env import ContactAwareResidualV20Env
from .phase_split_residual_v21_env import PhaseSplitResidualV21Env
from .counterfactual_probe_v22_env import (
    CounterfactualProbeV22Env,
    FirstStepStanceHipRollV22Env,
)

__all__ = [
    "KHR3HVEnv",
    "KHR3HVV2Env",
    "KHR3HVV21Env",
    "KHR3HVV22Env",
    "KHR3HVV3Env",
    "KHR3HVV31Env",
    "KHR3HVV32Env",
    "KHR3HVV33Env",
    "KHR3HVV34Env",
    "SingleSupportV4Env",
    "SingleStepV5Env",
    "TwoStepV6Env",
    "FourStepV7Env",
    "EightStepV8Env",
    "FastEightStepV9Env",
    "SoftLandingEightStepV10Env",
    "RobustSoftLandingV11Env",
    "CurriculumSoftLandingV12Env",
    "CorrectedForwardV13Env",
    "DynamicForwardV14Env",
    "DynamicForwardV15Env",
    "DynamicForwardV16Env",
    "ForwardMarginV17Env",
    "RobustForwardMarginV18Env",
    "LandingResidualV19Env",
    "V19_DOMAIN_RANGES",
    "ContactAwareResidualV20Env",
    "PhaseSplitResidualV21Env",
    "CounterfactualProbeV22Env",
    "FirstStepStanceHipRollV22Env",
]
