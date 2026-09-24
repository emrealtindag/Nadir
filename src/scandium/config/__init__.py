"""Configuration module for Scandium."""

from scandium.config.schema import (
    CameraConfig,
    ControlConfig,
    FiducialsConfig,
    LandabilityConfig,
    MavlinkConfig,
    PoseConfig,
    ProjectConfig,
    ScandiumConfig,
    SimulationConfig,
)
from scandium.config.loader import load_config
from scandium.config.validation import validate_config

__all__ = [
    "CameraConfig",
    "ControlConfig",
    "FiducialsConfig",
    "LandabilityConfig",
    "MavlinkConfig",
    "PoseConfig",
    "ProjectConfig",
    "ScandiumConfig",
    "SimulationConfig",
    "load_config",
    "validate_config",
]
