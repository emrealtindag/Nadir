"""
Configuration validation for Scandium.

Provides cross-field validation and runtime checks beyond Pydantic schema validation.
"""

from pathlib import Path
from typing import Optional

from scandium.config.schema import ScandiumConfig, CameraSource, FiducialBackend


class ConfigurationError(Exception):
    """Configuration validation error."""

    pass


def validate_config(config: ScandiumConfig, config_dir: Optional[Path] = None) -> None:
    """
    Perform cross-field and runtime validation on configuration.

    Args:
        config: ScandiumConfig instance to validate.
        config_dir: Optional base directory for path resolution.

    Raises:
        ConfigurationError: If validation fails.
    """
    errors: list[str] = []

    # Validate camera configuration
    errors.extend(_validate_camera(config, config_dir))

    # Validate fiducial configuration
    errors.extend(_validate_fiducials(config))

    # Validate MAVLink configuration
    errors.extend(_validate_mavlink(config))

    # Validate control configuration
    errors.extend(_validate_control(config))

    # Validate landability configuration
    errors.extend(_validate_landability(config))

    if errors:
        error_msg = "Configuration validation failed:\n" + "\n".join(
            f"  - {e}" for e in errors
        )
        raise ConfigurationError(error_msg)


def _validate_camera(config: ScandiumConfig, config_dir: Optional[Path]) -> list[str]:
    """Validate camera configuration."""
    errors: list[str] = []

    if config.camera.source == CameraSource.VIDEO_FILE:
        if not config.camera.video_path:
            errors.append("camera.video_path is required when source is 'video_file'")

    if config.camera.source == CameraSource.UVC:
        if config.camera.device_index < 0:
            errors.append("camera.device_index must be non-negative for UVC source")

    # Note: Path existence checks are optional at load time
    # as paths may be relative to working directory at runtime

    return errors


def _validate_fiducials(config: ScandiumConfig) -> list[str]:
    """Validate fiducial detection configuration."""
    errors: list[str] = []

    if not config.fiducials.target_id_allowlist:
        errors.append("fiducials.target_id_allowlist must contain at least one ID")

    if config.fiducials.marker_size_m <= 0:
        errors.append("fiducials.marker_size_m must be positive")

    # Validate ArUco dictionary name
    if config.fiducials.backend == FiducialBackend.ARUCO:
        valid_dicts = {
            "DICT_4X4_50",
            "DICT_4X4_100",
            "DICT_4X4_250",
            "DICT_4X4_1000",
            "DICT_5X5_50",
            "DICT_5X5_100",
            "DICT_5X5_250",
            "DICT_5X5_1000",
            "DICT_6X6_50",
            "DICT_6X6_100",
            "DICT_6X6_250",
            "DICT_6X6_1000",
            "DICT_7X7_50",
            "DICT_7X7_100",
            "DICT_7X7_250",
            "DICT_7X7_1000",
            "DICT_ARUCO_ORIGINAL",
            "DICT_APRILTAG_16h5",
            "DICT_APRILTAG_25h9",
            "DICT_APRILTAG_36h10",
            "DICT_APRILTAG_36h11",
        }
        if config.fiducials.aruco.dictionary not in valid_dicts:
            errors.append(
                f"Invalid ArUco dictionary: {config.fiducials.aruco.dictionary}"
            )

    return errors


def _validate_mavlink(config: ScandiumConfig) -> list[str]:
    """Validate MAVLink configuration."""
    errors: list[str] = []

    # Validate rate
    if (
        config.mavlink.landing_target_rate_hz < 1
        or config.mavlink.landing_target_rate_hz > 100
    ):
        errors.append("mavlink.landing_target_rate_hz must be between 1 and 100")

    # Validate IDs
    if config.mavlink.system_id == config.mavlink.target_system_id:
        # This is actually allowed in some scenarios, just a warning
        pass

    return errors


def _validate_control(config: ScandiumConfig) -> list[str]:
    """Validate control configuration."""
    errors: list[str] = []

    # Validate thresholds make sense
    if (
        config.control.thresholds.abort_landability
        >= config.control.thresholds.acquire_confidence
    ):
        errors.append(
            "control.thresholds.abort_landability should be less than acquire_confidence"
        )

    # Validate limits are positive
    if config.control.limits.max_lateral_speed_mps <= 0:
        errors.append("control.limits.max_lateral_speed_mps must be positive")

    if config.control.limits.max_descent_speed_mps <= 0:
        errors.append("control.limits.max_descent_speed_mps must be positive")

    return errors


def _validate_landability(config: ScandiumConfig) -> list[str]:
    """Validate landability configuration."""
    errors: list[str] = []

    from scandium.config.schema import LandabilityMethod

    if config.landability.method == LandabilityMethod.ML:
        if not config.landability.ml.model_path:
            errors.append("landability.ml.model_path is required when method is 'ml'")

    return errors
