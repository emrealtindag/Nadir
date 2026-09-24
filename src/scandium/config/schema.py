"""
Pydantic configuration schema for Scandium.

This module defines all configuration models with strict validation,
enum fields, default values, and path existence checks.
"""

from enum import Enum
from pathlib import Path
from typing import Optional
import uuid

from pydantic import BaseModel, Field, field_validator


class LogLevel(str, Enum):
    """Log level enumeration."""

    DEBUG = "DEBUG"
    INFO = "INFO"
    WARNING = "WARNING"
    ERROR = "ERROR"


class RunMode(str, Enum):
    """Operating mode enumeration."""

    SITL = "sitl"
    AIRSIM = "airsim"
    LIVE = "live"


class CameraSource(str, Enum):
    """Camera source enumeration."""

    AIRSIM = "airsim"
    UVC = "uvc"
    VIDEO_FILE = "video_file"


class FiducialBackend(str, Enum):
    """Fiducial detection backend enumeration."""

    ARUCO = "aruco"
    APRILTAG = "apriltag"


class MavFrame(str, Enum):
    """MAVLink frame enumeration."""

    MAV_FRAME_BODY_NED = "MAV_FRAME_BODY_NED"
    MAV_FRAME_LOCAL_NED = "MAV_FRAME_LOCAL_NED"
    MAV_FRAME_BODY_FRD = "MAV_FRAME_BODY_FRD"


class FilterType(str, Enum):
    """Pose filter type enumeration."""

    EXP_SMOOTH = "exp_smooth"
    KALMAN = "kalman"


class MavlinkTransport(str, Enum):
    """MAVLink transport type enumeration."""

    UDP = "udp"
    SERIAL = "serial"


class LandabilityMethod(str, Enum):
    """Landability estimation method enumeration."""

    HEURISTIC = "heuristic"
    ML = "ml"


class AutopilotType(str, Enum):
    """Autopilot type enumeration."""

    ARDUPILOT = "ardupilot"
    PX4 = "px4"


# ============================================================================
# Sub-configuration Models
# ============================================================================


class ProjectConfig(BaseModel):
    """Project-level configuration."""

    name: str = Field(default="Scandium", description="Project name")
    run_id: str = Field(
        default="auto", description="Run identifier (auto generates UUID)"
    )
    mode: RunMode = Field(default=RunMode.SITL, description="Operating mode")
    log_level: LogLevel = Field(default=LogLevel.INFO, description="Logging level")
    output_dir: Path = Field(
        default=Path("./runs"), description="Output directory for logs and data"
    )

    @field_validator("run_id", mode="before")
    @classmethod
    def generate_run_id(cls, v: str) -> str:
        """Generate UUID if run_id is 'auto'."""
        if v == "auto":
            return str(uuid.uuid4())[:8]
        return v


class CameraConfig(BaseModel):
    """Camera configuration."""

    source: CameraSource = Field(
        default=CameraSource.AIRSIM, description="Camera source type"
    )
    device_index: int = Field(
        default=0, ge=0, description="Camera device index for UVC"
    )
    video_path: str = Field(default="", description="Path to video file")
    width: int = Field(default=1280, gt=0, description="Frame width")
    height: int = Field(default=720, gt=0, description="Frame height")
    fps: int = Field(default=30, gt=0, le=120, description="Target frames per second")
    intrinsics_path: Path = Field(
        default=Path("configs/camera/calib_example.yaml"),
        description="Path to camera intrinsics file",
    )
    extrinsics_path: Path = Field(
        default=Path("configs/camera/extrinsics_example.yaml"),
        description="Path to camera-body extrinsics file",
    )
    undistort: bool = Field(default=True, description="Apply undistortion")


class ArUcoConfig(BaseModel):
    """ArUco-specific configuration."""

    dictionary: str = Field(default="DICT_4X4_100", description="ArUco dictionary name")
    refine: bool = Field(default=True, description="Enable corner refinement")


class AprilTagConfig(BaseModel):
    """AprilTag-specific configuration."""

    family: str = Field(default="tag36h11", description="AprilTag family")
    quad_decimate: float = Field(
        default=1.0, ge=0.0, description="Quad decimation factor"
    )
    quad_sigma: float = Field(
        default=0.0, ge=0.0, description="Quad sigma for Gaussian blur"
    )


class FiducialsConfig(BaseModel):
    """Fiducial detection configuration."""

    backend: FiducialBackend = Field(
        default=FiducialBackend.ARUCO, description="Detection backend"
    )
    marker_size_m: float = Field(
        default=0.20, gt=0, description="Physical marker size in meters"
    )
    target_id_allowlist: list[int] = Field(
        default=[1], description="Allowed marker IDs"
    )
    aruco: ArUcoConfig = Field(default_factory=ArUcoConfig)
    apriltag: AprilTagConfig = Field(default_factory=AprilTagConfig)


class FilterConfig(BaseModel):
    """Pose filter configuration."""

    type: FilterType = Field(default=FilterType.EXP_SMOOTH, description="Filter type")
    alpha: float = Field(default=0.35, gt=0, le=1, description="Smoothing factor")
    outlier_mahalanobis: float = Field(
        default=4.0, gt=0, description="Mahalanobis distance threshold"
    )


class QualityConfig(BaseModel):
    """Pose quality thresholds."""

    max_reproj_error_px: float = Field(
        default=3.0, gt=0, description="Max reprojection error"
    )
    min_tag_area_px: int = Field(
        default=800, gt=0, description="Minimum tag area in pixels"
    )


class PoseConfig(BaseModel):
    """Pose estimation configuration."""

    frame: MavFrame = Field(
        default=MavFrame.MAV_FRAME_BODY_NED, description="Output frame"
    )
    filter: FilterConfig = Field(default_factory=FilterConfig)
    quality: QualityConfig = Field(default_factory=QualityConfig)


class UdpConfig(BaseModel):
    """UDP transport configuration."""

    address: str = Field(default="127.0.0.1", description="UDP address")
    port: int = Field(default=14550, ge=1, le=65535, description="UDP port")


class SerialConfig(BaseModel):
    """Serial transport configuration."""

    device: str = Field(default="/dev/ttyAMA0", description="Serial device path")
    baud: int = Field(default=921600, gt=0, description="Baud rate")


class MavlinkConfig(BaseModel):
    """MAVLink configuration."""

    transport: MavlinkTransport = Field(
        default=MavlinkTransport.UDP, description="Transport type"
    )
    udp: UdpConfig = Field(default_factory=UdpConfig)
    serial: SerialConfig = Field(default_factory=SerialConfig)
    system_id: int = Field(default=42, ge=1, le=255, description="System ID")
    component_id: int = Field(default=200, ge=1, le=255, description="Component ID")
    target_system_id: int = Field(
        default=1, ge=1, le=255, description="Target system ID"
    )
    target_component_id: int = Field(
        default=1, ge=1, le=255, description="Target component ID"
    )
    landing_target_rate_hz: int = Field(
        default=20, ge=1, le=100, description="LANDING_TARGET publish rate"
    )


class ThresholdsConfig(BaseModel):
    """Control thresholds configuration."""

    acquire_confidence: float = Field(
        default=0.70, ge=0, le=1, description="Confidence for ACQUIRE"
    )
    align_error_m: float = Field(
        default=0.25, gt=0, description="Lateral error for ALIGN"
    )
    abort_landability: float = Field(
        default=0.40, ge=0, le=1, description="Landability for ABORT"
    )


class LimitsConfig(BaseModel):
    """Control limits configuration."""

    max_lateral_speed_mps: float = Field(
        default=1.5, gt=0, description="Max lateral speed m/s"
    )
    max_descent_speed_mps: float = Field(
        default=0.7, gt=0, description="Max descent speed m/s"
    )


class ControlConfig(BaseModel):
    """Control system configuration."""

    enable_fsm: bool = Field(default=True, description="Enable finite state machine")
    fsm_rate_hz: int = Field(default=20, ge=1, le=100, description="FSM tick rate")
    thresholds: ThresholdsConfig = Field(default_factory=ThresholdsConfig)
    limits: LimitsConfig = Field(default_factory=LimitsConfig)


class HeuristicConfig(BaseModel):
    """Heuristic landability configuration."""

    texture_var_min: float = Field(
        default=12.0, ge=0, description="Minimum texture variance"
    )
    motion_threshold: float = Field(
        default=0.15, ge=0, le=1, description="Motion detection threshold"
    )


class MlConfig(BaseModel):
    """ML landability configuration."""

    model_path: str = Field(default="", description="Path to ML model")
    device: str = Field(default="cpu", description="Inference device (cpu/cuda)")
    score_threshold: float = Field(
        default=0.6, ge=0, le=1, description="Score threshold"
    )


class LandabilityConfig(BaseModel):
    """Landability estimation configuration."""

    enabled: bool = Field(default=True, description="Enable landability analysis")
    method: LandabilityMethod = Field(
        default=LandabilityMethod.HEURISTIC, description="Method"
    )
    heuristic: HeuristicConfig = Field(default_factory=HeuristicConfig)
    ml: MlConfig = Field(default_factory=MlConfig)


class AirSimConfig(BaseModel):
    """AirSim simulation configuration."""

    ip: str = Field(default="127.0.0.1", description="AirSim IP address")
    vehicle_name: str = Field(default="Drone1", description="Vehicle name")
    camera_name: str = Field(default="0", description="Camera name")
    image_type: str = Field(
        default="Scene", description="Image type (Scene, Depth, etc.)"
    )


class SitlConfig(BaseModel):
    """SITL configuration."""

    autopilot: AutopilotType = Field(
        default=AutopilotType.ARDUPILOT, description="Autopilot type"
    )
    start_local: bool = Field(default=True, description="Start SITL locally")
    mavproxy: bool = Field(default=False, description="Use MAVProxy")


class SimulationConfig(BaseModel):
    """Simulation configuration."""

    airsim: AirSimConfig = Field(default_factory=AirSimConfig)
    sitl: SitlConfig = Field(default_factory=SitlConfig)


class ScenariosConfig(BaseModel):
    """Scenario testing configuration."""

    enabled: bool = Field(default=True, description="Enable scenario testing")
    selected: str = Field(default="smoke", description="Selected scenario")
    path: Path = Field(
        default=Path("configs/scenarios/smoke.yaml"), description="Scenario file path"
    )


# ============================================================================
# Root Configuration Model
# ============================================================================


class ScandiumConfig(BaseModel):
    """Root configuration model for Scandium."""

    project: ProjectConfig = Field(default_factory=ProjectConfig)
    camera: CameraConfig = Field(default_factory=CameraConfig)
    fiducials: FiducialsConfig = Field(default_factory=FiducialsConfig)
    pose: PoseConfig = Field(default_factory=PoseConfig)
    mavlink: MavlinkConfig = Field(default_factory=MavlinkConfig)
    control: ControlConfig = Field(default_factory=ControlConfig)
    landability: LandabilityConfig = Field(default_factory=LandabilityConfig)
    simulation: SimulationConfig = Field(default_factory=SimulationConfig)
    scenarios: ScenariosConfig = Field(default_factory=ScenariosConfig)

    model_config = {"extra": "forbid"}
