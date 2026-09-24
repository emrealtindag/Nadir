"""
LANDING_TARGET message handling for Scandium.

Provides message construction and rate-limited publishing.
"""

from dataclasses import dataclass
from typing import Optional
import time

from scandium.mavlink.transport import MavlinkTransport
from scandium.utils.throttling import RateLimiter
from scandium.logging.setup import get_logger

logger = get_logger(__name__)


# MAVLink frame constants
MAV_FRAME_BODY_NED = 8
MAV_FRAME_LOCAL_NED = 1
MAV_FRAME_BODY_FRD = 12

# LANDING_TARGET_TYPE constants
LANDING_TARGET_TYPE_LIGHT_BEACON = 0
LANDING_TARGET_TYPE_RADIO_BEACON = 1
LANDING_TARGET_TYPE_VISION_FIDUCIAL = 2
LANDING_TARGET_TYPE_VISION_OTHER = 3


@dataclass
class LandingTargetData:
    """
    LANDING_TARGET message data.

    Attributes:
        timestamp_us: Timestamp in microseconds.
        angle_x: X-axis angular offset (rad). Positive = target right.
        angle_y: Y-axis angular offset (rad). Positive = target down.
        distance_m: Distance to target in meters.
        x_m: X position in specified frame (m).
        y_m: Y position in specified frame (m).
        z_m: Z position in specified frame (m).
        position_valid: True if x, y, z are valid.
        frame: MAVLink frame ID.
    """

    timestamp_us: int
    angle_x: float
    angle_y: float
    distance_m: float
    x_m: float = 0.0
    y_m: float = 0.0
    z_m: float = 0.0
    position_valid: bool = True
    frame: int = MAV_FRAME_BODY_NED
    quaternion: Optional[list[float]] = None


def build_landing_target(
    angle_x: float,
    angle_y: float,
    distance_m: float,
    x_m: float = 0.0,
    y_m: float = 0.0,
    z_m: float = 0.0,
    position_valid: bool = True,
    frame: int = MAV_FRAME_BODY_NED,
    timestamp_us: Optional[int] = None,
) -> LandingTargetData:
    """
    Build LANDING_TARGET data structure.

    Args:
        angle_x: X-axis angular offset (rad).
        angle_y: Y-axis angular offset (rad).
        distance_m: Distance to target (m).
        x_m: X position (m).
        y_m: Y position (m).
        z_m: Z position (m).
        position_valid: Whether position is valid.
        frame: MAVLink frame.
        timestamp_us: Timestamp (uses current time if None).

    Returns:
        LandingTargetData ready for sending.
    """
    if timestamp_us is None:
        timestamp_us = int(time.time() * 1_000_000)

    return LandingTargetData(
        timestamp_us=timestamp_us,
        angle_x=angle_x,
        angle_y=angle_y,
        distance_m=distance_m,
        x_m=x_m,
        y_m=y_m,
        z_m=z_m,
        position_valid=position_valid,
        frame=frame,
    )


class LandingTargetPublisher:
    """
    Rate-limited LANDING_TARGET publisher.

    Manages message publishing at configurable rate.
    """

    def __init__(
        self,
        transport: MavlinkTransport,
        rate_hz: int = 20,
        target_num: int = 0,
    ) -> None:
        """
        Initialize publisher.

        Args:
            transport: MAVLink transport instance.
            rate_hz: Publishing rate in Hz.
            target_num: Target number (usually 0).
        """
        self._transport = transport
        self._rate_limiter = RateLimiter(rate_hz)
        self._target_num = target_num
        self._msg_count = 0

    def publish(
        self,
        data: LandingTargetData,
        force: bool = False,
    ) -> bool:
        """
        Publish LANDING_TARGET message.

        Args:
            data: Landing target data.
            force: Force publish, ignoring rate limit.

        Returns:
            True if message was sent.
        """
        if not force and not self._rate_limiter.should_run():
            return False

        success = self._transport.send_landing_target(
            timestamp_us=data.timestamp_us,
            target_num=self._target_num,
            frame=data.frame,
            angle_x=data.angle_x,
            angle_y=data.angle_y,
            distance=data.distance_m,
            x=data.x_m,
            y=data.y_m,
            z=data.z_m,
            q=data.quaternion,
            position_valid=1 if data.position_valid else 0,
        )

        if success:
            self._msg_count += 1
            logger.debug(
                "landing_target_sent",
                angle_x=f"{data.angle_x:.4f}",
                angle_y=f"{data.angle_y:.4f}",
                distance=f"{data.distance_m:.2f}",
                count=self._msg_count,
            )

        return success

    def publish_from_pose(
        self,
        tvec: "NDArray[float]",
        angle_x: float,
        angle_y: float,
        position_valid: bool = True,
        frame: int = MAV_FRAME_BODY_NED,
        force: bool = False,
    ) -> bool:
        """
        Publish from pose estimation result.

        Args:
            tvec: Translation vector (x, y, z) in body frame.
            angle_x: X-axis angle (rad).
            angle_y: Y-axis angle (rad).
            position_valid: Whether position is valid.
            frame: MAVLink frame.
            force: Force publish.

        Returns:
            True if message was sent.
        """
        import numpy as np

        distance = float(np.linalg.norm(tvec))

        data = build_landing_target(
            angle_x=angle_x,
            angle_y=angle_y,
            distance_m=distance,
            x_m=float(tvec[0]),
            y_m=float(tvec[1]),
            z_m=float(tvec[2]),
            position_valid=position_valid,
            frame=frame,
        )

        return self.publish(data, force=force)

    @property
    def message_count(self) -> int:
        """Get total messages sent."""
        return self._msg_count

    @property
    def rate_hz(self) -> float:
        """Get configured rate."""
        return self._rate_limiter.rate_hz

    def reset(self) -> None:
        """Reset publisher state."""
        self._msg_count = 0
        self._rate_limiter.reset()
