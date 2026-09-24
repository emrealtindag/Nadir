"""
MAVLink offboard control interface for Scandium.

Provides high-level offboard control commands for autopilot interaction
during precision landing operations.
"""

from dataclasses import dataclass
from typing import Optional
import time

from scandium.mavlink.transport import MavlinkTransport
from scandium.logging.setup import get_logger

logger = get_logger(__name__)


@dataclass
class PositionSetpoint:
    """
    Position setpoint for offboard control.

    Attributes:
        x: X position in local NED frame (meters).
        y: Y position in local NED frame (meters).
        z: Z position in local NED frame (meters, negative up).
        yaw: Yaw angle in radians.
    """

    x: float
    y: float
    z: float
    yaw: float = 0.0


@dataclass
class VelocitySetpoint:
    """
    Velocity setpoint for offboard control.

    Attributes:
        vx: X velocity in local NED frame (m/s).
        vy: Y velocity in local NED frame (m/s).
        vz: Z velocity in local NED frame (m/s, positive down).
        yaw_rate: Yaw rate in rad/s.
    """

    vx: float
    vy: float
    vz: float
    yaw_rate: float = 0.0


class OffboardController:
    """
    Offboard flight mode controller for PX4 and ArduPilot.

    Provides velocity and position setpoint commands via MAVLink
    SET_POSITION_TARGET_LOCAL_NED messages.
    """

    # Type masks for SET_POSITION_TARGET_LOCAL_NED
    TYPE_MASK_POS = 0b0000111111111000  # Position only
    TYPE_MASK_VEL = 0b0000111111000111  # Velocity only
    TYPE_MASK_POS_VEL = 0b0000111111000000  # Position and velocity

    def __init__(
        self,
        transport: MavlinkTransport,
        rate_hz: float = 20.0,
    ) -> None:
        """
        Initialize offboard controller.

        Args:
            transport: MAVLink transport instance.
            rate_hz: Setpoint publishing rate in Hz.
        """
        self._transport = transport
        self._rate_hz = rate_hz
        self._interval_s = 1.0 / rate_hz
        self._last_send_time: float = 0.0
        self._offboard_enabled = False

    def send_velocity_setpoint(
        self,
        setpoint: VelocitySetpoint,
        coordinate_frame: int = 1,  # MAV_FRAME_LOCAL_NED
    ) -> bool:
        """
        Send velocity setpoint.

        Args:
            setpoint: Velocity setpoint.
            coordinate_frame: MAVLink coordinate frame.

        Returns:
            True if message sent successfully.
        """
        if self._transport.mav is None:
            return False

        try:
            self._transport.mav.set_position_target_local_ned_send(
                time_boot_ms=int(time.time() * 1000) % (2**32),
                target_system=self._transport.target_system,
                target_component=self._transport.target_component,
                coordinate_frame=coordinate_frame,
                type_mask=self.TYPE_MASK_VEL,
                x=0.0,
                y=0.0,
                z=0.0,
                vx=setpoint.vx,
                vy=setpoint.vy,
                vz=setpoint.vz,
                afx=0.0,
                afy=0.0,
                afz=0.0,
                yaw=0.0,
                yaw_rate=setpoint.yaw_rate,
            )

            self._last_send_time = time.time()
            return True

        except Exception as e:
            logger.error("offboard_velocity_send_failed", error=str(e))
            return False

    def send_position_setpoint(
        self,
        setpoint: PositionSetpoint,
        coordinate_frame: int = 1,  # MAV_FRAME_LOCAL_NED
    ) -> bool:
        """
        Send position setpoint.

        Args:
            setpoint: Position setpoint.
            coordinate_frame: MAVLink coordinate frame.

        Returns:
            True if message sent successfully.
        """
        if self._transport.mav is None:
            return False

        try:
            self._transport.mav.set_position_target_local_ned_send(
                time_boot_ms=int(time.time() * 1000) % (2**32),
                target_system=self._transport.target_system,
                target_component=self._transport.target_component,
                coordinate_frame=coordinate_frame,
                type_mask=self.TYPE_MASK_POS,
                x=setpoint.x,
                y=setpoint.y,
                z=setpoint.z,
                vx=0.0,
                vy=0.0,
                vz=0.0,
                afx=0.0,
                afy=0.0,
                afz=0.0,
                yaw=setpoint.yaw,
                yaw_rate=0.0,
            )

            self._last_send_time = time.time()
            return True

        except Exception as e:
            logger.error("offboard_position_send_failed", error=str(e))
            return False

    def enable_offboard_mode(self) -> bool:
        """
        Request transition to offboard flight mode.

        Note: Requires continuous setpoint stream to maintain offboard.

        Returns:
            True if command sent successfully.
        """
        if self._transport.mav is None:
            return False

        try:
            # PX4: Custom mode for OFFBOARD is 6
            # ArduPilot: GUIDED mode is 4
            self._transport.mav.command_long_send(
                target_system=self._transport.target_system,
                target_component=self._transport.target_component,
                command=176,  # MAV_CMD_DO_SET_MODE
                confirmation=0,
                param1=1,  # MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
                param2=6,  # PX4 OFFBOARD mode
                param3=0,
                param4=0,
                param5=0,
                param6=0,
                param7=0,
            )

            self._offboard_enabled = True
            logger.info("offboard_mode_requested")
            return True

        except Exception as e:
            logger.error("offboard_mode_request_failed", error=str(e))
            return False

    def arm(self) -> bool:
        """
        Send arm command.

        Returns:
            True if command sent successfully.
        """
        if self._transport.mav is None:
            return False

        try:
            self._transport.mav.command_long_send(
                target_system=self._transport.target_system,
                target_component=self._transport.target_component,
                command=400,  # MAV_CMD_COMPONENT_ARM_DISARM
                confirmation=0,
                param1=1,  # Arm
                param2=0,
                param3=0,
                param4=0,
                param5=0,
                param6=0,
                param7=0,
            )

            logger.info("arm_command_sent")
            return True

        except Exception as e:
            logger.error("arm_command_failed", error=str(e))
            return False

    def disarm(self, force: bool = False) -> bool:
        """
        Send disarm command.

        Args:
            force: Force disarm even if in flight.

        Returns:
            True if command sent successfully.
        """
        if self._transport.mav is None:
            return False

        try:
            self._transport.mav.command_long_send(
                target_system=self._transport.target_system,
                target_component=self._transport.target_component,
                command=400,  # MAV_CMD_COMPONENT_ARM_DISARM
                confirmation=0,
                param1=0,  # Disarm
                param2=21196.0 if force else 0,  # Force disarm magic number
                param3=0,
                param4=0,
                param5=0,
                param6=0,
                param7=0,
            )

            logger.info("disarm_command_sent", force=force)
            return True

        except Exception as e:
            logger.error("disarm_command_failed", error=str(e))
            return False

    def land(self) -> bool:
        """
        Send land command.

        Returns:
            True if command sent successfully.
        """
        if self._transport.mav is None:
            return False

        try:
            self._transport.mav.command_long_send(
                target_system=self._transport.target_system,
                target_component=self._transport.target_component,
                command=21,  # MAV_CMD_NAV_LAND
                confirmation=0,
                param1=0,
                param2=0,
                param3=0,
                param4=0,
                param5=0,  # Latitude (0 = current)
                param6=0,  # Longitude (0 = current)
                param7=0,  # Altitude (0 = current)
            )

            logger.info("land_command_sent")
            return True

        except Exception as e:
            logger.error("land_command_failed", error=str(e))
            return False

    @property
    def is_offboard_enabled(self) -> bool:
        """Check if offboard mode was requested."""
        return self._offboard_enabled
