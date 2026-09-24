"""
MAVLink transport layer for Scandium.

Provides UDP and serial transport for MAVLink communication.
"""

from enum import Enum
from typing import Optional, Any
import time

from scandium.logging.setup import get_logger

logger = get_logger(__name__)


class TransportType(Enum):
    """Transport type enumeration."""

    UDP = "udp"
    SERIAL = "serial"


class MavlinkTransport:
    """
    MAVLink transport abstraction.

    Supports UDP and serial connections using pymavlink.
    """

    def __init__(
        self,
        transport: str = "udp",
        udp_address: str = "127.0.0.1",
        udp_port: int = 14550,
        serial_device: str = "/dev/ttyAMA0",
        serial_baud: int = 921600,
        system_id: int = 42,
        component_id: int = 200,
        target_system: int = 1,
        target_component: int = 1,
    ) -> None:
        """
        Initialize MAVLink transport.

        Args:
            transport: Transport type ('udp' or 'serial').
            udp_address: UDP address for connection.
            udp_port: UDP port.
            serial_device: Serial device path.
            serial_baud: Serial baud rate.
            system_id: This system's ID.
            component_id: This component's ID.
            target_system: Target system ID.
            target_component: Target component ID.
        """
        self._transport_type = TransportType(transport.lower())
        self._udp_address = udp_address
        self._udp_port = udp_port
        self._serial_device = serial_device
        self._serial_baud = serial_baud
        self._system_id = system_id
        self._component_id = component_id
        self._target_system = target_system
        self._target_component = target_component

        self._connection: Optional[Any] = None
        self._connected = False
        self._last_heartbeat_time: Optional[float] = None

    def connect(self) -> bool:
        """
        Establish MAVLink connection.

        Returns:
            True if connection successful.
        """
        try:
            from pymavlink import mavutil

            if self._transport_type == TransportType.UDP:
                # UDP connection string
                conn_str = f"udp:{self._udp_address}:{self._udp_port}"
                self._connection = mavutil.mavlink_connection(
                    conn_str,
                    source_system=self._system_id,
                    source_component=self._component_id,
                )
            else:
                # Serial connection
                self._connection = mavutil.mavlink_connection(
                    self._serial_device,
                    baud=self._serial_baud,
                    source_system=self._system_id,
                    source_component=self._component_id,
                )

            self._connected = True
            logger.info(
                "mavlink_connected",
                transport=self._transport_type.value,
            )
            return True

        except Exception as e:
            logger.error("mavlink_connection_failed", error=str(e))
            self._connected = False
            return False

    def wait_heartbeat(self, timeout_s: float = 10.0) -> bool:
        """
        Wait for heartbeat from autopilot.

        Args:
            timeout_s: Timeout in seconds.

        Returns:
            True if heartbeat received.
        """
        if self._connection is None:
            return False

        try:
            msg = self._connection.recv_match(
                type="HEARTBEAT",
                blocking=True,
                timeout=timeout_s,
            )

            if msg:
                self._last_heartbeat_time = time.time()
                logger.info(
                    "mavlink_heartbeat",
                    autopilot=msg.autopilot,
                    type=msg.type,
                )
                return True
            return False

        except Exception as e:
            logger.error("mavlink_heartbeat_failed", error=str(e))
            return False

    def send(self, message: Any) -> bool:
        """
        Send MAVLink message.

        Args:
            message: MAVLink message object.

        Returns:
            True if sent successfully.
        """
        if self._connection is None:
            return False

        try:
            self._connection.mav.send(message)
            return True
        except Exception as e:
            logger.error("mavlink_send_failed", error=str(e))
            return False

    def send_landing_target(
        self,
        timestamp_us: int,
        target_num: int,
        frame: int,
        angle_x: float,
        angle_y: float,
        distance: float,
        size_x: float = 0.0,
        size_y: float = 0.0,
        x: float = 0.0,
        y: float = 0.0,
        z: float = 0.0,
        q: Optional[list[float]] = None,
        position_valid: int = 0,
    ) -> bool:
        """
        Send LANDING_TARGET message.

        Args:
            timestamp_us: Timestamp in microseconds.
            target_num: Target number.
            frame: MAV_FRAME enum value.
            angle_x: X-axis angular offset (rad).
            angle_y: Y-axis angular offset (rad).
            distance: Distance to target (m).
            size_x: Target size in X (rad).
            size_y: Target size in Y (rad).
            x: X position (m).
            y: Y position (m).
            z: Z position (m).
            q: Quaternion of landing target orientation [w, x, y, z].
            position_valid: 1 if x, y, z are valid.

        Returns:
            True if sent successfully.
        """
        if self._connection is None:
            return False

        try:
            if q is None:
                q = [1.0, 0.0, 0.0, 0.0]

            self._connection.mav.landing_target_send(
                time_usec=timestamp_us,
                target_num=target_num,
                frame=frame,
                angle_x=angle_x,
                angle_y=angle_y,
                distance=distance,
                size_x=size_x,
                size_y=size_y,
                x=x,
                y=y,
                z=z,
                q=q,
                type=0,  # LANDING_TARGET_TYPE_LIGHT_BEACON or similar
                position_valid=position_valid,
            )
            return True

        except Exception as e:
            logger.error("mavlink_landing_target_failed", error=str(e))
            return False

    def recv(self, blocking: bool = False, timeout_s: float = 0.1) -> Optional[Any]:
        """
        Receive MAVLink message.

        Args:
            blocking: Whether to block for message.
            timeout_s: Timeout for blocking receive.

        Returns:
            MAVLink message or None.
        """
        if self._connection is None:
            return None

        try:
            return self._connection.recv_match(
                blocking=blocking,
                timeout=timeout_s,
            )
        except Exception:
            return None

    def close(self) -> None:
        """Close MAVLink connection."""
        if self._connection is not None:
            try:
                self._connection.close()
            except Exception:
                pass
            self._connection = None
            self._connected = False
            logger.info("mavlink_closed")

    @property
    def is_connected(self) -> bool:
        """Check if connected."""
        return self._connected and self._connection is not None

    @property
    def mav(self) -> Optional[Any]:
        """Get MAVLink interface for direct message construction."""
        if self._connection is not None:
            return self._connection.mav
        return None

    @property
    def target_system(self) -> int:
        """Get target system ID."""
        return self._target_system

    @property
    def target_component(self) -> int:
        """Get target component ID."""
        return self._target_component
