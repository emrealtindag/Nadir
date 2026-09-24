"""
Heartbeat monitoring for Scandium.

Provides connection health monitoring via MAVLink heartbeat.
"""

from typing import Optional
import time
import threading

from scandium.mavlink.transport import MavlinkTransport
from scandium.logging.setup import get_logger

logger = get_logger(__name__)


class HeartbeatMonitor:
    """
    Monitors MAVLink heartbeat for connection health.

    Runs in a background thread to continuously check connection.
    """

    def __init__(
        self,
        transport: MavlinkTransport,
        timeout_s: float = 3.0,
        send_rate_hz: float = 1.0,
    ) -> None:
        """
        Initialize heartbeat monitor.

        Args:
            transport: MAVLink transport instance.
            timeout_s: Timeout for considering connection lost.
            send_rate_hz: Rate at which to send our heartbeat.
        """
        self._transport = transport
        self._timeout_s = timeout_s
        self._send_interval_s = 1.0 / send_rate_hz

        self._last_recv_time: Optional[float] = None
        self._last_send_time: float = 0.0
        self._connected = False
        self._running = False
        self._thread: Optional[threading.Thread] = None

        # Received heartbeat info
        self._autopilot_type: Optional[int] = None
        self._vehicle_type: Optional[int] = None
        self._system_status: Optional[int] = None

    def start(self) -> None:
        """Start heartbeat monitoring thread."""
        if self._running:
            return

        self._running = True
        self._thread = threading.Thread(target=self._monitor_loop, daemon=True)
        self._thread.start()
        logger.info("heartbeat_monitor_started")

    def stop(self) -> None:
        """Stop heartbeat monitoring."""
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
        logger.info("heartbeat_monitor_stopped")

    def _monitor_loop(self) -> None:
        """Background monitoring loop."""
        while self._running:
            try:
                self._check_heartbeat()
                self._send_heartbeat()
                time.sleep(0.1)
            except Exception as e:
                logger.error("heartbeat_monitor_error", error=str(e))

    def _check_heartbeat(self) -> None:
        """Check for received heartbeat."""
        msg = self._transport.recv(blocking=False)

        if msg is not None and msg.get_type() == "HEARTBEAT":
            self._last_recv_time = time.time()
            self._autopilot_type = msg.autopilot
            self._vehicle_type = msg.type
            self._system_status = msg.system_status

            if not self._connected:
                self._connected = True
                logger.info(
                    "autopilot_connected",
                    autopilot=msg.autopilot,
                    type=msg.type,
                )

        # Check timeout
        if self._last_recv_time is not None:
            if time.time() - self._last_recv_time > self._timeout_s:
                if self._connected:
                    self._connected = False
                    logger.warning("autopilot_disconnected")

    def _send_heartbeat(self) -> None:
        """Send our heartbeat."""
        now = time.time()
        if now - self._last_send_time >= self._send_interval_s:
            self._send_companion_heartbeat()
            self._last_send_time = now

    def _send_companion_heartbeat(self) -> None:
        """Send companion computer heartbeat."""
        if self._transport.mav is None:
            return

        try:
            from pymavlink import mavutil

            self._transport.mav.heartbeat_send(
                type=mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                autopilot=mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                base_mode=0,
                custom_mode=0,
                system_status=mavutil.mavlink.MAV_STATE_ACTIVE,
            )
        except Exception as e:
            logger.debug("heartbeat_send_failed", error=str(e))

    @property
    def is_connected(self) -> bool:
        """Check if autopilot is connected."""
        return self._connected

    @property
    def last_heartbeat_age_s(self) -> float:
        """Get age of last received heartbeat in seconds."""
        if self._last_recv_time is None:
            return float("inf")
        return time.time() - self._last_recv_time

    @property
    def autopilot_info(self) -> dict:
        """Get autopilot information from heartbeat."""
        return {
            "connected": self._connected,
            "autopilot_type": self._autopilot_type,
            "vehicle_type": self._vehicle_type,
            "system_status": self._system_status,
            "last_heartbeat_age_s": self.last_heartbeat_age_s,
        }
