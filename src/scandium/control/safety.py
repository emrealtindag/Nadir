"""
Safety supervisor for Scandium.

Monitors system health and enforces safety constraints.
"""

from dataclasses import dataclass
from typing import Optional
import time

from scandium.logging.setup import get_logger

logger = get_logger(__name__)


@dataclass
class SafetyLimits:
    """
    Safety limits configuration.

    Attributes:
        max_lateral_speed_mps: Maximum lateral speed in m/s.
        max_descent_speed_mps: Maximum descent speed in m/s.
        min_safe_altitude_m: Minimum safe altitude for abort.
        max_jerk_mps3: Maximum jerk (rate of acceleration change).
        perception_timeout_s: Timeout for perception loop.
        mavlink_timeout_s: Timeout for MAVLink heartbeat.
    """

    max_lateral_speed_mps: float = 1.5
    max_descent_speed_mps: float = 0.7
    min_safe_altitude_m: float = 2.0
    max_jerk_mps3: float = 5.0
    perception_timeout_s: float = 1.0
    mavlink_timeout_s: float = 3.0


@dataclass
class SafetyStatus:
    """
    Current safety status.

    Attributes:
        is_safe: Overall safety status.
        violations: List of active violations.
        last_perception_time: Timestamp of last perception update.
        last_mavlink_time: Timestamp of last MAVLink heartbeat.
    """

    is_safe: bool = True
    violations: list[str] = None  # type: ignore
    last_perception_time: float = 0.0
    last_mavlink_time: float = 0.0

    def __post_init__(self) -> None:
        if self.violations is None:
            self.violations = []


class SafetySupervisor:
    """
    Safety supervisor monitors system health and enforces constraints.

    Implements watchdog functionality and limit enforcement.
    """

    def __init__(self, limits: Optional[SafetyLimits] = None) -> None:
        """
        Initialize safety supervisor.

        Args:
            limits: Safety limits configuration.
        """
        self._limits = limits or SafetyLimits()
        self._last_perception_time = time.time()
        self._last_mavlink_time = time.time()
        self._violations: list[str] = []

    def check(
        self,
        perception_active: bool = True,
        mavlink_active: bool = True,
        lateral_speed: float = 0.0,
        descent_speed: float = 0.0,
        altitude: float = 100.0,
        landability_score: float = 1.0,
        human_present: bool = False,
    ) -> SafetyStatus:
        """
        Perform safety check.

        Args:
            perception_active: Whether perception loop is active.
            mavlink_active: Whether MAVLink connection is active.
            lateral_speed: Current lateral speed in m/s.
            descent_speed: Current descent speed in m/s.
            altitude: Current altitude in m.
            landability_score: Current landability score.
            human_present: Whether human is detected.

        Returns:
            SafetyStatus with check results.
        """
        self._violations.clear()
        now = time.time()

        # Update timestamps
        if perception_active:
            self._last_perception_time = now
        if mavlink_active:
            self._last_mavlink_time = now

        # Check perception timeout
        if now - self._last_perception_time > self._limits.perception_timeout_s:
            self._violations.append("perception_timeout")
            logger.warning("safety_violation", violation="perception_timeout")

        # Check MAVLink timeout
        if now - self._last_mavlink_time > self._limits.mavlink_timeout_s:
            self._violations.append("mavlink_timeout")
            logger.warning("safety_violation", violation="mavlink_timeout")

        # Check speed limits
        if lateral_speed > self._limits.max_lateral_speed_mps:
            self._violations.append("lateral_speed_exceeded")
            logger.warning(
                "safety_violation",
                violation="lateral_speed_exceeded",
                speed=lateral_speed,
                limit=self._limits.max_lateral_speed_mps,
            )

        if descent_speed > self._limits.max_descent_speed_mps:
            self._violations.append("descent_speed_exceeded")
            logger.warning(
                "safety_violation",
                violation="descent_speed_exceeded",
                speed=descent_speed,
                limit=self._limits.max_descent_speed_mps,
            )

        # Check human presence
        if human_present:
            self._violations.append("human_detected")
            logger.warning("safety_violation", violation="human_detected")

        # Check landability
        if landability_score < 0.2:
            self._violations.append("critical_landability")
            logger.warning(
                "safety_violation",
                violation="critical_landability",
                score=landability_score,
            )

        is_safe = len(self._violations) == 0

        return SafetyStatus(
            is_safe=is_safe,
            violations=self._violations.copy(),
            last_perception_time=self._last_perception_time,
            last_mavlink_time=self._last_mavlink_time,
        )

    def clamp_velocity(
        self,
        vx: float,
        vy: float,
        vz: float,
        confidence: float = 1.0,
    ) -> tuple[float, float, float]:
        """
        Clamp velocity commands to safe limits.

        Args:
            vx: X velocity (forward).
            vy: Y velocity (right).
            vz: Z velocity (down).
            confidence: Confidence factor [0, 1] to scale limits.

        Returns:
            Tuple of clamped (vx, vy, vz).
        """
        # Scale limits by confidence
        lat_limit = self._limits.max_lateral_speed_mps * confidence
        desc_limit = self._limits.max_descent_speed_mps * confidence

        # Clamp lateral
        lateral_speed = (vx**2 + vy**2) ** 0.5
        if lateral_speed > lat_limit:
            scale = lat_limit / lateral_speed
            vx *= scale
            vy *= scale

        # Clamp descent (positive = down)
        if vz > desc_limit:
            vz = desc_limit

        return vx, vy, vz

    def should_abort(self) -> bool:
        """Check if abort is required based on current violations."""
        critical = {"perception_timeout", "mavlink_timeout", "human_detected"}
        return bool(set(self._violations) & critical)

    @property
    def limits(self) -> SafetyLimits:
        """Get current safety limits."""
        return self._limits

    def reset(self) -> None:
        """Reset supervisor state."""
        self._last_perception_time = time.time()
        self._last_mavlink_time = time.time()
        self._violations.clear()
