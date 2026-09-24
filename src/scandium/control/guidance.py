"""
Guidance module for Scandium.

Provides setpoint generation for landing approach.
"""

from dataclasses import dataclass
from typing import Optional

import numpy as np


@dataclass
class GuidanceSetpoint:
    """
    Guidance setpoint for landing approach.

    Attributes:
        vx: X velocity setpoint (forward, m/s).
        vy: Y velocity setpoint (right, m/s).
        vz: Z velocity setpoint (down, m/s).
        yaw_rate: Yaw rate setpoint (rad/s).
    """

    vx: float = 0.0
    vy: float = 0.0
    vz: float = 0.0
    yaw_rate: float = 0.0


class GuidanceController:
    """
    Simple proportional guidance controller for landing approach.
    """

    def __init__(
        self,
        lateral_gain: float = 0.5,
        descent_rate: float = 0.3,
        alignment_threshold_m: float = 0.1,
    ) -> None:
        """
        Initialize guidance controller.

        Args:
            lateral_gain: Proportional gain for lateral correction.
            descent_rate: Base descent rate in m/s.
            alignment_threshold_m: Threshold for alignment (pause descent).
        """
        self._lateral_gain = lateral_gain
        self._descent_rate = descent_rate
        self._alignment_threshold = alignment_threshold_m

    def compute_setpoint(
        self,
        target_x: float,
        target_y: float,
        target_z: float,
        current_altitude: float,
        confidence: float = 1.0,
    ) -> GuidanceSetpoint:
        """
        Compute velocity setpoint to approach target.

        Args:
            target_x: Target X offset (forward, m). Positive = target ahead.
            target_y: Target Y offset (right, m). Positive = target right.
            target_z: Target Z offset (down, m). Distance to target.
            current_altitude: Current altitude AGL (m).
            confidence: Detection confidence for gain scaling.

        Returns:
            GuidanceSetpoint with velocity commands.
        """
        # Scale gains by confidence
        gain = self._lateral_gain * confidence

        # Lateral corrections (move toward target)
        vx = -gain * target_x  # Negative because offset is from us to target
        vy = -gain * target_y

        # Descent control
        lateral_error = np.sqrt(target_x**2 + target_y**2)

        if lateral_error > self._alignment_threshold:
            # Reduce or pause descent when not aligned
            vz = self._descent_rate * 0.3
        else:
            # Normal descent when aligned
            vz = self._descent_rate * confidence

        return GuidanceSetpoint(vx=vx, vy=vy, vz=vz, yaw_rate=0.0)

    def compute_search_pattern(
        self,
        time_s: float,
        radius_m: float = 2.0,
        period_s: float = 10.0,
    ) -> GuidanceSetpoint:
        """
        Compute setpoint for search pattern.

        Args:
            time_s: Time since search started.
            radius_m: Search pattern radius.
            period_s: Time for one complete pattern.

        Returns:
            GuidanceSetpoint for expanding spiral search.
        """
        # Expanding spiral
        angle = 2 * np.pi * time_s / period_s
        expansion = min(1.0, time_s / (period_s * 2))
        r = radius_m * expansion

        vx = r * np.cos(angle) * (2 * np.pi / period_s)
        vy = r * np.sin(angle) * (2 * np.pi / period_s)

        return GuidanceSetpoint(vx=vx, vy=vy, vz=0.0, yaw_rate=0.0)
