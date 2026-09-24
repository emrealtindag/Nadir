"""
Coordinate frame transformations for Scandium.

Provides functions for converting between camera, body, and MAVLink frames.
"""

from dataclasses import dataclass
from typing import Tuple

import numpy as np
from numpy.typing import NDArray

from scandium.perception.calib import CameraExtrinsics


@dataclass
class Transform3D:
    """
    3D rigid transformation (rotation + translation).

    Attributes:
        R: 3x3 rotation matrix.
        t: 3-element translation vector.
    """

    R: NDArray[np.float64]
    t: NDArray[np.float64]

    def apply(self, point: NDArray[np.float64]) -> NDArray[np.float64]:
        """Apply transform to a point: p_out = R @ p_in + t."""
        return self.R @ point + self.t

    def inverse(self) -> "Transform3D":
        """Return inverse transform."""
        R_inv = self.R.T
        t_inv = -R_inv @ self.t
        return Transform3D(R=R_inv, t=t_inv)

    def compose(self, other: "Transform3D") -> "Transform3D":
        """Compose with another transform: T_out = self * other."""
        R_new = self.R @ other.R
        t_new = self.R @ other.t + self.t
        return Transform3D(R=R_new, t=t_new)


@dataclass
class Pose3D:
    """
    3D pose (position + orientation).

    Attributes:
        position_m: Position in meters (x, y, z).
        quat_wxyz: Quaternion orientation (w, x, y, z).
    """

    position_m: NDArray[np.float64]
    quat_wxyz: NDArray[np.float64]

    @property
    def x(self) -> float:
        """X position."""
        return float(self.position_m[0])

    @property
    def y(self) -> float:
        """Y position."""
        return float(self.position_m[1])

    @property
    def z(self) -> float:
        """Z position."""
        return float(self.position_m[2])


def cam_to_body(
    p_cam: NDArray[np.float64],
    extrinsics: CameraExtrinsics,
) -> NDArray[np.float64]:
    """
    Transform point from camera frame to body frame.

    Args:
        p_cam: Point in camera frame (x, y, z).
        extrinsics: Camera-to-body extrinsic calibration.

    Returns:
        Point in body frame (x_body, y_body, z_body).
    """
    return extrinsics.transform_point(p_cam)


def body_to_mavlink_fields(
    p_body: NDArray[np.float64],
    frame: str = "MAV_FRAME_BODY_NED",
) -> Tuple[float, float, float, float, float, bool]:
    """
    Convert body-frame position to MAVLink LANDING_TARGET fields.

    Args:
        p_body: Position in body frame (x, y, z) in meters.
            - x: forward (positive forward)
            - y: right (positive right)
            - z: down (positive down)
        frame: MAVLink frame identifier.

    Returns:
        Tuple of (x, y, z, angle_x, angle_y, position_valid).

        For MAV_FRAME_BODY_NED:
        - x, y, z: Position in meters (body NED frame).
        - angle_x: Angle to target in body X-Z plane (radians).
        - angle_y: Angle to target in body Y-Z plane (radians).
        - position_valid: True if x,y,z are valid.
    """
    x = float(p_body[0])
    y = float(p_body[1])
    z = float(p_body[2])

    # Compute angles
    # angle_x: rotation about Y axis (forward direction)
    # angle_y: rotation about X axis (right direction)
    distance_xz = np.sqrt(x**2 + z**2)
    distance_yz = np.sqrt(y**2 + z**2)

    if distance_xz > 1e-6:
        angle_x = np.arctan2(x, z)  # Angle in X-Z plane
    else:
        angle_x = 0.0

    if distance_yz > 1e-6:
        angle_y = np.arctan2(y, z)  # Angle in Y-Z plane
    else:
        angle_y = 0.0

    position_valid = True

    return x, y, z, float(angle_x), float(angle_y), position_valid


def compute_angles_from_camera(
    tvec: NDArray[np.float64],
) -> Tuple[float, float]:
    """
    Compute angles directly from camera-frame translation vector.

    For downward-facing camera:
    - Camera X is right
    - Camera Y is down
    - Camera Z is forward (viewing direction)

    Args:
        tvec: Translation vector from solvePnP (target in camera frame).

    Returns:
        Tuple of (angle_x, angle_y) in radians.
    """
    x, y, z = tvec[0], tvec[1], tvec[2]

    # angle_x: horizontal angle (left-right)
    angle_x = np.arctan2(x, z)

    # angle_y: vertical angle (up-down)
    angle_y = np.arctan2(y, z)

    return float(angle_x), float(angle_y)


def ned_to_body_ned(
    p_ned: NDArray[np.float64],
    yaw_rad: float,
) -> NDArray[np.float64]:
    """
    Transform from local NED to body NED frame.

    Args:
        p_ned: Position in local NED (North, East, Down).
        yaw_rad: Vehicle yaw angle (rotation about Down axis).

    Returns:
        Position in body NED (Forward, Right, Down).
    """
    cos_yaw = np.cos(yaw_rad)
    sin_yaw = np.sin(yaw_rad)

    # Rotation matrix from NED to body
    R = np.array(
        [
            [cos_yaw, sin_yaw, 0],
            [-sin_yaw, cos_yaw, 0],
            [0, 0, 1],
        ]
    )

    return R @ p_ned
