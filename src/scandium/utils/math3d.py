"""
3D math utilities for Scandium.

Provides rotation conversions, transformation utilities, and geometric functions.
"""

import numpy as np
from numpy.typing import NDArray


def rotation_matrix_to_euler(R: NDArray[np.float64]) -> tuple[float, float, float]:
    """
    Convert rotation matrix to Euler angles (roll, pitch, yaw).

    Uses ZYX convention (yaw-pitch-roll).

    Args:
        R: 3x3 rotation matrix.

    Returns:
        Tuple of (roll, pitch, yaw) in radians.
    """
    sy = np.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)

    singular = sy < 1e-6

    if not singular:
        roll = np.arctan2(R[2, 1], R[2, 2])
        pitch = np.arctan2(-R[2, 0], sy)
        yaw = np.arctan2(R[1, 0], R[0, 0])
    else:
        roll = np.arctan2(-R[1, 2], R[1, 1])
        pitch = np.arctan2(-R[2, 0], sy)
        yaw = 0.0

    return float(roll), float(pitch), float(yaw)


def euler_to_rotation_matrix(
    roll: float, pitch: float, yaw: float
) -> NDArray[np.float64]:
    """
    Convert Euler angles to rotation matrix.

    Uses ZYX convention (yaw-pitch-roll).

    Args:
        roll: Roll angle in radians.
        pitch: Pitch angle in radians.
        yaw: Yaw angle in radians.

    Returns:
        3x3 rotation matrix.
    """
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)

    R = np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ],
        dtype=np.float64,
    )

    return R


def quaternion_to_rotation_matrix(q: NDArray[np.float64]) -> NDArray[np.float64]:
    """
    Convert quaternion to rotation matrix.

    Args:
        q: Quaternion [w, x, y, z].

    Returns:
        3x3 rotation matrix.
    """
    w, x, y, z = q[0], q[1], q[2], q[3]

    R = np.array(
        [
            [1 - 2 * (y**2 + z**2), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x**2 + z**2), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x**2 + y**2)],
        ],
        dtype=np.float64,
    )

    return R


def rotation_matrix_to_quaternion(R: NDArray[np.float64]) -> NDArray[np.float64]:
    """
    Convert rotation matrix to quaternion.

    Args:
        R: 3x3 rotation matrix.

    Returns:
        Quaternion [w, x, y, z].
    """
    trace = np.trace(R)

    if trace > 0:
        s = 0.5 / np.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (R[2, 1] - R[1, 2]) * s
        y = (R[0, 2] - R[2, 0]) * s
        z = (R[1, 0] - R[0, 1]) * s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s

    return np.array([w, x, y, z], dtype=np.float64)


def rodrigues_to_rotation_matrix(rvec: NDArray[np.float64]) -> NDArray[np.float64]:
    """
    Convert Rodrigues vector to rotation matrix.

    Args:
        rvec: 3-element Rodrigues vector.

    Returns:
        3x3 rotation matrix.
    """
    import cv2

    R, _ = cv2.Rodrigues(rvec.reshape(3, 1))
    return R  # type: ignore[return-value]


def rotation_matrix_to_rodrigues(R: NDArray[np.float64]) -> NDArray[np.float64]:
    """
    Convert rotation matrix to Rodrigues vector.

    Args:
        R: 3x3 rotation matrix.

    Returns:
        3-element Rodrigues vector.
    """
    import cv2

    rvec, _ = cv2.Rodrigues(R)
    return rvec.flatten()


def transform_point(
    point: NDArray[np.float64],
    R: NDArray[np.float64],
    t: NDArray[np.float64],
) -> NDArray[np.float64]:
    """
    Apply rigid transform to a point.

    p_out = R @ p_in + t

    Args:
        point: 3D point.
        R: 3x3 rotation matrix.
        t: 3-element translation vector.

    Returns:
        Transformed 3D point.
    """
    return R @ point + t


def normalize_vector(v: NDArray[np.float64]) -> NDArray[np.float64]:
    """
    Normalize a vector to unit length.

    Args:
        v: Input vector.

    Returns:
        Unit vector in same direction.
    """
    norm = np.linalg.norm(v)
    if norm < 1e-10:
        return v
    return v / norm
