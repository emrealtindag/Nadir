"""
PnP-based pose estimation for Scandium.

Provides functions for estimating marker pose from 2D-3D correspondences.
"""

from dataclasses import dataclass
from typing import Optional

import cv2
import numpy as np
from numpy.typing import NDArray

from scandium.perception.calib import CameraIntrinsics


@dataclass
class PoseEstimate:
    """
    Pose estimation result from solvePnP.

    Attributes:
        rvec: Rodrigues rotation vector (3,).
        tvec: Translation vector (3,) in camera frame.
        reproj_error_px: Reprojection error in pixels.
        success: Whether estimation succeeded.
    """

    rvec: NDArray[np.float64]
    tvec: NDArray[np.float64]
    reproj_error_px: float
    success: bool

    @property
    def distance_m(self) -> float:
        """Distance from camera to marker center in meters."""
        return float(np.linalg.norm(self.tvec))

    def get_rotation_matrix(self) -> NDArray[np.float64]:
        """Convert rvec to 3x3 rotation matrix."""
        R, _ = cv2.Rodrigues(self.rvec.reshape(3, 1))
        return R


def estimate_pose_from_corners(
    corners_px: NDArray[np.float64],
    marker_size_m: float,
    intrinsics: CameraIntrinsics,
    method: int = cv2.SOLVEPNP_IPPE_SQUARE,
) -> PoseEstimate:
    """
    Estimate marker pose from corner detections.

    Args:
        corners_px: 2D corner coordinates in pixels, shape (4, 2).
        marker_size_m: Physical marker size in meters.
        intrinsics: Camera intrinsic parameters.
        method: solvePnP method (default: IPPE_SQUARE for square markers).

    Returns:
        PoseEstimate with rotation, translation, and error.
    """
    # Define 3D object points for square marker
    # Marker centered at origin, lying in Z=0 plane
    half_size = marker_size_m / 2.0
    object_points = np.array(
        [
            [-half_size, -half_size, 0],  # Top-left (when viewed from front)
            [half_size, -half_size, 0],  # Top-right
            [half_size, half_size, 0],  # Bottom-right
            [-half_size, half_size, 0],  # Bottom-left
        ],
        dtype=np.float64,
    )

    # Ensure correct shapes
    image_points = corners_px.reshape(-1, 1, 2).astype(np.float64)
    object_points = object_points.reshape(-1, 1, 3)

    # Solve PnP
    success, rvec, tvec = cv2.solvePnP(
        object_points,
        image_points,
        intrinsics.K,
        intrinsics.dist_coeffs,
        flags=method,
    )

    if not success:
        return PoseEstimate(
            rvec=np.zeros(3, dtype=np.float64),
            tvec=np.zeros(3, dtype=np.float64),
            reproj_error_px=float("inf"),
            success=False,
        )

    rvec = rvec.flatten()
    tvec = tvec.flatten()

    # Calculate reprojection error
    reproj_error = compute_reprojection_error(
        object_points.reshape(-1, 3),
        corners_px,
        rvec,
        tvec,
        intrinsics,
    )

    return PoseEstimate(
        rvec=rvec,
        tvec=tvec,
        reproj_error_px=reproj_error,
        success=True,
    )


def compute_reprojection_error(
    object_points: NDArray[np.float64],
    image_points: NDArray[np.float64],
    rvec: NDArray[np.float64],
    tvec: NDArray[np.float64],
    intrinsics: CameraIntrinsics,
) -> float:
    """
    Compute mean reprojection error.

    Args:
        object_points: 3D points, shape (N, 3).
        image_points: Observed 2D points, shape (N, 2).
        rvec: Rotation vector.
        tvec: Translation vector.
        intrinsics: Camera intrinsics.

    Returns:
        Mean reprojection error in pixels.
    """
    projected, _ = cv2.projectPoints(
        object_points,
        rvec,
        tvec,
        intrinsics.K,
        intrinsics.dist_coeffs,
    )
    projected = projected.reshape(-1, 2)

    error = np.linalg.norm(image_points - projected, axis=1)
    return float(np.mean(error))


def refine_pose(
    pose: PoseEstimate,
    corners_px: NDArray[np.float64],
    marker_size_m: float,
    intrinsics: CameraIntrinsics,
    iterations: int = 100,
) -> PoseEstimate:
    """
    Refine pose estimate using iterative optimization.

    Args:
        pose: Initial pose estimate.
        corners_px: 2D corner coordinates.
        marker_size_m: Physical marker size.
        intrinsics: Camera intrinsics.
        iterations: Maximum iterations.

    Returns:
        Refined PoseEstimate.
    """
    if not pose.success:
        return pose

    # Define object points
    half_size = marker_size_m / 2.0
    object_points = np.array(
        [
            [-half_size, -half_size, 0],
            [half_size, -half_size, 0],
            [half_size, half_size, 0],
            [-half_size, half_size, 0],
        ],
        dtype=np.float64,
    )

    # Use LM refinement
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, iterations, 1e-6)

    rvec = pose.rvec.reshape(3, 1)
    tvec = pose.tvec.reshape(3, 1)

    rvec_refined, tvec_refined = cv2.solvePnPRefineLM(
        object_points.reshape(-1, 1, 3),
        corners_px.reshape(-1, 1, 2),
        intrinsics.K,
        intrinsics.dist_coeffs,
        rvec,
        tvec,
        criteria,
    )

    rvec_refined = rvec_refined.flatten()
    tvec_refined = tvec_refined.flatten()

    reproj_error = compute_reprojection_error(
        object_points,
        corners_px,
        rvec_refined,
        tvec_refined,
        intrinsics,
    )

    return PoseEstimate(
        rvec=rvec_refined,
        tvec=tvec_refined,
        reproj_error_px=reproj_error,
        success=True,
    )
