"""
Pose estimator for Nadir.

Provides a small PoseEstimator that runs OpenCV's solvePnP on a detected
fiducial marker (FiducialDetection.corners_px) using a CameraIntrinsics
instance from calib.py. Returns rvec/tvec in camera coordinates.

The implementation follows the project's dataclass / typing style.
"""
from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np
from numpy.typing import NDArray

from nadir.perception.fiducials.base import FiducialDetection
from nadir.perception.calib import CameraIntrinsics


@dataclass
class PoseResult:
    """Result of a pose estimation.

    Attributes:
        rvec: Rotation vector (Rodrigues) as shape (3,) numpy array in camera frame.
        tvec: Translation vector as shape (3,) numpy array (meters) in camera frame.
        success: Whether solvePnP reported success.
        reproj_error: Mean reprojection error in pixels (float). Lower is better.
    """

    rvec: Optional[NDArray[np.float64]]
    tvec: Optional[NDArray[np.float64]]
    success: bool
    reproj_error: Optional[float] = None


class PoseEstimator:
    """Estimate fiducial pose using OpenCV solvePnP.

    Usage:
        pe = PoseEstimator(marker_size_m=0.2)
        res = pe.estimate(detection, intrinsics)

    Notes:
    - FiducialDetection.corners_px is expected in order TL, TR, BR, BL (4x2).
    - The marker is assumed to lie on the Z=0 plane in its own coordinate frame.
    - Returned tvec is in meters in the camera coordinate system.
    """

    def __init__(self, marker_size_m: float = 0.2, solve_method: int = 0, dt: float = 0.033) -> None:
        """Create a PoseEstimator.

        Args:
            marker_size_m: Physical marker side length in meters.
            solve_method: OpenCV solvePnP flags (e.g. cv2.SOLVEPNP_ITERATIVE).
                          If 0, the implementation will choose cv2.SOLVEPNP_ITERATIVE
                          when cv2 is available.
            dt: Time step for the Extended Kalman Filter (Sensor Fusion).
        """
        self.marker_size_m = float(marker_size_m)
        self._solve_method = solve_method
        self._ekf = None
        self._dt = dt
        self._initialized_ekf = False

        # Attempt to load the C++ Sensor Fusion module
        try:
            import nadir_ekf
            # Process noise and measurement noise tuned for UAV dynamics
            self._ekf = nadir_ekf.ExtendedKalmanFilter(self._dt, 0.5, 0.05)
            # We also import logging here to show success if needed
            import logging
            logging.getLogger(__name__).info("Successfully loaded C++ ExtendedKalmanFilter for Sensor Fusion.")
        except ImportError:
            import logging
            logging.getLogger(__name__).warning("C++ nadir_ekf module not found. Falling back to raw solvePnP. Run scripts/build_cpp.sh")

    def _object_points(self) -> NDArray[np.float64]:
        """Return 3D object points of marker corners in marker frame.

        Order corresponds to FiducialDetection.corners_px order: TL, TR, BR, BL.
        Marker lies on Z=0 plane, centered at origin.
        """
        s = self.marker_size_m / 2.0
        # define (x, y, z) for TL, TR, BR, BL
        # coordinate convention: +X right, +Y down, +Z forward (camera)
        # but marker frame here: centered on marker plane, Z=0
        obj = np.array(
            [
                [-s, s, 0.0],  # TL
                [s, s, 0.0],  # TR
                [s, -s, 0.0],  # BR
                [-s, -s, 0.0],  # BL
            ],
            dtype=np.float64,
        )
        return obj

    def estimate(
        self,
        detection: FiducialDetection,
        intrinsics: CameraIntrinsics,
        refine: bool = True,
    ) -> PoseResult:
        """Estimate pose from a single fiducial detection.

        Args:
            detection: FiducialDetection including corners_px (4x2)
            intrinsics: CameraIntrinsics with K and dist_coeffs
            refine: if True and cv2.solvePnPRefineLM available, refine the solution

        Returns:
            PoseResult with rvec, tvec (camera frame), success flag and reprojection error.
        """
        import cv2

        if not detection.is_valid:
            return PoseResult(rvec=None, tvec=None, success=False, reproj_error=None)

        # Prepare object and image points
        objp = self._object_points()

        # image points must be float32 or float64 Nx2
        imgp = np.asarray(detection.corners_px, dtype=np.float64).reshape(-1, 2)

        # camera matrix and distortion
        K = np.asarray(intrinsics.K, dtype=np.float64)
        dist = np.asarray(intrinsics.dist_coeffs, dtype=np.float64)

        # choose solve method flag
        solve_flag = self._solve_method
        if solve_flag == 0:
            # default to ITERATIVE if available
            solve_flag = getattr(cv2, "SOLVEPNP_ITERATIVE", 0)

        # OpenCV expects objectPoints shape (N,3) and imagePoints (N,2)
        try:
            success, rvec, tvec = cv2.solvePnP(
                objp,
                imgp,
                K,
                dist,
                flags=solve_flag,
            )
        except TypeError:
            # older OpenCV signature returns retval, rvec, tvec differently
            retval = cv2.solvePnP(objp, imgp, K, dist, flags=solve_flag)
            # solvePnP can return (retval, rvec, tvec) or (rvec, tvec)
            if isinstance(retval, tuple) and len(retval) == 3:
                success, rvec, tvec = retval
            else:
                # unexpected
                return PoseResult(rvec=None, tvec=None, success=False, reproj_error=None)

        if not success:
            return PoseResult(rvec=None, tvec=None, success=False, reproj_error=None)

        # rvec/tvec to consistent numpy arrays (shape (3,)) and float64
        rvec = np.asarray(rvec, dtype=np.float64).reshape(3)
        tvec = np.asarray(tvec, dtype=np.float64).reshape(3)

        # optional refinement using Levenberg-Marquardt
        if refine:
            # try to refine solution if function available
            try:
                if hasattr(cv2, "solvePnPRefineLM"):
                    # prepare points in required shapes
                    cv2.solvePnPRefineLM(objp, imgp, K, dist, rvec, tvec)
                    rvec = np.asarray(rvec, dtype=np.float64).reshape(3)
                    tvec = np.asarray(tvec, dtype=np.float64).reshape(3)
            except Exception:
                # ignore refinement failures
                pass

        # reprojection error (mean euclidean distance in pixels)
        try:
            projected, _ = cv2.projectPoints(objp, rvec, tvec, K, dist)
            projected = projected.reshape(-1, 2)
            diffs = projected - imgp
            dists = np.linalg.norm(diffs, axis=1)
            reproj_err = float(np.mean(dists))
        except Exception:
            reproj_err = None

        # Apply C++ Extended Kalman Filter if available
        if self._ekf is not None:
            if not self._initialized_ekf:
                # Initialize state with first measurement [x, y, z, 0, 0, 0]
                self._ekf.reset([float(tvec[0]), float(tvec[1]), float(tvec[2]), 0.0, 0.0, 0.0])
                self._initialized_ekf = True
            else:
                self._ekf.predict()
                self._ekf.update([float(tvec[0]), float(tvec[1]), float(tvec[2])])
                
                # Retrieve filtered position
                filtered_pos = self._ekf.get_position()
                tvec = np.array([filtered_pos[0], filtered_pos[1], filtered_pos[2]], dtype=np.float64)

        return PoseResult(rvec=rvec, tvec=tvec, success=True, reproj_error=reproj_err)
