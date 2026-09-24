"""
Camera calibration utilities for Scandium.

Provides loading and management of camera intrinsics and extrinsics.
"""

from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import numpy as np
from numpy.typing import NDArray

from scandium.utils.io import load_yaml


@dataclass
class CameraIntrinsics:
    """
    Camera intrinsic parameters.

    Attributes:
        K: 3x3 camera matrix.
        dist_coeffs: Distortion coefficients (k1, k2, p1, p2, k3, ...).
        width: Image width.
        height: Image height.
    """

    K: NDArray[np.float64]
    dist_coeffs: NDArray[np.float64]
    width: int
    height: int

    @classmethod
    def from_yaml(cls, path: Path) -> "CameraIntrinsics":
        """
        Load intrinsics from YAML file.

        Expected format:
            camera_matrix: [[fx, 0, cx], [0, fy, cy], [0, 0, 1]]
            dist_coeffs: [k1, k2, p1, p2, k3]
            image_width: 1280
            image_height: 720

        Args:
            path: Path to YAML file.

        Returns:
            CameraIntrinsics instance.
        """
        data = load_yaml(path)

        K = np.array(data["camera_matrix"], dtype=np.float64)
        dist_coeffs = np.array(
            data.get("dist_coeffs", [0, 0, 0, 0, 0]), dtype=np.float64
        )
        width = int(data.get("image_width", 1280))
        height = int(data.get("image_height", 720))

        return cls(K=K, dist_coeffs=dist_coeffs, width=width, height=height)

    @classmethod
    def default(cls, width: int = 1280, height: int = 720) -> "CameraIntrinsics":
        """
        Create default intrinsics (approximate, for testing).

        Uses typical webcam parameters.

        Args:
            width: Image width.
            height: Image height.

        Returns:
            CameraIntrinsics with default values.
        """
        fx = fy = width  # Approximate focal length
        cx, cy = width / 2, height / 2

        K = np.array(
            [
                [fx, 0, cx],
                [0, fy, cy],
                [0, 0, 1],
            ],
            dtype=np.float64,
        )

        return cls(
            K=K,
            dist_coeffs=np.zeros(5, dtype=np.float64),
            width=width,
            height=height,
        )

    @property
    def fx(self) -> float:
        """Focal length x."""
        return float(self.K[0, 0])

    @property
    def fy(self) -> float:
        """Focal length y."""
        return float(self.K[1, 1])

    @property
    def cx(self) -> float:
        """Principal point x."""
        return float(self.K[0, 2])

    @property
    def cy(self) -> float:
        """Principal point y."""
        return float(self.K[1, 2])


@dataclass
class CameraExtrinsics:
    """
    Camera extrinsic parameters (camera to body transform).

    Attributes:
        R: 3x3 rotation matrix (camera -> body).
        t: 3-element translation vector (camera -> body).
    """

    R: NDArray[np.float64]
    t: NDArray[np.float64]

    @classmethod
    def from_yaml(cls, path: Path) -> "CameraExtrinsics":
        """
        Load extrinsics from YAML file.

        Expected format:
            rotation_matrix: [[r11, r12, r13], [r21, r22, r23], [r31, r32, r33]]
            translation: [tx, ty, tz]

        Args:
            path: Path to YAML file.

        Returns:
            CameraExtrinsics instance.
        """
        data = load_yaml(path)

        R = np.array(data["rotation_matrix"], dtype=np.float64)
        t = np.array(data.get("translation", [0, 0, 0]), dtype=np.float64)

        return cls(R=R, t=t)

    @classmethod
    def identity(cls) -> "CameraExtrinsics":
        """
        Create identity transform (camera aligned with body).

        Returns:
            CameraExtrinsics with identity transform.
        """
        return cls(
            R=np.eye(3, dtype=np.float64),
            t=np.zeros(3, dtype=np.float64),
        )

    @classmethod
    def downward_facing(cls) -> "CameraExtrinsics":
        """
        Create extrinsics for downward-facing camera.

        Assumes camera looks down (-Z body), with X forward.
        OpenCV camera: +Z forward, +X right, +Y down
        Body NED: +X forward, +Y right, +Z down

        Returns:
            CameraExtrinsics for downward camera.
        """
        # Rotation to convert camera frame to body frame
        # Camera +Z -> Body -Z (camera looks down)
        # Camera +X -> Body +Y (camera right is body right)
        # Camera +Y -> Body +X (camera down is body forward)
        R = np.array(
            [
                [0, 1, 0],
                [1, 0, 0],
                [0, 0, -1],
            ],
            dtype=np.float64,
        )

        return cls(R=R, t=np.zeros(3, dtype=np.float64))

    def transform_point(self, p_cam: NDArray[np.float64]) -> NDArray[np.float64]:
        """
        Transform point from camera frame to body frame.

        Args:
            p_cam: Point in camera frame.

        Returns:
            Point in body frame.
        """
        return self.R @ p_cam + self.t


class CalibrationManager:
    """
    Manages camera intrinsics and extrinsics.
    """

    def __init__(
        self,
        intrinsics: Optional[CameraIntrinsics] = None,
        extrinsics: Optional[CameraExtrinsics] = None,
    ) -> None:
        """
        Initialize calibration manager.

        Args:
            intrinsics: Camera intrinsics.
            extrinsics: Camera extrinsics.
        """
        self._intrinsics = intrinsics
        self._extrinsics = extrinsics

    @classmethod
    def from_config(
        cls,
        intrinsics_path: Optional[Path] = None,
        extrinsics_path: Optional[Path] = None,
        width: int = 1280,
        height: int = 720,
    ) -> "CalibrationManager":
        """
        Load calibration from config paths.

        Args:
            intrinsics_path: Path to intrinsics YAML (optional).
            extrinsics_path: Path to extrinsics YAML (optional).
            width: Default image width if no intrinsics.
            height: Default image height if no intrinsics.

        Returns:
            CalibrationManager instance.
        """
        intrinsics: Optional[CameraIntrinsics] = None
        extrinsics: Optional[CameraExtrinsics] = None

        if intrinsics_path and Path(intrinsics_path).exists():
            intrinsics = CameraIntrinsics.from_yaml(Path(intrinsics_path))
        else:
            intrinsics = CameraIntrinsics.default(width, height)

        if extrinsics_path and Path(extrinsics_path).exists():
            extrinsics = CameraExtrinsics.from_yaml(Path(extrinsics_path))
        else:
            extrinsics = CameraExtrinsics.downward_facing()

        return cls(intrinsics=intrinsics, extrinsics=extrinsics)

    @property
    def intrinsics(self) -> CameraIntrinsics:
        """Get intrinsics (default if not set)."""
        if self._intrinsics is None:
            self._intrinsics = CameraIntrinsics.default()
        return self._intrinsics

    @property
    def extrinsics(self) -> CameraExtrinsics:
        """Get extrinsics (identity if not set)."""
        if self._extrinsics is None:
            self._extrinsics = CameraExtrinsics.identity()
        return self._extrinsics
