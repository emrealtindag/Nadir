"""
AprilTag marker detector for Scandium.

Implements fiducial detection using AprilTag library (if available).
Falls back to OpenCV's AprilTag support.
"""

from typing import Optional

import cv2
import numpy as np

from scandium.perception.camera import Frame
from scandium.perception.fiducials.base import FiducialDetection, IFiducialDetector


class AprilTagDetector(IFiducialDetector):
    """
    AprilTag marker detector.

    Uses OpenCV's ArUco module with AprilTag dictionaries as fallback
    if native AprilTag library is not available.
    """

    def __init__(
        self,
        family: str = "tag36h11",
        quad_decimate: float = 1.0,
        quad_sigma: float = 0.0,
        allowlist: list[int] | None = None,
    ) -> None:
        """
        Initialize AprilTag detector.

        Args:
            family: Tag family (tag16h5, tag25h9, tag36h10, tag36h11).
            quad_decimate: Quad decimation factor.
            quad_sigma: Gaussian blur sigma for quad detection.
            allowlist: Optional list of allowed tag IDs.
        """
        self._family = family
        self._quad_decimate = quad_decimate
        self._quad_sigma = quad_sigma
        self._allowlist = allowlist or []

        # Try native AprilTag first
        self._native_detector: Optional[object] = None
        try:
            self._init_native()
        except ImportError:
            # Fall back to OpenCV ArUco with AprilTag dictionaries
            self._init_opencv_fallback()

    def _init_native(self) -> None:
        """Initialize native AprilTag detector."""
        import apriltag

        options = apriltag.DetectorOptions(
            families=self._family,
            quad_decimate=self._quad_decimate,
            quad_sigma=self._quad_sigma,
        )
        self._native_detector = apriltag.Detector(options)

    def _init_opencv_fallback(self) -> None:
        """Initialize OpenCV fallback detector."""
        # Map AprilTag family to OpenCV dictionary
        family_map = {
            "tag16h5": cv2.aruco.DICT_APRILTAG_16h5,
            "tag25h9": cv2.aruco.DICT_APRILTAG_25h9,
            "tag36h10": cv2.aruco.DICT_APRILTAG_36h10,
            "tag36h11": cv2.aruco.DICT_APRILTAG_36h11,
        }

        dict_id = family_map.get(self._family, cv2.aruco.DICT_APRILTAG_36h11)
        aruco_dict = cv2.aruco.getPredefinedDictionary(dict_id)
        params = cv2.aruco.DetectorParameters()
        params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX

        self._opencv_detector = cv2.aruco.ArucoDetector(aruco_dict, params)

    def detect(self, frame: Frame) -> list[FiducialDetection]:
        """
        Detect AprilTag markers in frame.

        Args:
            frame: Input camera frame.

        Returns:
            List of detected markers.
        """
        gray = cv2.cvtColor(frame.image_bgr, cv2.COLOR_BGR2GRAY)

        if self._native_detector is not None:
            return self._detect_native(gray)
        else:
            return self._detect_opencv(gray)

    def _detect_native(self, gray: "np.ndarray") -> list[FiducialDetection]:
        """Detect using native AprilTag library."""
        import apriltag

        detector = self._native_detector
        assert detector is not None

        results = detector.detect(gray)  # type: ignore[union-attr]

        detections: list[FiducialDetection] = []
        for r in results:
            if self._allowlist and r.tag_id not in self._allowlist:
                continue

            # AprilTag returns corners in specific order
            corners = r.corners.astype(np.float64)

            detection = FiducialDetection(
                id=int(r.tag_id),
                corners_px=corners,
                confidence=float(r.decision_margin) / 100.0,
            )
            detections.append(detection)

        return detections

    def _detect_opencv(self, gray: "np.ndarray") -> list[FiducialDetection]:
        """Detect using OpenCV fallback."""
        corners, ids, rejected = self._opencv_detector.detectMarkers(gray)

        if ids is None or len(ids) == 0:
            return []

        detections: list[FiducialDetection] = []
        for i, tag_id in enumerate(ids.flatten()):
            if self._allowlist and tag_id not in self._allowlist:
                continue

            marker_corners = corners[i].reshape(4, 2).astype(np.float64)

            detection = FiducialDetection(
                id=int(tag_id),
                corners_px=marker_corners,
                confidence=1.0,
            )
            detections.append(detection)

        return detections

    def configure(self, **kwargs: object) -> None:
        """Configure detector parameters."""
        if "family" in kwargs:
            self._family = str(kwargs["family"])
            # Reinitialize detector
            try:
                self._init_native()
            except ImportError:
                self._init_opencv_fallback()

        if "allowlist" in kwargs:
            self._allowlist = list(kwargs["allowlist"])  # type: ignore[arg-type]

    @property
    def backend_name(self) -> str:
        """Return backend name."""
        if self._native_detector is not None:
            return "apriltag-native"
        return "apriltag-opencv"

    @property
    def family(self) -> str:
        """Return tag family."""
        return self._family
