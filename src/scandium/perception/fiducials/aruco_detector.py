"""
ArUco marker detector for Scandium.

Implements fiducial detection using OpenCV's ArUco module.
"""

from typing import Any

import cv2
import numpy as np
from numpy.typing import NDArray

from scandium.perception.camera import Frame
from scandium.perception.fiducials.base import FiducialDetection, IFiducialDetector


# ArUco dictionary mapping
ARUCO_DICTS = {
    "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
    "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
    "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
    "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
    "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
    "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
    "DICT_5X5_250": cv2.aruco.DICT_5X5_250,
    "DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
    "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
    "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
    "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
    "DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
    "DICT_7X7_50": cv2.aruco.DICT_7X7_50,
    "DICT_7X7_100": cv2.aruco.DICT_7X7_100,
    "DICT_7X7_250": cv2.aruco.DICT_7X7_250,
    "DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
    "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
    "DICT_APRILTAG_16h5": cv2.aruco.DICT_APRILTAG_16h5,
    "DICT_APRILTAG_25h9": cv2.aruco.DICT_APRILTAG_25h9,
    "DICT_APRILTAG_36h10": cv2.aruco.DICT_APRILTAG_36h10,
    "DICT_APRILTAG_36h11": cv2.aruco.DICT_APRILTAG_36h11,
}


class ArUcoDetector(IFiducialDetector):
    """
    ArUco marker detector using OpenCV.

    Supports various ArUco dictionaries and corner refinement.
    """

    def __init__(
        self,
        dictionary: str = "DICT_4X4_100",
        refine: bool = True,
        allowlist: list[int] | None = None,
    ) -> None:
        """
        Initialize ArUco detector.

        Args:
            dictionary: ArUco dictionary name.
            refine: Enable corner refinement.
            allowlist: Optional list of allowed marker IDs.
        """
        self._dictionary_name = dictionary
        self._refine = refine
        self._allowlist = allowlist or []

        # Get dictionary
        dict_id = ARUCO_DICTS.get(dictionary, cv2.aruco.DICT_4X4_100)
        self._aruco_dict = cv2.aruco.getPredefinedDictionary(dict_id)

        # Create detector parameters
        self._params = cv2.aruco.DetectorParameters()
        if refine:
            self._params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX

        # Create detector
        self._detector = cv2.aruco.ArucoDetector(self._aruco_dict, self._params)

    def detect(self, frame: Frame) -> list[FiducialDetection]:
        """
        Detect ArUco markers in frame.

        Args:
            frame: Input camera frame.

        Returns:
            List of detected markers.
        """
        # Convert to grayscale for detection
        gray = cv2.cvtColor(frame.image_bgr, cv2.COLOR_BGR2GRAY)

        # Detect markers
        corners, ids, rejected = self._detector.detectMarkers(gray)

        if ids is None or len(ids) == 0:
            return []

        detections: list[FiducialDetection] = []

        for i, marker_id in enumerate(ids.flatten()):
            # Filter by allowlist if set
            if self._allowlist and marker_id not in self._allowlist:
                continue

            # Get corners for this marker
            marker_corners = corners[i].reshape(4, 2).astype(np.float64)

            detection = FiducialDetection(
                id=int(marker_id),
                corners_px=marker_corners,
                confidence=1.0,  # ArUco doesn't provide confidence
            )
            detections.append(detection)

        return detections

    def configure(self, **kwargs: object) -> None:
        """Configure detector parameters."""
        if "dictionary" in kwargs:
            dict_name = str(kwargs["dictionary"])
            if dict_name in ARUCO_DICTS:
                self._dictionary_name = dict_name
                dict_id = ARUCO_DICTS[dict_name]
                self._aruco_dict = cv2.aruco.getPredefinedDictionary(dict_id)
                self._detector = cv2.aruco.ArucoDetector(self._aruco_dict, self._params)

        if "refine" in kwargs:
            self._refine = bool(kwargs["refine"])
            if self._refine:
                self._params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
            else:
                self._params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_NONE
            self._detector = cv2.aruco.ArucoDetector(self._aruco_dict, self._params)

        if "allowlist" in kwargs:
            self._allowlist = list(kwargs["allowlist"])  # type: ignore[arg-type]

    @property
    def backend_name(self) -> str:
        """Return backend name."""
        return "aruco"

    @property
    def dictionary_name(self) -> str:
        """Return dictionary name."""
        return self._dictionary_name


def draw_detections(
    frame: Frame,
    detections: list[FiducialDetection],
    color: tuple[int, int, int] = (0, 255, 0),
    thickness: int = 2,
) -> NDArray[np.uint8]:
    """
    Draw detections on frame copy.

    Args:
        frame: Input frame.
        detections: List of detections to draw.
        color: BGR color for drawing.
        thickness: Line thickness.

    Returns:
        Frame copy with drawn detections.
    """
    output = frame.image_bgr.copy()

    for det in detections:
        # Draw corners
        corners = det.corners_px.astype(np.int32)
        cv2.polylines(output, [corners], True, color, thickness)

        # Draw center
        if det.center_px is not None:
            center = tuple(det.center_px.astype(np.int32))
            cv2.circle(output, center, 5, color, -1)

        # Draw ID
        if det.center_px is not None:
            org = (int(det.center_px[0]) - 10, int(det.center_px[1]) - 20)
            cv2.putText(
                output,
                f"ID:{det.id}",
                org,
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                color,
                2,
            )

    return output
