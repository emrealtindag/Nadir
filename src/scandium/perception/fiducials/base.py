"""
Base classes for fiducial detection.

Provides abstract interface and data types for marker detection.
"""

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import Optional

import numpy as np
from numpy.typing import NDArray

from scandium.perception.camera import Frame


@dataclass
class FiducialDetection:
    """
    Single fiducial marker detection result.

    Attributes:
        id: Marker ID.
        corners_px: Corner pixel coordinates, shape (4, 2), order: TL, TR, BR, BL.
        confidence: Detection confidence [0, 1].
        center_px: Center point in pixels.
        area_px: Marker area in pixels (useful for distance estimation).
    """

    id: int
    corners_px: NDArray[np.float64]
    confidence: float = 1.0
    center_px: Optional[NDArray[np.float64]] = None
    area_px: Optional[float] = None

    def __post_init__(self) -> None:
        """Compute derived fields."""
        if self.center_px is None:
            self.center_px = np.mean(self.corners_px, axis=0)
        if self.area_px is None:
            self.area_px = self._compute_area()

    def _compute_area(self) -> float:
        """Compute marker area using shoelace formula."""
        n = len(self.corners_px)
        area = 0.0
        for i in range(n):
            j = (i + 1) % n
            area += self.corners_px[i, 0] * self.corners_px[j, 1]
            area -= self.corners_px[j, 0] * self.corners_px[i, 1]
        return abs(area) / 2.0

    @property
    def is_valid(self) -> bool:
        """Check if detection is valid."""
        return self.corners_px.shape == (4, 2) and self.confidence > 0 and self.id >= 0


class IFiducialDetector(ABC):
    """
    Abstract base class for fiducial marker detectors.
    """

    @abstractmethod
    def detect(self, frame: Frame) -> list[FiducialDetection]:
        """
        Detect fiducial markers in frame.

        Args:
            frame: Input camera frame.

        Returns:
            List of detected markers.
        """
        pass

    @abstractmethod
    def configure(self, **kwargs: object) -> None:
        """
        Configure detector parameters.

        Args:
            **kwargs: Backend-specific parameters.
        """
        pass

    @property
    @abstractmethod
    def backend_name(self) -> str:
        """Return backend name (aruco, apriltag, etc.)."""
        pass


def filter_by_allowlist(
    detections: list[FiducialDetection],
    allowlist: list[int],
) -> list[FiducialDetection]:
    """
    Filter detections by ID allowlist.

    Args:
        detections: List of detections.
        allowlist: List of allowed marker IDs.

    Returns:
        Filtered list containing only allowed IDs.
    """
    if not allowlist:
        return detections
    return [d for d in detections if d.id in allowlist]


def filter_by_area(
    detections: list[FiducialDetection],
    min_area: float = 0.0,
    max_area: float = float("inf"),
) -> list[FiducialDetection]:
    """
    Filter detections by marker area.

    Args:
        detections: List of detections.
        min_area: Minimum area in pixels.
        max_area: Maximum area in pixels.

    Returns:
        Filtered list.
    """
    return [
        d
        for d in detections
        if d.area_px is not None and min_area <= d.area_px <= max_area
    ]
