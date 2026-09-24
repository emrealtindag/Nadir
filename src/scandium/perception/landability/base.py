"""
Base classes for landability estimation.

Provides abstract interface and result types for landing zone assessment.
"""

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import Any, Optional

import numpy as np
from numpy.typing import NDArray

from scandium.perception.camera import Frame


@dataclass
class LandabilityResult:
    """
    Landability assessment result.

    Attributes:
        score: Overall landability score [0, 1]. Higher = safer.
        flags: Set of risk flags (e.g., 'human_present', 'obstacle_present').
        debug: Debug information for visualization.
        roi_center: ROI center in pixels.
        roi_size: ROI size in pixels.
    """

    score: float
    flags: set[str] = field(default_factory=set)
    debug: dict[str, Any] = field(default_factory=dict)
    roi_center: Optional[tuple[int, int]] = None
    roi_size: Optional[tuple[int, int]] = None

    @property
    def is_safe(self) -> bool:
        """Check if landing zone is considered safe (no critical flags)."""
        critical_flags = {"human_present", "vehicle_present"}
        return len(self.flags & critical_flags) == 0 and self.score > 0.4

    @property
    def should_abort(self) -> bool:
        """Check if landing should be aborted."""
        return "human_present" in self.flags or self.score < 0.2

    def to_dict(self) -> dict[str, Any]:
        """Convert to dictionary for logging."""
        return {
            "score": round(self.score, 3),
            "flags": list(self.flags),
            "is_safe": self.is_safe,
            "should_abort": self.should_abort,
            "roi_center": self.roi_center,
            "roi_size": self.roi_size,
        }


class ILandabilityEstimator(ABC):
    """
    Abstract base class for landability estimators.
    """

    @abstractmethod
    def estimate(
        self,
        frame: Frame,
        roi_center: Optional[tuple[int, int]] = None,
        roi_size: Optional[tuple[int, int]] = None,
    ) -> LandabilityResult:
        """
        Estimate landability of the current frame.

        Args:
            frame: Input camera frame.
            roi_center: Optional ROI center (x, y) in pixels.
            roi_size: Optional ROI size (width, height) in pixels.

        Returns:
            LandabilityResult with score and flags.
        """
        pass

    @abstractmethod
    def reset(self) -> None:
        """Reset estimator state."""
        pass

    @property
    @abstractmethod
    def method_name(self) -> str:
        """Return method name (heuristic, ml, etc.)."""
        pass


def extract_roi(
    image: NDArray[np.uint8],
    center: tuple[int, int],
    size: tuple[int, int],
) -> NDArray[np.uint8]:
    """
    Extract region of interest from image.

    Args:
        image: Input image.
        center: ROI center (x, y).
        size: ROI size (width, height).

    Returns:
        ROI image (may be smaller if at edge).
    """
    h, w = image.shape[:2]
    half_w, half_h = size[0] // 2, size[1] // 2

    x1 = max(0, center[0] - half_w)
    y1 = max(0, center[1] - half_h)
    x2 = min(w, center[0] + half_w)
    y2 = min(h, center[1] + half_h)

    return image[y1:y2, x1:x2]
