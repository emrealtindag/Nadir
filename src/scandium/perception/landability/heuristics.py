"""
Heuristic-based landability estimation for Scandium.

Provides model-free landing zone assessment using texture, motion, and edge analysis.
"""

from typing import Optional

import cv2
import numpy as np
from numpy.typing import NDArray

from scandium.perception.camera import Frame
from scandium.perception.landability.base import (
    LandabilityResult,
    ILandabilityEstimator,
    extract_roi,
)


class HeuristicLandabilityEstimator(ILandabilityEstimator):
    """
    Heuristic-based landability estimator.

    Uses texture variance, motion detection, and edge density
    to assess landing zone safety.
    """

    def __init__(
        self,
        texture_var_min: float = 12.0,
        motion_threshold: float = 0.15,
        edge_density_max: float = 0.3,
        low_light_threshold: float = 30.0,
        default_roi_scale: float = 2.0,
    ) -> None:
        """
        Initialize heuristic estimator.

        Args:
            texture_var_min: Minimum texture variance for good visibility.
            motion_threshold: Motion detection threshold.
            edge_density_max: Maximum edge density before flagging.
            low_light_threshold: Mean intensity below which low_light flag is set.
            default_roi_scale: Default ROI scale relative to marker.
        """
        self._texture_var_min = texture_var_min
        self._motion_threshold = motion_threshold
        self._edge_density_max = edge_density_max
        self._low_light_threshold = low_light_threshold
        self._default_roi_scale = default_roi_scale

        self._prev_frame: Optional[NDArray[np.uint8]] = None
        self._prev_roi: Optional[NDArray[np.uint8]] = None

    def estimate(
        self,
        frame: Frame,
        roi_center: Optional[tuple[int, int]] = None,
        roi_size: Optional[tuple[int, int]] = None,
    ) -> LandabilityResult:
        """
        Estimate landability using heuristics.

        Args:
            frame: Input camera frame.
            roi_center: ROI center (x, y). Defaults to frame center.
            roi_size: ROI size (w, h). Defaults to 1/4 of frame.

        Returns:
            LandabilityResult with score and flags.
        """
        h, w = frame.height, frame.width

        # Default ROI if not specified
        if roi_center is None:
            roi_center = (w // 2, h // 2)
        if roi_size is None:
            roi_size = (w // 4, h // 4)

        # Extract ROI
        gray = frame.to_gray()
        roi = extract_roi(gray, roi_center, roi_size)

        if roi.size == 0:
            return LandabilityResult(
                score=0.0,
                flags={"insufficient_roi"},
                roi_center=roi_center,
                roi_size=roi_size,
            )

        # Compute features
        texture_score = self._compute_texture_score(roi)
        motion_score = self._compute_motion_score(roi)
        edge_score = self._compute_edge_score(roi)
        light_score = self._compute_light_score(roi)

        # Aggregate score
        score = (
            0.3 * texture_score
            + 0.3 * motion_score
            + 0.2 * edge_score
            + 0.2 * light_score
        )

        # Determine flags
        flags: set[str] = set()
        debug: dict = {
            "texture_score": texture_score,
            "motion_score": motion_score,
            "edge_score": edge_score,
            "light_score": light_score,
        }

        if texture_score < 0.3:
            flags.add("insufficient_texture")
        if motion_score < 0.5:
            flags.add("high_motion")
        if edge_score < 0.5:
            flags.add("high_edge_density")
        if light_score < 0.3:
            flags.add("low_light")

        # Store for next frame
        self._prev_roi = roi.copy()

        return LandabilityResult(
            score=score,
            flags=flags,
            debug=debug,
            roi_center=roi_center,
            roi_size=roi_size,
        )

    def _compute_texture_score(self, roi: NDArray[np.uint8]) -> float:
        """
        Compute texture variance score.

        Higher variance = more texture = better visibility.
        """
        laplacian = cv2.Laplacian(roi, cv2.CV_64F)
        variance = float(laplacian.var())

        # Normalize to [0, 1]
        if variance >= self._texture_var_min:
            return 1.0
        return variance / self._texture_var_min

    def _compute_motion_score(self, roi: NDArray[np.uint8]) -> float:
        """
        Compute motion detection score.

        Lower motion = safer = higher score.
        """
        if self._prev_roi is None or self._prev_roi.shape != roi.shape:
            # No previous frame, assume no motion
            return 1.0

        # Frame difference
        diff = cv2.absdiff(roi, self._prev_roi)
        motion_ratio = float(np.mean(diff)) / 255.0

        if motion_ratio < self._motion_threshold:
            return 1.0

        # Scale down score based on motion
        return max(0.0, 1.0 - (motion_ratio - self._motion_threshold) * 5)

    def _compute_edge_score(self, roi: NDArray[np.uint8]) -> float:
        """
        Compute edge density score.

        Lower edge density = flatter surface = safer.
        """
        edges = cv2.Canny(roi, 50, 150)
        edge_density = float(np.sum(edges > 0)) / edges.size

        if edge_density <= self._edge_density_max:
            return 1.0

        # Reduce score for high edge density
        return max(0.0, 1.0 - (edge_density - self._edge_density_max) * 3)

    def _compute_light_score(self, roi: NDArray[np.uint8]) -> float:
        """
        Compute lighting condition score.

        Good lighting = higher score.
        """
        mean_intensity = float(np.mean(roi))

        if mean_intensity >= self._low_light_threshold:
            return min(1.0, mean_intensity / 128.0)

        return mean_intensity / self._low_light_threshold

    def reset(self) -> None:
        """Reset estimator state."""
        self._prev_frame = None
        self._prev_roi = None

    @property
    def method_name(self) -> str:
        """Return method name."""
        return "heuristic"
