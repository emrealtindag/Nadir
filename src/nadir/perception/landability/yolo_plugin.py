"""
YOLO-based landability estimation plugin for Nadir.

Modified to process the full frame and return bounding boxes for rendering.
"""

from typing import Optional, Any
import numpy as np
import cv2

try:
    from ultralytics import YOLO
    HAVE_YOLO = True
except ImportError:
    HAVE_YOLO = False

from nadir.perception.camera import Frame
from nadir.perception.landability.base import (
    LandabilityResult,
    ILandabilityEstimator,
    extract_roi,
)

class YOLOLandabilityPlugin(ILandabilityEstimator):
    """
    YOLO-based dynamic obstacle detector for landing zone safety.
    """

    def __init__(
        self,
        weights_path: str = "weights/best.pt",
        confidence_threshold: float = 0.40,
    ) -> None:
        self._weights_path = weights_path
        self._conf_threshold = confidence_threshold
        self._model: Optional[Any] = None
        self._initialized = False

        if HAVE_YOLO:
            self._load_model()

    def _load_model(self) -> None:
        from pathlib import Path
        candidate_paths = [
            Path(self._weights_path),
            Path("weights") / Path(self._weights_path).name,
            Path(__file__).resolve().parents[3] / "weights" / Path(self._weights_path).name,
            Path("best.pt"),
            Path(__file__).resolve().parents[3] / "best.pt",
        ]
        resolved = None
        for p in candidate_paths:
            if p.exists():
                resolved = str(p)
                break
        if resolved is None:
            resolved = self._weights_path

        try:
            self._model = YOLO(resolved)
            self._initialized = True
        except Exception:
            self._initialized = False

    def estimate(
        self,
        frame: Frame,
        roi_center: Optional[tuple[int, int]] = None,
        roi_size: Optional[tuple[int, int]] = None,
    ) -> LandabilityResult:
        if not self._initialized or not self._model:
            return self._fallback_estimate(frame, roi_center, roi_size)

        try:
            # Run YOLO inference on the FULL image so we can see all detections
            results = self._model(frame.image_bgr, verbose=False)
            
            obstacle_detected = False
            obstacle_in_zone = False
            highest_conf = 0.0
            boxes_out = []

            # If landing pad ROI is provided, construct landing zone perimeter box
            pad_box = None
            if roi_center is not None and roi_size is not None:
                pad_box = (
                    roi_center[0] - roi_size[0] / 2.0,
                    roi_center[1] - roi_size[1] / 2.0,
                    roi_center[0] + roi_size[0] / 2.0,
                    roi_center[1] + roi_size[1] / 2.0,
                )

            for box in results[0].boxes:
                conf = float(box.conf[0])
                cls_id = int(box.cls[0].item()) if hasattr(box.cls[0], 'item') else int(box.cls[0])
                coords = box.xyxy[0]
                x1, y1, x2, y2 = coords.tolist() if hasattr(coords, 'tolist') else list(coords)
                
                if conf > self._conf_threshold:
                    boxes_out.append((x1, y1, x2, y2, conf, cls_id))
                    obstacle_detected = True
                    if conf > highest_conf:
                        highest_conf = conf

                    # Spatial intersection check: does detected obstacle overlap the landing pad ROI?
                    if pad_box is not None:
                        overlap_x = max(0.0, min(x2, pad_box[2]) - max(x1, pad_box[0]))
                        overlap_y = max(0.0, min(y2, pad_box[3]) - max(y1, pad_box[1]))
                        if (overlap_x * overlap_y) > 0.0:
                            obstacle_in_zone = True

            flags: set[str] = set()
            score = 1.0

            if obstacle_detected:
                flags.add("yolo_objects_detected")

            if obstacle_in_zone:
                # Direct spatial collision hazard: obstacle intersects landing pad
                score = 0.20
                flags.add("obstacle_in_landing_zone")
            elif obstacle_detected:
                # Obstacle is visible in camera FOV (e.g., runway vehicle), but clear of pad perimeter
                score = 1.0
                flags.add("spatial_clearance_verified")

            return LandabilityResult(
                score=score,
                flags=flags,
                debug={
                    "yolo_max_conf": highest_conf,
                    "boxes": boxes_out,
                    "obstacle_in_zone": obstacle_in_zone,
                },
                roi_center=roi_center,
                roi_size=roi_size,
            )

        except Exception as e:
            return LandabilityResult(
                score=0.5,
                flags={"yolo_inference_error"},
                debug={"error": str(e)}
            )

    def _fallback_estimate(
        self,
        frame: Frame,
        roi_center: Optional[tuple[int, int]],
        roi_size: Optional[tuple[int, int]],
    ) -> LandabilityResult:
        return LandabilityResult(
            score=0.5,
            flags={"yolo_model_unavailable"},
            debug={"fallback": True},
        )

    def reset(self) -> None:
        pass

    @property
    def method_name(self) -> str:
        return "yolo"

    @property
    def is_initialized(self) -> bool:
        return self._initialized
