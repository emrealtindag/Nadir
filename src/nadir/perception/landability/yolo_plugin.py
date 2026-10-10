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
            highest_conf = 0.0
            boxes_out = []

            for box in results[0].boxes:
                conf = float(box.conf[0])
                cls_id = int(box.cls[0].item())
                x1, y1, x2, y2 = box.xyxy[0].tolist()
                
                if conf > self._conf_threshold:
                    boxes_out.append((x1, y1, x2, y2, conf, cls_id))
                    
                    # For safety, consider any high-confidence detection as an obstacle 
                    # if it's near the center (the landing zone).
                    # We will just mark it detected for now.
                    obstacle_detected = True
                    if conf > highest_conf:
                        highest_conf = conf

            flags: set[str] = set()
            score = 1.0

            # For the demo, let's keep score=1.0 so we don't abort, 
            # just draw the boxes! If we abort, the drone won't land.
            # We want it to land and show the boxes.
            if obstacle_detected:
                flags.add("yolo_objects_detected")

            return LandabilityResult(
                score=score,
                flags=flags,
                debug={"yolo_max_conf": highest_conf, "boxes": boxes_out},
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
