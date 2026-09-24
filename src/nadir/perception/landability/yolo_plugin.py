"""
YOLO-based landability estimation plugin for Nadir.

Provides dynamic obstacle detection (vehicles, humans) using ultralytics YOLO.
This ensures the landing zone is clear of dynamic threats, triggering GO_AROUND
if a vehicle or human enters the descent cone.
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
        weights_path: str = "yolov8s.pt",
        confidence_threshold: float = 0.40,
    ) -> None:
        self._weights_path = weights_path
        self._conf_threshold = confidence_threshold
        self._model: Optional[Any] = None
        self._initialized = False

        if HAVE_YOLO:
            self._load_model()

    def _load_model(self) -> None:
        try:
            self._model = YOLO(self._weights_path)
            self._initialized = True
        except Exception as e:
            self._initialized = False

    def estimate(
        self,
        frame: Frame,
        roi_center: Optional[tuple[int, int]] = None,
        roi_size: Optional[tuple[int, int]] = None,
    ) -> LandabilityResult:
        if not self._initialized or not self._model:
            return self._fallback_estimate(frame, roi_center, roi_size)

        h, w = frame.height, frame.width

        if roi_center is None:
            roi_center = (w // 2, h // 2)
        if roi_size is None:
            roi_size = (w // 2, h // 2)

        # Extract ROI for faster inference
        roi = extract_roi(frame.image_bgr, roi_center, roi_size)
        if roi.size == 0:
            return LandabilityResult(score=0.0, flags={"insufficient_roi"})

        try:
            # Run YOLO inference
            results = self._model(roi, verbose=False)
            
            obstacle_detected = False
            highest_conf = 0.0

            for box in results[0].boxes:
                conf = float(box.conf[0])
                cls_id = int(box.cls[0].item())
                
                # Class 0: Person, Class 2: Car, 3: Motorcycle, 5: Bus, 7: Truck (COCO classes)
                if cls_id in [0, 2, 3, 5, 7] and conf > self._conf_threshold:
                    obstacle_detected = True
                    if conf > highest_conf:
                        highest_conf = conf

            flags: set[str] = set()
            score = 1.0

            if obstacle_detected:
                flags.add("dynamic_obstacle_detected")
                score = 0.0  # Absolutely unsafe

            return LandabilityResult(
                score=score,
                flags=flags,
                debug={"yolo_max_conf": highest_conf},
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
