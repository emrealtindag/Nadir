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

CLASS_NAMES_DEFAULT = {0: "Tasit", 1: "Insan"}


class YOLOLandabilityPlugin(ILandabilityEstimator):
    """
    YOLO-based dynamic obstacle and human detector for landing zone safety.
    Strictly filters detections to Tasit (vehicles) and Insan (humans).
    """

    def __init__(
        self,
        weights_path: str = "weights/best.pt",
        confidence_threshold: float = 0.40,
    ) -> None:
        self._weights_path = weights_path
        self._conf_threshold = confidence_threshold
        self._model: Optional[Any] = None
        self._class_names: dict[int, str] = CLASS_NAMES_DEFAULT
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
            # Strictly restrict supported classes to Tasit (0) and Insan (1)
            self._class_names = {0: "Tasit", 1: "Insan"}
        except Exception:
            self._initialized = False
            self._class_names = CLASS_NAMES_DEFAULT

    def estimate(
        self,
        frame: Frame,
        roi_center: Optional[tuple[int, int]] = None,
        roi_size: Optional[tuple[int, int]] = None,
    ) -> LandabilityResult:
        if not self._initialized or not self._model:
            return self._fallback_estimate(frame, roi_center, roi_size)

        try:
            # Run YOLO inference on the full image
            results = self._model(frame.image_bgr, verbose=False)
            
            obstacle_detected = False
            obstacle_in_zone = False
            human_in_zone = False
            vehicle_in_zone = False
            highest_conf = 0.0
            boxes_out = []

            # If landing pad ROI is provided, construct landing zone perimeter box.
            # If target ROI is not yet provided (e.g. ArUco occluded by vehicle or searching),
            # monitor the central nadir descent corridor (center 50% of camera FOV).
            pad_box = None
            if roi_center is not None and roi_size is not None:
                pad_box = (
                    roi_center[0] - roi_size[0] / 2.0,
                    roi_center[1] - roi_size[1] / 2.0,
                    roi_center[0] + roi_size[0] / 2.0,
                    roi_center[1] + roi_size[1] / 2.0,
                )
            else:
                h_img, w_img = frame.image_bgr.shape[:2]
                cw = w_img * 0.50
                ch = h_img * 0.50
                pad_box = (
                    (w_img - cw) / 2.0,
                    (h_img - ch) / 2.0,
                    (w_img + cw) / 2.0,
                    (h_img + ch) / 2.0,
                )

            for box in results[0].boxes:
                conf = float(box.conf[0])
                cls_id = int(box.cls[0].item()) if hasattr(box.cls[0], 'item') else int(box.cls[0])
                coords = box.xyxy[0]
                x1, y1, x2, y2 = coords.tolist() if hasattr(coords, 'tolist') else list(coords)

                # Inspect raw model label if available
                raw_label = ""
                if hasattr(self._model, "names") and isinstance(self._model.names, dict):
                    raw_label = str(self._model.names.get(cls_id, "")).lower()

                # Strictly filter: ONLY Tasit (0) and Insan (1) are allowed
                is_tasit = (cls_id == 0) or ("tasit" in raw_label) or ("car" in raw_label) or ("vehicle" in raw_label)
                is_insan = (cls_id == 1) or ("insan" in raw_label) or ("person" in raw_label) or ("human" in raw_label)
                if not (is_tasit or is_insan):
                    continue

                if conf > self._conf_threshold:
                    intersects = False
                    if pad_box is not None:
                        overlap_x = max(0.0, min(x2, pad_box[2]) - max(x1, pad_box[0]))
                        overlap_y = max(0.0, min(y2, pad_box[3]) - max(y1, pad_box[1]))
                        if (overlap_x * overlap_y) > 0.0:
                            intersects = True

                    canonical_name = "Insan" if is_insan else "Tasit"
                    canonical_id = 1 if is_insan else 0

                    boxes_out.append((x1, y1, x2, y2, conf, canonical_id, canonical_name, intersects))
                    obstacle_detected = True
                    if conf > highest_conf:
                        highest_conf = conf

                    if intersects:
                        obstacle_in_zone = True
                        if is_insan:
                            human_in_zone = True
                        else:
                            vehicle_in_zone = True

            flags: set[str] = set()
            score = 1.0

            if obstacle_detected:
                flags.add("yolo_objects_detected")

            if human_in_zone:
                # Direct human presence on landing pad: strict zero-tolerance abort
                score = 0.0
                flags.add("human_present")
                flags.add("obstacle_in_landing_zone")
            elif vehicle_in_zone:
                # Vehicle occupying landing pad: collision hazard abort
                score = 0.20
                flags.add("vehicle_present")
                flags.add("obstacle_in_landing_zone")
            elif obstacle_detected:
                # Obstacle detected in camera FOV (e.g. runway car), but clear of pad perimeter
                score = 1.0
                flags.add("spatial_clearance_verified")

            return LandabilityResult(
                score=score,
                flags=flags,
                debug={
                    "yolo_max_conf": highest_conf,
                    "boxes": boxes_out,
                    "obstacle_in_zone": obstacle_in_zone,
                    "human_in_zone": human_in_zone,
                    "vehicle_in_zone": vehicle_in_zone,
                },
                roi_center=roi_center,
                roi_size=roi_size,
            )

        except Exception as e:
            return LandabilityResult(
                score=0.0,
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
            score=0.0,
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
