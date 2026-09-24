"""
ML-based landability estimation plugin for Scandium.

Provides interface for machine learning-based landing zone segmentation.
Note: To train or fine-tune the model, use `scripts/train_segmentation.py`
which utilizes PyTorch and DeepLabV3 architecture. The resulting PyTorch model 
is exported to ONNX format and loaded here via ONNXRuntime for fast inference 
in real-time constraints.
"""

from pathlib import Path
from typing import Optional

import numpy as np
from numpy.typing import NDArray

from scandium.perception.camera import Frame
from scandium.perception.landability.base import (
    LandabilityResult,
    ILandabilityEstimator,
    extract_roi,
)


class MLLandabilityPlugin(ILandabilityEstimator):
    """
    Machine learning-based landability estimator.

    Uses a trained segmentation model to classify landing zones.
    Supports ONNX models for inference.
    """

    def __init__(
        self,
        model_path: str = "",
        device: str = "cpu",
        score_threshold: float = 0.6,
        input_size: tuple[int, int] = (256, 256),
    ) -> None:
        """
        Initialize ML plugin.

        Args:
            model_path: Path to ONNX model file.
            device: Inference device ('cpu' or 'cuda').
            score_threshold: Minimum score for safe classification.
            input_size: Model input size (width, height).
        """
        self._model_path = model_path
        self._device = device
        self._score_threshold = score_threshold
        self._input_size = input_size

        self._session: Optional[object] = None
        self._initialized = False

        if model_path:
            self._load_model()

    def _load_model(self) -> None:
        """Load ONNX model for inference."""
        if not self._model_path or not Path(self._model_path).exists():
            return

        try:
            import onnxruntime as ort

            providers = ["CPUExecutionProvider"]
            if self._device == "cuda":
                providers.insert(0, "CUDAExecutionProvider")

            self._session = ort.InferenceSession(
                self._model_path,
                providers=providers,
            )
            self._initialized = True
        except ImportError:
            # onnxruntime not available
            self._initialized = False
        except Exception:
            self._initialized = False

    def estimate(
        self,
        frame: Frame,
        roi_center: Optional[tuple[int, int]] = None,
        roi_size: Optional[tuple[int, int]] = None,
    ) -> LandabilityResult:
        """
        Estimate landability using ML model.

        Args:
            frame: Input camera frame.
            roi_center: ROI center (x, y).
            roi_size: ROI size (w, h).

        Returns:
            LandabilityResult with score and flags.
        """
        if not self._initialized:
            # Fallback to basic heuristic if model not loaded
            return self._fallback_estimate(frame, roi_center, roi_size)

        h, w = frame.height, frame.width

        if roi_center is None:
            roi_center = (w // 2, h // 2)
        if roi_size is None:
            roi_size = (w // 4, h // 4)

        # Extract and preprocess ROI
        import cv2

        roi = extract_roi(frame.image_bgr, roi_center, roi_size)
        if roi.size == 0:
            return LandabilityResult(
                score=0.0,
                flags={"insufficient_roi"},
                roi_center=roi_center,
                roi_size=roi_size,
            )

        # Resize to model input size
        roi_resized = cv2.resize(roi, self._input_size)
        roi_rgb = cv2.cvtColor(roi_resized, cv2.COLOR_BGR2RGB)

        # Normalize according to ImageNet standards
        input_tensor = roi_rgb.astype(np.float32) / 255.0
        mean = np.array([0.485, 0.456, 0.406], dtype=np.float32)
        std = np.array([0.229, 0.224, 0.225], dtype=np.float32)
        input_tensor = (input_tensor - mean) / std
        
        input_tensor = np.transpose(input_tensor, (2, 0, 1))  # HWC -> CHW
        input_tensor = np.expand_dims(input_tensor, axis=0)  # Add batch

        # Run inference
        try:
            outputs = self._session.run(None, {"input": input_tensor})  # type: ignore
            # Apply softmax across classes (assuming output shape [1, 2, H, W])
            logits = outputs[0][0]
            exp_logits = np.exp(logits - np.max(logits, axis=0))
            probs = exp_logits / np.sum(exp_logits, axis=0)
            
            mask = probs[1]  # Probability of class 1 (safe zone)

            # Compute score from mask
            # Assuming binary mask: 1 = safe, 0 = unsafe
            score = float(np.mean(mask > 0.5))

            flags: set[str] = set()
            if score < self._score_threshold:
                flags.add("ml_unsafe_zone")

            return LandabilityResult(
                score=score,
                flags=flags,
                debug={"ml_confidence": score},
                roi_center=roi_center,
                roi_size=roi_size,
            )
        except Exception as e:
            return LandabilityResult(
                score=0.5,
                flags={"ml_inference_error"},
                debug={"error": str(e)},
                roi_center=roi_center,
                roi_size=roi_size,
            )

    def _fallback_estimate(
        self,
        frame: Frame,
        roi_center: Optional[tuple[int, int]],
        roi_size: Optional[tuple[int, int]],
    ) -> LandabilityResult:
        """Basic fallback when model not available."""
        return LandabilityResult(
            score=0.5,
            flags={"ml_model_unavailable"},
            debug={"fallback": True},
            roi_center=roi_center,
            roi_size=roi_size,
        )

    def reset(self) -> None:
        """Reset estimator state."""
        pass  # Stateless inference

    @property
    def method_name(self) -> str:
        """Return method name."""
        return "ml"

    @property
    def is_initialized(self) -> bool:
        """Check if model is loaded and ready."""
        return self._initialized
