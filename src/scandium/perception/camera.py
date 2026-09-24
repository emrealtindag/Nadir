"""
Camera abstraction for Scandium.

Provides unified interface for different camera sources including
AirSim, UVC cameras, and video files.
"""

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Optional

import numpy as np
from numpy.typing import NDArray


class CameraHealth(Enum):
    """Camera health status."""

    OK = "ok"
    DEGRADED = "degraded"
    ERROR = "error"
    DISCONNECTED = "disconnected"


@dataclass
class Frame:
    """
    Single camera frame with metadata.

    Attributes:
        image_bgr: BGR image as numpy array, shape (H, W, 3), dtype uint8.
        timestamp_s: UTC timestamp in seconds.
        frame_id: Sequential frame identifier.
        meta: Additional metadata dictionary.
    """

    image_bgr: NDArray[np.uint8]
    timestamp_s: float
    frame_id: int
    meta: dict[str, Any] = field(default_factory=dict)

    @property
    def shape(self) -> tuple[int, int, int]:
        """Image shape (H, W, C)."""
        return self.image_bgr.shape  # type: ignore[return-value]

    @property
    def height(self) -> int:
        """Image height in pixels."""
        return self.image_bgr.shape[0]

    @property
    def width(self) -> int:
        """Image width in pixels."""
        return self.image_bgr.shape[1]

    def to_rgb(self) -> NDArray[np.uint8]:
        """Convert to RGB format."""
        import cv2

        return cv2.cvtColor(self.image_bgr, cv2.COLOR_BGR2RGB)

    def to_gray(self) -> NDArray[np.uint8]:
        """Convert to grayscale."""
        import cv2

        return cv2.cvtColor(self.image_bgr, cv2.COLOR_BGR2GRAY)


class ICameraSource(ABC):
    """
    Abstract base class for camera sources.

    Implementations must provide frame reading and health monitoring.
    """

    @abstractmethod
    def read(self) -> Optional[Frame]:
        """
        Read next frame from camera.

        Returns:
            Frame if available, None otherwise.
        """
        pass

    @abstractmethod
    def close(self) -> None:
        """Release camera resources."""
        pass

    @abstractmethod
    def health(self) -> CameraHealth:
        """
        Get current camera health status.

        Returns:
            CameraHealth enum value.
        """
        pass

    @property
    @abstractmethod
    def is_open(self) -> bool:
        """Check if camera is open and ready."""
        pass


class UvcCameraSource(ICameraSource):
    """USB/UVC camera source using OpenCV VideoCapture."""

    def __init__(
        self,
        device_index: int = 0,
        width: int = 1280,
        height: int = 720,
        fps: int = 30,
    ) -> None:
        """
        Initialize UVC camera.

        Args:
            device_index: Camera device index.
            width: Frame width.
            height: Frame height.
            fps: Target FPS.
        """
        import cv2

        self._device_index = device_index
        self._cap = cv2.VideoCapture(device_index)
        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self._cap.set(cv2.CAP_PROP_FPS, fps)
        self._frame_id = 0
        self._health = (
            CameraHealth.OK if self._cap.isOpened() else CameraHealth.DISCONNECTED
        )

    def read(self) -> Optional[Frame]:
        """Read frame from UVC camera."""
        import time

        if not self._cap.isOpened():
            self._health = CameraHealth.DISCONNECTED
            return None

        ret, frame = self._cap.read()
        if not ret or frame is None:
            self._health = CameraHealth.ERROR
            return None

        self._frame_id += 1
        self._health = CameraHealth.OK

        return Frame(
            image_bgr=frame,
            timestamp_s=time.time(),
            frame_id=self._frame_id,
            meta={"source": "uvc", "device": self._device_index},
        )

    def close(self) -> None:
        """Release camera."""
        self._cap.release()
        self._health = CameraHealth.DISCONNECTED

    def health(self) -> CameraHealth:
        """Get camera health."""
        return self._health

    @property
    def is_open(self) -> bool:
        """Check if camera is open."""
        return self._cap.isOpened()


class VideoFileCameraSource(ICameraSource):
    """Video file source using OpenCV VideoCapture."""

    def __init__(self, video_path: str, loop: bool = True) -> None:
        """
        Initialize video file source.

        Args:
            video_path: Path to video file.
            loop: Whether to loop when reaching end.
        """
        import cv2

        self._path = video_path
        self._loop = loop
        self._cap = cv2.VideoCapture(video_path)
        self._frame_id = 0
        self._health = CameraHealth.OK if self._cap.isOpened() else CameraHealth.ERROR

    def read(self) -> Optional[Frame]:
        """Read frame from video file."""
        import cv2
        import time

        if not self._cap.isOpened():
            self._health = CameraHealth.DISCONNECTED
            return None

        ret, frame = self._cap.read()
        if not ret or frame is None:
            if self._loop:
                self._cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                ret, frame = self._cap.read()
                if not ret or frame is None:
                    self._health = CameraHealth.ERROR
                    return None
            else:
                self._health = CameraHealth.ERROR
                return None

        self._frame_id += 1
        self._health = CameraHealth.OK

        return Frame(
            image_bgr=frame,
            timestamp_s=time.time(),
            frame_id=self._frame_id,
            meta={"source": "video_file", "path": self._path},
        )

    def close(self) -> None:
        """Release video capture."""
        self._cap.release()
        self._health = CameraHealth.DISCONNECTED

    def health(self) -> CameraHealth:
        """Get source health."""
        return self._health

    @property
    def is_open(self) -> bool:
        """Check if video is open."""
        return self._cap.isOpened()


class AirSimCameraSource(ICameraSource):
    """AirSim simulation camera source."""

    def __init__(
        self,
        ip: str = "127.0.0.1",
        vehicle_name: str = "Drone1",
        camera_name: str = "0",
        image_type: str = "Scene",
    ) -> None:
        """
        Initialize AirSim camera.

        Args:
            ip: AirSim server IP.
            vehicle_name: Vehicle name in AirSim.
            camera_name: Camera name.
            image_type: Image type (Scene, Depth, etc.).
        """
        self._ip = ip
        self._vehicle_name = vehicle_name
        self._camera_name = camera_name
        self._image_type = image_type
        self._client: Any = None
        self._frame_id = 0
        self._health = CameraHealth.DISCONNECTED

        self._connect()

    def _connect(self) -> None:
        """Connect to AirSim."""
        try:
            import airsim

            self._client = airsim.MultirotorClient(ip=self._ip)
            self._client.confirmConnection()
            self._health = CameraHealth.OK
        except Exception:
            self._health = CameraHealth.DISCONNECTED
            self._client = None

    def read(self) -> Optional[Frame]:
        """Read frame from AirSim."""
        import time

        if self._client is None:
            return None

        try:
            import airsim
            import numpy as np

            # Get image type enum
            img_type = getattr(
                airsim.ImageType, self._image_type, airsim.ImageType.Scene
            )

            responses = self._client.simGetImages(
                [airsim.ImageRequest(self._camera_name, img_type, False, False)],
                vehicle_name=self._vehicle_name,
            )

            if not responses or len(responses) == 0:
                self._health = CameraHealth.ERROR
                return None

            response = responses[0]
            img1d = np.frombuffer(response.image_data_uint8, dtype=np.uint8)
            img_bgr = img1d.reshape(response.height, response.width, 3)

            self._frame_id += 1
            self._health = CameraHealth.OK

            return Frame(
                image_bgr=img_bgr,
                timestamp_s=time.time(),
                frame_id=self._frame_id,
                meta={
                    "source": "airsim",
                    "vehicle": self._vehicle_name,
                    "camera": self._camera_name,
                },
            )
        except Exception:
            self._health = CameraHealth.ERROR
            return None

    def close(self) -> None:
        """Close AirSim connection."""
        self._client = None
        self._health = CameraHealth.DISCONNECTED

    def health(self) -> CameraHealth:
        """Get connection health."""
        return self._health

    @property
    def is_open(self) -> bool:
        """Check if connected to AirSim."""
        return self._client is not None and self._health == CameraHealth.OK
