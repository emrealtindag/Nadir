"""
Telemetry data collection for Scandium.

Provides structures for collecting and reporting runtime metrics.
"""

from dataclasses import dataclass, field
from typing import Any, Optional
from collections import deque
import time


@dataclass
class TelemetryData:
    """Single telemetry measurement."""

    timestamp_s: float
    fps: float
    latency_ms: float
    target_confidence: float
    pose_x: Optional[float] = None
    pose_y: Optional[float] = None
    pose_z: Optional[float] = None
    variance: float = 0.0
    landability_score: float = 1.0
    fsm_state: str = "IDLE"
    frame_id: int = 0

    def to_dict(self) -> dict[str, Any]:
        """Convert to dictionary for logging/serialization."""
        return {
            "timestamp_s": self.timestamp_s,
            "fps": round(self.fps, 2),
            "latency_ms": round(self.latency_ms, 2),
            "target_confidence": round(self.target_confidence, 3),
            "pose": {
                "x": round(self.pose_x, 4) if self.pose_x is not None else None,
                "y": round(self.pose_y, 4) if self.pose_y is not None else None,
                "z": round(self.pose_z, 4) if self.pose_z is not None else None,
            },
            "variance": round(self.variance, 4),
            "landability_score": round(self.landability_score, 3),
            "fsm_state": self.fsm_state,
            "frame_id": self.frame_id,
        }


@dataclass
class TelemetryCollector:
    """
    Collects and aggregates telemetry data.

    Maintains a sliding window of recent measurements for statistics.
    """

    window_size: int = 100
    _data: deque[TelemetryData] = field(default_factory=lambda: deque(maxlen=100))
    _frame_times: deque[float] = field(default_factory=lambda: deque(maxlen=30))
    _last_frame_time: Optional[float] = None

    def __post_init__(self) -> None:
        """Initialize deques with correct maxlen."""
        self._data = deque(maxlen=self.window_size)
        self._frame_times = deque(maxlen=30)

    def record_frame_start(self) -> float:
        """
        Record the start of frame processing.

        Returns:
            Current timestamp.
        """
        now = time.perf_counter()
        if self._last_frame_time is not None:
            self._frame_times.append(now - self._last_frame_time)
        self._last_frame_time = now
        return now

    def record(self, data: TelemetryData) -> None:
        """Record a telemetry measurement."""
        self._data.append(data)

    def get_fps(self) -> float:
        """
        Calculate current FPS from recent frame times.

        Returns:
            Frames per second.
        """
        if len(self._frame_times) < 2:
            return 0.0
        avg_frame_time = sum(self._frame_times) / len(self._frame_times)
        return 1.0 / avg_frame_time if avg_frame_time > 0 else 0.0

    def get_latency_stats(self) -> dict[str, float]:
        """
        Calculate latency statistics.

        Returns:
            Dictionary with mean, min, max latency in ms.
        """
        if not self._data:
            return {"mean": 0.0, "min": 0.0, "max": 0.0}

        latencies = [d.latency_ms for d in self._data]
        return {
            "mean": sum(latencies) / len(latencies),
            "min": min(latencies),
            "max": max(latencies),
        }

    def get_summary(self) -> dict[str, Any]:
        """
        Get summary of collected telemetry.

        Returns:
            Summary dictionary.
        """
        if not self._data:
            return {
                "sample_count": 0,
                "fps": 0.0,
                "latency": {"mean": 0.0, "min": 0.0, "max": 0.0},
            }

        return {
            "sample_count": len(self._data),
            "fps": round(self.get_fps(), 2),
            "latency": self.get_latency_stats(),
            "confidence": {
                "mean": sum(d.target_confidence for d in self._data) / len(self._data),
                "min": min(d.target_confidence for d in self._data),
            },
            "latest_state": self._data[-1].fsm_state if self._data else "UNKNOWN",
        }

    def clear(self) -> None:
        """Clear all collected data."""
        self._data.clear()
        self._frame_times.clear()
        self._last_frame_time = None
