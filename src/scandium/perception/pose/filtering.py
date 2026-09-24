"""
Temporal filtering for pose estimation in Scandium.

Provides exponential smoothing and Kalman filtering for pose stabilization.
"""

from dataclasses import dataclass, field
from typing import Optional

import numpy as np
from numpy.typing import NDArray


@dataclass
class FilteredPose:
    """
    Filtered pose estimate with uncertainty.

    Attributes:
        position: Filtered position (x, y, z) in meters.
        variance: Position variance (uncertainty).
        velocity: Estimated velocity (vx, vy, vz) in m/s.
        is_valid: Whether the filter state is valid.
        timestamps: Timestamp of the filter state.
    """

    position: NDArray[np.float64]
    variance: NDArray[np.float64]
    velocity: NDArray[np.float64] = field(default_factory=lambda: np.zeros(3))
    is_valid: bool = True
    timestamp: float = 0.0


class ExpSmoother:
    """
    Exponential smoothing filter for 3D position.

    Implements first-order exponential smoothing with outlier rejection
    based on Mahalanobis distance.
    """

    def __init__(
        self,
        alpha: float = 0.35,
        outlier_threshold: float = 4.0,
        initial_variance: float = 1.0,
    ) -> None:
        """
        Initialize exponential smoother.

        Args:
            alpha: Smoothing factor [0, 1]. Higher = more responsive, less smooth.
            outlier_threshold: Mahalanobis distance threshold for outlier rejection.
            initial_variance: Initial variance for new state.
        """
        if not 0 < alpha <= 1:
            raise ValueError("alpha must be in (0, 1]")

        self._alpha = alpha
        self._outlier_threshold = outlier_threshold
        self._initial_variance = initial_variance

        self._state: Optional[NDArray[np.float64]] = None
        self._variance: Optional[NDArray[np.float64]] = None
        self._last_time: Optional[float] = None
        self._velocity: NDArray[np.float64] = np.zeros(3, dtype=np.float64)

    def update(
        self,
        measurement: NDArray[np.float64],
        measurement_variance: float = 0.1,
        timestamp: float = 0.0,
    ) -> FilteredPose:
        """
        Update filter with new measurement.

        Args:
            measurement: New position measurement (x, y, z).
            measurement_variance: Variance of the measurement.
            timestamp: Measurement timestamp.

        Returns:
            FilteredPose with updated state.
        """
        measurement = np.asarray(measurement, dtype=np.float64).flatten()

        # Initialize on first measurement
        if self._state is None:
            self._state = measurement.copy()
            self._variance = np.full(3, self._initial_variance, dtype=np.float64)
            self._last_time = timestamp
            return FilteredPose(
                position=self._state.copy(),
                variance=self._variance.copy(),
                velocity=self._velocity.copy(),
                is_valid=True,
                timestamp=timestamp,
            )

        # Check for outlier
        if self._is_outlier(measurement):
            # Increase variance but don't update state
            self._variance = self._variance * 1.5
            return FilteredPose(
                position=self._state.copy(),
                variance=self._variance.copy(),
                velocity=self._velocity.copy(),
                is_valid=True,
                timestamp=timestamp,
            )

        # Update velocity estimate
        if self._last_time is not None and timestamp > self._last_time:
            dt = timestamp - self._last_time
            new_velocity = (measurement - self._state) / dt
            self._velocity = (
                self._alpha * new_velocity + (1 - self._alpha) * self._velocity
            )

        # Exponential smoothing update
        self._state = self._alpha * measurement + (1 - self._alpha) * self._state

        # Update variance (blend towards measurement variance)
        self._variance = (
            self._alpha * measurement_variance + (1 - self._alpha) * self._variance
        )

        self._last_time = timestamp

        return FilteredPose(
            position=self._state.copy(),
            variance=self._variance.copy(),
            velocity=self._velocity.copy(),
            is_valid=True,
            timestamp=timestamp,
        )

    def _is_outlier(self, measurement: NDArray[np.float64]) -> bool:
        """Check if measurement is an outlier using Mahalanobis distance."""
        if self._state is None or self._variance is None:
            return False

        diff = measurement - self._state

        # Simplified Mahalanobis using diagonal covariance
        # d^2 = sum((diff_i)^2 / var_i)
        var_safe = np.maximum(self._variance, 1e-6)
        mahal_sq = np.sum(diff**2 / var_safe)

        return float(np.sqrt(mahal_sq)) > self._outlier_threshold

    def reset(self) -> None:
        """Reset filter state."""
        self._state = None
        self._variance = None
        self._last_time = None
        self._velocity = np.zeros(3, dtype=np.float64)

    @property
    def state(self) -> Optional[NDArray[np.float64]]:
        """Current filter state."""
        return self._state.copy() if self._state is not None else None

    @property
    def variance(self) -> Optional[NDArray[np.float64]]:
        """Current state variance."""
        return self._variance.copy() if self._variance is not None else None

    @property
    def is_initialized(self) -> bool:
        """Check if filter is initialized."""
        return self._state is not None


class KalmanFilter3D:
    """
    Kalman filter for 3D position and velocity estimation.

    State: [x, y, z, vx, vy, vz]
    Measurement: [x, y, z]
    """

    def __init__(
        self,
        process_noise: float = 0.1,
        measurement_noise: float = 0.5,
        dt: float = 0.05,
    ) -> None:
        """
        Initialize Kalman filter.

        Args:
            process_noise: Process noise covariance.
            measurement_noise: Measurement noise covariance.
            dt: Time step (default 20 Hz).
        """
        self._dt = dt

        # State: [x, y, z, vx, vy, vz]
        self._x = np.zeros(6, dtype=np.float64)

        # State covariance
        self._P = np.eye(6, dtype=np.float64) * 1.0

        # State transition matrix
        self._F = np.eye(6, dtype=np.float64)
        self._F[0, 3] = dt
        self._F[1, 4] = dt
        self._F[2, 5] = dt

        # Measurement matrix (we measure position only)
        self._H = np.zeros((3, 6), dtype=np.float64)
        self._H[0, 0] = 1
        self._H[1, 1] = 1
        self._H[2, 2] = 1

        # Process noise
        self._Q = np.eye(6, dtype=np.float64) * process_noise

        # Measurement noise
        self._R = np.eye(3, dtype=np.float64) * measurement_noise

        self._initialized = False

    def update(
        self,
        measurement: NDArray[np.float64],
        dt: Optional[float] = None,
    ) -> FilteredPose:
        """
        Update filter with new measurement.

        Args:
            measurement: Position measurement (x, y, z).
            dt: Optional time step override.

        Returns:
            FilteredPose with updated state.
        """
        measurement = np.asarray(measurement, dtype=np.float64).flatten()

        if not self._initialized:
            self._x[:3] = measurement
            self._initialized = True
            return FilteredPose(
                position=self._x[:3].copy(),
                variance=np.diag(self._P[:3, :3]).copy(),
                velocity=self._x[3:6].copy(),
                is_valid=True,
            )

        # Update state transition matrix if dt changed
        if dt is not None and dt != self._dt:
            self._dt = dt
            self._F[0, 3] = dt
            self._F[1, 4] = dt
            self._F[2, 5] = dt

        # Predict
        self._x = self._F @ self._x
        self._P = self._F @ self._P @ self._F.T + self._Q

        # Update
        y = measurement - self._H @ self._x  # Innovation
        S = self._H @ self._P @ self._H.T + self._R  # Innovation covariance
        K = self._P @ self._H.T @ np.linalg.inv(S)  # Kalman gain

        self._x = self._x + K @ y
        self._P = (np.eye(6) - K @ self._H) @ self._P

        return FilteredPose(
            position=self._x[:3].copy(),
            variance=np.diag(self._P[:3, :3]).copy(),
            velocity=self._x[3:6].copy(),
            is_valid=True,
        )

    def reset(self) -> None:
        """Reset filter state."""
        self._x = np.zeros(6, dtype=np.float64)
        self._P = np.eye(6, dtype=np.float64) * 1.0
        self._initialized = False

    @property
    def is_initialized(self) -> bool:
        """Check if filter is initialized."""
        return self._initialized
