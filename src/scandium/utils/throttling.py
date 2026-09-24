"""
Rate limiting and throttling utilities for Scandium.

Provides tools for limiting execution rates and managing timing.
"""

import time
from threading import Lock


class RateLimiter:
    """
    Rate limiter for controlling execution frequency.

    Thread-safe implementation using monotonic clock.

    Example:
        limiter = RateLimiter(rate_hz=20)  # 20 Hz

        while running:
            if limiter.should_run():
                publish_message()
    """

    def __init__(self, rate_hz: float) -> None:
        """
        Initialize rate limiter.

        Args:
            rate_hz: Target rate in Hz (executions per second).
        """
        if rate_hz <= 0:
            raise ValueError("rate_hz must be positive")

        self._interval_s = 1.0 / rate_hz
        self._last_time: float = 0.0
        self._lock = Lock()

    @property
    def interval_s(self) -> float:
        """Minimum interval between executions in seconds."""
        return self._interval_s

    @property
    def rate_hz(self) -> float:
        """Target rate in Hz."""
        return 1.0 / self._interval_s

    def should_run(self) -> bool:
        """
        Check if enough time has passed since last execution.

        Returns:
            True if rate limit allows execution, False otherwise.
        """
        current_time = time.monotonic()

        with self._lock:
            if current_time - self._last_time >= self._interval_s:
                self._last_time = current_time
                return True
            return False

    def wait_and_run(self) -> None:
        """
        Wait until rate limit allows execution, then mark as executed.

        Blocks if called too frequently.
        """
        with self._lock:
            current_time = time.monotonic()
            elapsed = current_time - self._last_time

            if elapsed < self._interval_s:
                time.sleep(self._interval_s - elapsed)

            self._last_time = time.monotonic()

    def reset(self) -> None:
        """Reset the rate limiter, allowing immediate execution."""
        with self._lock:
            self._last_time = 0.0

    def time_until_next(self) -> float:
        """
        Get time until next allowed execution.

        Returns:
            Seconds until next execution is allowed. Zero if allowed now.
        """
        with self._lock:
            elapsed = time.monotonic() - self._last_time
            remaining = self._interval_s - elapsed
            return max(0.0, remaining)


class BurstLimiter:
    """
    Burst-aware rate limiter allowing temporary bursts above base rate.

    Allows up to `burst` executions immediately, then enforces rate limit.
    Burst capacity regenerates over time.
    """

    def __init__(self, rate_hz: float, burst: int = 1) -> None:
        """
        Initialize burst limiter.

        Args:
            rate_hz: Base rate in Hz.
            burst: Maximum burst size.
        """
        if rate_hz <= 0:
            raise ValueError("rate_hz must be positive")
        if burst < 1:
            raise ValueError("burst must be at least 1")

        self._interval_s = 1.0 / rate_hz
        self._burst = burst
        self._tokens = float(burst)
        self._last_time = time.monotonic()
        self._lock = Lock()

    def should_run(self) -> bool:
        """
        Check if execution is allowed.

        Returns:
            True if allowed, False otherwise.
        """
        with self._lock:
            self._regenerate_tokens()

            if self._tokens >= 1.0:
                self._tokens -= 1.0
                return True
            return False

    def _regenerate_tokens(self) -> None:
        """Regenerate tokens based on elapsed time."""
        current = time.monotonic()
        elapsed = current - self._last_time
        self._last_time = current

        # Add tokens based on elapsed time
        tokens_to_add = elapsed / self._interval_s
        self._tokens = min(self._burst, self._tokens + tokens_to_add)
