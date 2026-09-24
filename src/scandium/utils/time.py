"""
Time utilities for Scandium.

Provides timing, rate limiting, and timestamp functions.
"""

import time
from contextlib import contextmanager
from typing import Iterator


def get_timestamp_s() -> float:
    """
    Get current timestamp in seconds (UTC epoch).

    Returns:
        Seconds since Unix epoch.
    """
    return time.time()


def get_timestamp_us() -> int:
    """
    Get current timestamp in microseconds (UTC epoch).

    Returns:
        Microseconds since Unix epoch.
    """
    return int(time.time() * 1_000_000)


class Timer:
    """
    Context manager for timing code blocks.

    Example:
        with Timer() as t:
            do_work()
        print(f"Elapsed: {t.elapsed_ms:.2f} ms")
    """

    def __init__(self) -> None:
        self._start: float = 0.0
        self._end: float = 0.0

    def __enter__(self) -> "Timer":
        self._start = time.perf_counter()
        return self

    def __exit__(self, *args: object) -> None:
        self._end = time.perf_counter()

    @property
    def elapsed_s(self) -> float:
        """Elapsed time in seconds."""
        if self._end == 0.0:
            return time.perf_counter() - self._start
        return self._end - self._start

    @property
    def elapsed_ms(self) -> float:
        """Elapsed time in milliseconds."""
        return self.elapsed_s * 1000.0


@contextmanager
def rate_limit(interval_s: float) -> Iterator[None]:
    """
    Context manager that ensures minimum time between iterations.

    Sleeps at the end of the block if the block executed faster than interval_s.

    Args:
        interval_s: Minimum interval in seconds.

    Example:
        while running:
            with rate_limit(0.05):  # 20 Hz
                process_frame()
    """
    start = time.perf_counter()
    yield
    elapsed = time.perf_counter() - start
    if elapsed < interval_s:
        time.sleep(interval_s - elapsed)


def monotonic_s() -> float:
    """
    Get monotonic time in seconds.

    Use for measuring elapsed time; not affected by system clock changes.

    Returns:
        Monotonic seconds.
    """
    return time.monotonic()
