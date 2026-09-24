#!/usr/bin/env python3
"""
Latency Benchmark Utility for Scandium.

Measures end-to-end pipeline latency including image acquisition,
fiducial detection, pose estimation, and MAVLink message generation.

Usage:
    python scripts/benchmark_latency.py --config configs/default.yaml --iterations 1000
"""

import argparse
from pathlib import Path
import statistics
import sys
import time
from typing import List

import cv2
import numpy as np


def generate_synthetic_frame(
    width: int = 1280,
    height: int = 720,
    with_marker: bool = True,
) -> np.ndarray:
    """
    Generate synthetic frame with optional ArUco marker.

    Args:
        width: Frame width.
        height: Frame height.
        with_marker: Whether to draw an ArUco marker.

    Returns:
        Synthetic BGR frame.
    """
    # Create gray background
    frame = np.ones((height, width, 3), dtype=np.uint8) * 128

    if with_marker:
        # Generate ArUco marker
        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100)
        marker_size = 200
        marker_img = cv2.aruco.generateImageMarker(aruco_dict, 1, marker_size)

        # Convert to BGR
        marker_bgr = cv2.cvtColor(marker_img, cv2.COLOR_GRAY2BGR)

        # Place marker in center
        x_offset = (width - marker_size) // 2
        y_offset = (height - marker_size) // 2
        frame[y_offset : y_offset + marker_size, x_offset : x_offset + marker_size] = (
            marker_bgr
        )

    return frame


def benchmark_detection(
    iterations: int,
    frame: np.ndarray,
    detector,
) -> List[float]:
    """
    Benchmark fiducial detection latency.

    Args:
        iterations: Number of iterations.
        frame: Input frame.
        detector: Fiducial detector instance.

    Returns:
        List of latency measurements in milliseconds.
    """
    latencies = []
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Warmup
    for _ in range(10):
        detector.detect(gray)

    # Benchmark
    for _ in range(iterations):
        start = time.perf_counter()
        detector.detect(gray)
        end = time.perf_counter()
        latencies.append((end - start) * 1000)

    return latencies


def benchmark_pose_estimation(
    iterations: int,
    corners: np.ndarray,
    marker_size: float,
    camera_matrix: np.ndarray,
    dist_coeffs: np.ndarray,
) -> List[float]:
    """
    Benchmark pose estimation latency.

    Args:
        iterations: Number of iterations.
        corners: Marker corners.
        marker_size: Marker size in meters.
        camera_matrix: Camera intrinsic matrix.
        dist_coeffs: Distortion coefficients.

    Returns:
        List of latency measurements in milliseconds.
    """
    latencies = []

    # Define object points
    half_size = marker_size / 2.0
    object_points = np.array(
        [
            [-half_size, -half_size, 0],
            [half_size, -half_size, 0],
            [half_size, half_size, 0],
            [-half_size, half_size, 0],
        ],
        dtype=np.float64,
    ).reshape(-1, 1, 3)

    image_points = corners.reshape(-1, 1, 2).astype(np.float64)

    # Warmup
    for _ in range(10):
        cv2.solvePnP(
            object_points,
            image_points,
            camera_matrix,
            dist_coeffs,
            flags=cv2.SOLVEPNP_IPPE_SQUARE,
        )

    # Benchmark
    for _ in range(iterations):
        start = time.perf_counter()
        cv2.solvePnP(
            object_points,
            image_points,
            camera_matrix,
            dist_coeffs,
            flags=cv2.SOLVEPNP_IPPE_SQUARE,
        )
        end = time.perf_counter()
        latencies.append((end - start) * 1000)

    return latencies


def print_statistics(name: str, latencies: List[float]) -> None:
    """Print latency statistics."""
    if not latencies:
        print(f"{name}: No data")
        return

    print(f"\n{name}")
    print("-" * 50)
    print(f"  Samples:     {len(latencies)}")
    print(f"  Mean:        {statistics.mean(latencies):.3f} ms")
    print(f"  Median:      {statistics.median(latencies):.3f} ms")
    print(f"  Std Dev:     {statistics.stdev(latencies):.3f} ms")
    print(f"  Min:         {min(latencies):.3f} ms")
    print(f"  Max:         {max(latencies):.3f} ms")
    print(f"  P95:         {sorted(latencies)[int(len(latencies) * 0.95)]:.3f} ms")
    print(f"  P99:         {sorted(latencies)[int(len(latencies) * 0.99)]:.3f} ms")
    print(f"  Throughput:  {1000 / statistics.mean(latencies):.1f} Hz")


def main() -> int:
    """Main entry point."""
    parser = argparse.ArgumentParser(
        description="Latency benchmark utility for Scandium precision landing system.",
    )

    parser.add_argument(
        "--iterations",
        "-n",
        type=int,
        default=1000,
        help="Number of benchmark iterations",
    )

    parser.add_argument(
        "--width",
        type=int,
        default=1280,
        help="Frame width",
    )

    parser.add_argument(
        "--height",
        type=int,
        default=720,
        help="Frame height",
    )

    parser.add_argument(
        "--marker-size",
        type=float,
        default=0.2,
        help="Marker size in meters",
    )

    args = parser.parse_args()

    print("=" * 60)
    print("SCANDIUM LATENCY BENCHMARK")
    print("=" * 60)
    print(f"Iterations: {args.iterations}")
    print(f"Resolution: {args.width}x{args.height}")
    print(f"Marker size: {args.marker_size} m")

    # Generate synthetic frame
    print("\nGenerating synthetic frame with ArUco marker...")
    frame = generate_synthetic_frame(args.width, args.height, with_marker=True)

    # Camera parameters (approximate)
    fx = fy = args.width
    cx, cy = args.width / 2, args.height / 2
    camera_matrix = np.array(
        [
            [fx, 0, cx],
            [0, fy, cy],
            [0, 0, 1],
        ],
        dtype=np.float64,
    )
    dist_coeffs = np.zeros(5, dtype=np.float64)

    # Initialize ArUco detector
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100)
    aruco_params = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)

    # Benchmark detection
    print(f"\nRunning detection benchmark ({args.iterations} iterations)...")
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    detection_latencies = []
    corners_list, ids, _ = detector.detectMarkers(gray)

    for _ in range(10):  # Warmup
        detector.detectMarkers(gray)

    for _ in range(args.iterations):
        start = time.perf_counter()
        corners_list, ids, _ = detector.detectMarkers(gray)
        end = time.perf_counter()
        detection_latencies.append((end - start) * 1000)

    print_statistics("ArUco Detection", detection_latencies)

    # Benchmark pose estimation (if marker was detected)
    if len(corners_list) > 0:
        print(f"\nRunning pose estimation benchmark ({args.iterations} iterations)...")
        corners = corners_list[0].reshape(-1, 2)

        pose_latencies = benchmark_pose_estimation(
            args.iterations,
            corners,
            args.marker_size,
            camera_matrix,
            dist_coeffs,
        )

        print_statistics("Pose Estimation (solvePnP)", pose_latencies)

        # Combined pipeline
        combined_mean = statistics.mean(detection_latencies) + statistics.mean(
            pose_latencies
        )
        print(f"\n{'=' * 50}")
        print(f"COMBINED PIPELINE (Detection + Pose)")
        print(f"{'=' * 50}")
        print(f"  Mean latency:   {combined_mean:.3f} ms")
        print(f"  Max throughput: {1000 / combined_mean:.1f} Hz")
    else:
        print("\nWarning: Marker not detected, skipping pose benchmark")

    print("\n" + "=" * 60)
    print("BENCHMARK COMPLETE")
    print("=" * 60)

    return 0


if __name__ == "__main__":
    sys.exit(main())
