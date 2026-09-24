"""Fiducial detection module for Nadir."""

from nadir.perception.fiducials.base import FiducialDetection, IFiducialDetector
from nadir.perception.fiducials.aruco_detector import ArUcoDetector
from nadir.perception.fiducials.apriltag_detector import AprilTagDetector

__all__ = [
    "FiducialDetection",
    "IFiducialDetector",
    "ArUcoDetector",
    "AprilTagDetector",
]
