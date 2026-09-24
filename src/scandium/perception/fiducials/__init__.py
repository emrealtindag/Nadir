"""Fiducial detection module for Scandium."""

from scandium.perception.fiducials.base import FiducialDetection, IFiducialDetector
from scandium.perception.fiducials.aruco_detector import ArUcoDetector
from scandium.perception.fiducials.apriltag_detector import AprilTagDetector

__all__ = [
    "FiducialDetection",
    "IFiducialDetector",
    "ArUcoDetector",
    "AprilTagDetector",
]
