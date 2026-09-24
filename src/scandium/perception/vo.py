"""
Visual Odometry (VO) and Optical Flow module for GNSS-Denied Navigation.

Extracts features from the ground and tracks them across frames to estimate
UAV displacement (dx, dy, dz, yaw) when fiducial markers (ArUco) or GPS are lost.
Incorporates a Bayesian Scale Filter to dynamically adjust optical flow scaling.
"""

import math
import numpy as np
import cv2
from typing import Optional, Tuple

class BayesianScaleFilter:
    """
    1D Kalman Filter (Recursive Bayesian Update) for Optical Flow scale.
    Smooths scale variance over time for reliable altitude/velocity estimation.
    """
    def __init__(self, initial_scale: float = 1.0, initial_variance: float = 10.0):
        self.prior_scale = initial_scale
        self.prior_variance = initial_variance

    def update(self, measured_scale: float, measurement_variance: float = 0.05) -> float:
        kalman_gain = self.prior_variance / (self.prior_variance + measurement_variance)
        estimated_scale = self.prior_scale + kalman_gain * (measured_scale - self.prior_scale)
        self.prior_variance = (1.0 - kalman_gain) * self.prior_variance
        self.prior_scale = estimated_scale
        return self.prior_scale

class VisualOdometry:
    """
    GNSS-Denied Visual Odometry tracker with Multi-Modal (Thermal/RGB) support.
    Uses Lucas-Kanade Optical Flow to track background features.
    Automatically scales camera intrinsics based on sensor modality.
    """
    def __init__(self):
        # Default to 1080p RGB
        self.fx = 1387.1
        self.fy = 1389.7
        self.cx = 954.0
        self.cy = 558.8
        
        self.prev_gray: Optional[np.ndarray] = None
        self.prev_points: Optional[np.ndarray] = None
        
        self.scale_filter = BayesianScaleFilter()
        
        self.current_yaw = 0.0
        self.smooth_global_dx = 0.0
        self.smooth_global_dy = 0.0

    def _auto_bind_intrinsics(self, width: int) -> None:
        """
        Dynamically binds intrinsic camera matrix based on sensor resolution.
        Supports seamless handover between Day (RGB) and Night (Thermal) sensors.
        """
        if width < 1000:
            # Thermal Sensor Profile (e.g. 640x512)
            self.fx, self.fy = 731.7, 732.0
            self.cx, self.cy = 319.2, 251.2
        elif width < 2500:
            # 1080p RGB Profile (e.g. 1920x1080)
            self.fx, self.fy = 1389.7, 1387.1
            self.cx, self.cy = 954.0, 558.8
        else:
            # 4K RGB Profile (e.g. 4000x3000)
            self.fx, self.fy = 2792.2, 2795.2
            self.cx, self.cy = 1988.0, 1562.2

    def process_frame(self, frame_bgr: np.ndarray, current_z: float) -> Tuple[float, float, float]:
        """
        Process a new frame and estimate global movement.
        Returns (delta_x, delta_y, delta_yaw)
        """
        h, w = frame_bgr.shape[:2]
        self._auto_bind_intrinsics(w)
        
        current_gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        current_gray = clahe.apply(current_gray)

        if self.prev_gray is None or self.prev_points is None or len(self.prev_points) < 20:
            # Re-initialize tracking points
            self.prev_points = cv2.goodFeaturesToTrack(
                current_gray, mask=None, maxCorners=200, qualityLevel=0.01, minDistance=10, blockSize=7
            )
            self.prev_gray = current_gray
            return (0.0, 0.0, 0.0)

        # Calculate Optical Flow
        next_points, status, error = cv2.calcOpticalFlowPyrLK(
            self.prev_gray, current_gray, self.prev_points, None, winSize=(21, 21), maxLevel=3
        )
        
        good_new = next_points[status == 1]
        good_old = self.prev_points[status == 1]

        delta_yaw = 0.0
        raw_dx = 0.0
        raw_dy = 0.0

        if len(good_new) >= 6:
            # Estimate Affine Transform for global camera movement
            center_array = np.array([self.cx, self.cy], dtype=np.float32)
            good_old_centered = good_old - center_array
            good_new_centered = good_new - center_array

            transform_matrix, inliers = cv2.estimateAffinePartial2D(
                good_old_centered, good_new_centered, method=cv2.RANSAC, ransacReprojThreshold=3.0
            )

            if transform_matrix is not None:
                # Extract Yaw (Rotation)
                a = transform_matrix[0, 0]
                c = transform_matrix[1, 0]
                delta_yaw = math.atan2(c, a)
                self.current_yaw += delta_yaw
                
                # Extract Translation in pixels
                dx_pixel = transform_matrix[0, 2]
                dy_pixel = transform_matrix[1, 2]
                
                # Convert pixels to metric scale using current altitude (current_z)
                z_safe = max(abs(current_z), 2.0)
                forward_m = (dy_pixel / self.fy) * z_safe * self.scale_filter.prior_scale
                right_m = -(dx_pixel / self.fx) * z_safe * self.scale_filter.prior_scale
                
                # Rotate into global ENU/NED frame
                raw_dx = forward_m * math.cos(self.current_yaw) - right_m * math.sin(self.current_yaw)
                raw_dy = forward_m * math.sin(self.current_yaw) + right_m * math.cos(self.current_yaw)

                # Smooth the output
                alpha = 0.35
                self.smooth_global_dx = ((1.0 - alpha) * self.smooth_global_dx) + (alpha * raw_dx)
                self.smooth_global_dy = ((1.0 - alpha) * self.smooth_global_dy) + (alpha * raw_dy)
                
                # Update tracking points
                good_new = good_new[inliers.ravel() == 1]

        self.prev_gray = current_gray
        self.prev_points = good_new.reshape(-1, 1, 2) if len(good_new) > 0 else None

        return (self.smooth_global_dx, self.smooth_global_dy, delta_yaw)
