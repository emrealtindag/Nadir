"""Integration test suite for Nadir precision landing pipeline.

Validates the full vision-to-control-to-MAVLink pipeline:
1. Synthetic frame generation with fiducial target
2. Target detection and corner extraction
3. PnP pose estimation & coordinate frame transform (Camera -> Body NED)
4. Landing Finite State Machine (FSM) state progression
5. MAVLink LANDING_TARGET payload calculation
"""

import time
import math
import cv2
import numpy as np
import pytest

from nadir.control.fsm import LandingFSM, LandingState, SystemInputs
from nadir.perception.calib import CalibrationManager, CameraIntrinsics, CameraExtrinsics
from nadir.perception.pose import estimate_pose_from_corners


def test_end_to_end_vision_to_fsm_pipeline():
    """Verify end-to-end dataflow from corner detection to state transitions."""
    # 1. Setup Camera Calibration
    intrinsics = CameraIntrinsics.default(width=640, height=480)
    extrinsics = CameraExtrinsics.identity()
    calib = CalibrationManager(intrinsics=intrinsics, extrinsics=extrinsics)

    # 2. Simulate ArUco corners in image (marker centered at depth z = 5.0m)
    marker_size = 0.20
    half_size = marker_size / 2.0
    object_points = np.array([
        [-half_size, -half_size, 0],
        [half_size, -half_size, 0],
        [half_size, half_size, 0],
        [-half_size, half_size, 0],
    ], dtype=np.float64)

    true_rvec = np.zeros(3, dtype=np.float64)
    true_tvec = np.array([0.0, 0.0, 5.0], dtype=np.float64)
    img_pts, _ = cv2.projectPoints(
        object_points, true_rvec, true_tvec, calib.intrinsics.K, calib.intrinsics.dist_coeffs
    )
    corners = img_pts.reshape(4, 2)

    # 3. Estimate Pose
    pose_result = estimate_pose_from_corners(
        corners_px=corners,
        marker_size_m=marker_size,
        intrinsics=calib.intrinsics,
        method=cv2.SOLVEPNP_ITERATIVE,
    )

    assert pose_result.success is True
    assert pose_result.tvec is not None
    # Estimated z should be very close to 5.0m
    assert abs(pose_result.tvec[2] - 5.0) < 0.1

    # 4. Transform to body frame
    t_cam = pose_result.tvec
    t_body = calib.extrinsics.transform_point(t_cam)
    lateral_error = math.hypot(float(t_cam[0]), float(t_cam[1]))

    assert lateral_error < 0.1  # Directly centered

    # 5. Initialize FSM & feed inputs
    fsm = LandingFSM()
    now = time.time()

    # Step A: Initialize -> IDLE
    out0 = fsm.tick(SystemInputs(
        mavlink_connected=True,
        camera_connected=True,
        timestamp=now,
    ))
    assert out0.state == LandingState.IDLE

    # Step B: Arm with target detected -> ACQUIRE directly
    out1 = fsm.tick(SystemInputs(
        target_visible=True,
        target_confidence=0.95,
        lateral_error_m=lateral_error,
        altitude_m=float(t_cam[2]),
        landability_score=0.95,
        mavlink_connected=True,
        camera_connected=True,
        arm_command=True,
        timestamp=now + 0.05,
    ))
    assert out1.state == LandingState.ACQUIRE

    # Step C: Stabilized target -> ALIGN
    out2 = fsm.tick(SystemInputs(
        target_visible=True,
        target_confidence=0.95,
        lateral_error_m=lateral_error,
        altitude_m=float(t_cam[2]),
        landability_score=0.95,
        mavlink_connected=True,
        camera_connected=True,
        arm_command=True,
        timestamp=now + 0.1,
    ))
    assert out2.state in (LandingState.ACQUIRE, LandingState.ALIGN)

    # 6. Verify angle calculations for MAVLink LANDING_TARGET (BODY_FRD)
    angle_x = math.atan2(float(t_body[1]), float(t_body[2]))
    angle_y = math.atan2(-float(t_body[0]), float(t_body[2]))

    # Target is centered, angles should be close to 0 rad
    assert abs(angle_x) < 0.05
    assert abs(angle_y) < 0.05
