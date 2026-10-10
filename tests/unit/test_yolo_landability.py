import pytest
import numpy as np
from unittest.mock import MagicMock
from nadir.perception.camera import Frame
from nadir.perception.landability.yolo_plugin import YOLOLandabilityPlugin
from nadir.control.fsm import LandingFSM, SystemInputs, LandingState


def _create_mock_box(xyxy: list[float], conf: float = 0.85, cls_id: int = 0):
    box = MagicMock()
    box.conf = [conf]
    box.cls = [MagicMock(item=lambda: cls_id)]
    box.xyxy = [xyxy]
    return box


def test_yolo_spatial_clearance_when_vehicle_is_outside_pad():
    """Vehicle (Tasit, cls=0) is detected on runway but outside landing pad perimeter."""
    plugin = YOLOLandabilityPlugin(confidence_threshold=0.35)
    plugin._initialized = True
    plugin._class_names = {0: "Tasit", 1: "Insan", 2: "UAP", 3: "UAI"}
    
    # Runway vehicle at x: [50, 150], y: [50, 150]
    car_box = _create_mock_box([50.0, 50.0, 150.0, 150.0], conf=0.88, cls_id=0)
    mock_res = MagicMock()
    mock_res.boxes = [car_box]
    plugin._model = MagicMock(return_value=[mock_res])

    # Landing pad at center (320, 240) with size (100, 100) -> [270, 190, 370, 290]
    frame = Frame(
        image_bgr=np.zeros((480, 640, 3), dtype=np.uint8),
        timestamp_s=0.0,
        frame_id=1,
    )
    result = plugin.estimate(frame, roi_center=(320, 240), roi_size=(100, 100))

    assert result.score == 1.0
    assert "spatial_clearance_verified" in result.flags
    assert "obstacle_in_landing_zone" not in result.flags
    assert "vehicle_present" not in result.flags


def test_yolo_vehicle_in_pad_triggers_low_landability_abort():
    """Vehicle (Tasit, cls=0) directly intersects the landing pad perimeter."""
    plugin = YOLOLandabilityPlugin(confidence_threshold=0.35)
    plugin._initialized = True
    plugin._class_names = {0: "Tasit", 1: "Insan", 2: "UAP", 3: "UAI"}

    # Vehicle overlapping landing pad [270, 190, 370, 290]
    car_box = _create_mock_box([280.0, 200.0, 350.0, 280.0], conf=0.92, cls_id=0)
    mock_res = MagicMock()
    mock_res.boxes = [car_box]
    plugin._model = MagicMock(return_value=[mock_res])

    frame = Frame(
        image_bgr=np.zeros((480, 640, 3), dtype=np.uint8),
        timestamp_s=0.0,
        frame_id=1,
    )
    result = plugin.estimate(frame, roi_center=(320, 240), roi_size=(100, 100))

    assert result.score == 0.20
    assert "vehicle_present" in result.flags
    assert "obstacle_in_landing_zone" in result.flags

    # Feeding this result into FSM must trigger ABORT due to low landability
    fsm = LandingFSM()
    fsm.tick(SystemInputs(mavlink_connected=True, camera_connected=True, arm_command=True))
    fsm._state = LandingState.DESCEND

    fsm_out = fsm.tick(
        SystemInputs(
            mavlink_connected=True,
            camera_connected=True,
            target_visible=True,
            landability_score=result.score,
        )
    )
    assert fsm_out.state == LandingState.ABORT
    assert fsm_out.abort_reason == "low_landability"


def test_yolo_human_in_pad_triggers_human_abort():
    """Human (Insan, cls=1) intersects the landing pad perimeter -> instant zero tolerance abort."""
    plugin = YOLOLandabilityPlugin(confidence_threshold=0.35)
    plugin._initialized = True
    plugin._class_names = {0: "Tasit", 1: "Insan", 2: "UAP", 3: "UAI"}

    # Human overlapping landing pad [270, 190, 370, 290]
    human_box = _create_mock_box([290.0, 210.0, 330.0, 270.0], conf=0.95, cls_id=1)
    mock_res = MagicMock()
    mock_res.boxes = [human_box]
    plugin._model = MagicMock(return_value=[mock_res])

    frame = Frame(
        image_bgr=np.zeros((480, 640, 3), dtype=np.uint8),
        timestamp_s=0.0,
        frame_id=1,
    )
    result = plugin.estimate(frame, roi_center=(320, 240), roi_size=(100, 100))

    assert result.score == 0.0
    assert "human_present" in result.flags
    assert "obstacle_in_landing_zone" in result.flags

    # Feeding human_present into FSM must trigger instant ABORT with reason human_present
    fsm = LandingFSM()
    fsm.tick(SystemInputs(mavlink_connected=True, camera_connected=True, arm_command=True))
    fsm._state = LandingState.DESCEND

    fsm_out = fsm.tick(
        SystemInputs(
            mavlink_connected=True,
            camera_connected=True,
            target_visible=True,
            landability_score=result.score,
            human_present=("human_present" in result.flags),
        )
    )
    assert fsm_out.state == LandingState.ABORT
    assert fsm_out.abort_reason == "human_present"
