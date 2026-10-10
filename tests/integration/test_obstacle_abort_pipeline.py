import numpy as np
from unittest.mock import MagicMock
from nadir.perception.camera import Frame
from nadir.perception.landability.yolo_plugin import YOLOLandabilityPlugin
from nadir.control.fsm import LandingFSM, SystemInputs, LandingState


def _create_mock_box(xyxy: list[float], conf: float = 0.90, cls_id: int = 0):
    box = MagicMock()
    box.conf = [conf]
    box.cls = [MagicMock(item=lambda: cls_id)]
    box.xyxy = [xyxy]
    return box


def test_vehicle_obstacle_on_pad_triggers_abort_pipeline():
    """Validates end-to-end safety abort when a vehicle (Tasit, cls=0) encroaches on the landing pad."""
    plugin = YOLOLandabilityPlugin(confidence_threshold=0.35)
    plugin._initialized = True
    plugin._class_names = {0: "Tasit", 1: "Insan"}

    # Landing pad ROI is at (320, 240) with size (100, 100) -> [270, 190, 370, 290]
    # Vehicle box directly encroaches the pad at [280, 200, 350, 280]
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

    # 1. Perception Layer Verification
    assert result.score == 0.20
    assert "obstacle_in_landing_zone" in result.flags
    assert result.debug["obstacle_in_zone"] is True

    # 2. Control FSM Layer Verification
    fsm = LandingFSM(abort_landability=0.40)
    # Transition: INIT -> IDLE -> ACQUIRE -> ALIGN
    fsm.tick(SystemInputs(mavlink_connected=True, camera_connected=True))
    fsm.tick(SystemInputs(mavlink_connected=True, camera_connected=True, arm_command=True, target_visible=True))
    fsm.tick(SystemInputs(mavlink_connected=True, camera_connected=True, target_visible=True, variance=0.01))
    assert fsm.state == LandingState.ALIGN

    # Tick ALIGN with low landability score (0.20 < 0.40)
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
