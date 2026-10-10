import pytest
from nadir.control.fsm import LandingFSM, SystemInputs, LandingState

def test_fsm_initial_state():
    fsm = LandingFSM()
    assert fsm.state == LandingState.INIT

def test_fsm_init_to_idle():
    fsm = LandingFSM()
    inputs = SystemInputs(mavlink_connected=True, camera_connected=True)
    out = fsm.tick(inputs)
    assert out.state == LandingState.IDLE

def test_fsm_human_presence_aborts():
    fsm = LandingFSM()
    
    # Move to IDLE
    fsm.tick(SystemInputs(mavlink_connected=True, camera_connected=True))
    
    # Arm -> SEARCH
    fsm.tick(SystemInputs(mavlink_connected=True, camera_connected=True, arm_command=True))
    assert fsm.state == LandingState.SEARCH
    
    # Human appears -> ABORT
    out = fsm.tick(SystemInputs(mavlink_connected=True, camera_connected=True, human_present=True))
    assert out.state == LandingState.ABORT
    assert out.abort_reason == "human_present"

def test_fsm_failsafe_on_camera_loss():
    fsm = LandingFSM()
    # Move to IDLE
    fsm.tick(SystemInputs(mavlink_connected=True, camera_connected=True))
    
    # Camera disconnects -> FAILSAFE
    out = fsm.tick(SystemInputs(mavlink_connected=True, camera_connected=False))
    assert out.state == LandingState.FAILSAFE
    assert out.abort_reason == "link_lost"


def test_fsm_init_waits_until_hardware_connected():
    fsm = LandingFSM()
    # Still waiting for camera on startup -> stays in INIT
    out = fsm.tick(SystemInputs(mavlink_connected=True, camera_connected=False))
    assert out.state == LandingState.INIT

    # Connections established -> transitions to IDLE
    out = fsm.tick(SystemInputs(mavlink_connected=True, camera_connected=True))
    assert out.state == LandingState.IDLE


def test_fsm_target_lost_timer_resets_on_reacquisition():
    fsm = LandingFSM(target_lost_timeout_s=2.0)
    # Transition: INIT -> IDLE -> ACQUIRE -> ALIGN
    fsm.tick(SystemInputs(mavlink_connected=True, camera_connected=True))
    fsm.tick(SystemInputs(mavlink_connected=True, camera_connected=True, arm_command=True, target_visible=True))
    fsm.tick(SystemInputs(mavlink_connected=True, camera_connected=True, target_visible=True, variance=0.01))
    assert fsm.state == LandingState.ALIGN

    # Temporary flicker at t=1.0
    fsm.tick(SystemInputs(mavlink_connected=True, camera_connected=True, target_visible=False, timestamp=1.0))
    assert fsm.state == LandingState.ALIGN

    # Target re-acquired at t=1.2 -> timer must be cleared
    fsm.tick(SystemInputs(mavlink_connected=True, camera_connected=True, target_visible=True, timestamp=1.2))
    assert fsm.state == LandingState.ALIGN

    # Target flickers again at t=10.0 for 0.5s -> should NOT abort because elapsed is 0.5s < 2.0s
    fsm.tick(SystemInputs(mavlink_connected=True, camera_connected=True, target_visible=False, timestamp=10.0))
    fsm.tick(SystemInputs(mavlink_connected=True, camera_connected=True, target_visible=False, timestamp=10.5))
    assert fsm.state == LandingState.ALIGN

    # Target lost exceeds 2.0s -> aborts
    out = fsm.tick(SystemInputs(mavlink_connected=True, camera_connected=True, target_visible=False, timestamp=12.1))
    assert out.state == LandingState.ABORT
    assert out.abort_reason == "target_lost_timeout"
