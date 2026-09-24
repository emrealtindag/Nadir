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
