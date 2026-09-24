"""
Landing Finite State Machine for Scandium.

Implements deterministic state machine for precision landing sequence.
"""

from dataclasses import dataclass, field
from enum import Enum, auto
from typing import Optional, Callable
import time

from scandium.logging.setup import get_logger

logger = get_logger(__name__)


class LandingState(Enum):
    """Landing state machine states."""

    INIT = auto()
    IDLE = auto()
    SEARCH = auto()
    ACQUIRE = auto()
    ALIGN = auto()
    DESCEND = auto()
    TOUCHDOWN = auto()
    ABORT = auto()
    FAILSAFE = auto()


@dataclass
class SystemInputs:
    """
    Inputs to the landing FSM.

    Attributes:
        target_visible: Whether target is currently detected.
        target_confidence: Detection confidence [0, 1].
        lateral_error_m: Lateral distance to target in meters.
        altitude_m: Current altitude AGL in meters.
        landability_score: Landing zone safety score [0, 1].
        mavlink_connected: Whether MAVLink connection is alive.
        camera_connected: Whether camera is functional.
        arm_command: External arm/start command.
        abort_command: External abort command.
        timestamp: Current timestamp.
    """

    target_visible: bool = False
    target_confidence: float = 0.0
    lateral_error_m: float = float("inf")
    altitude_m: float = 100.0
    landability_score: float = 1.0
    mavlink_connected: bool = True
    camera_connected: bool = True
    arm_command: bool = False
    abort_command: bool = False
    timestamp: float = field(default_factory=time.time)
    human_present: bool = False
    variance: float = 0.0


@dataclass
class SystemOutputs:
    """
    Outputs from the landing FSM.

    Attributes:
        state: Current FSM state.
        publish_landing_target: Whether to publish LANDING_TARGET.
        abort_reason: Reason for abort (if applicable).
        confidence_gain: Gain scaling factor based on confidence.
        state_changed: Whether state changed this tick.
        abort_command: Suggest the higher-level system to run an abort (go-around).
        command: Optional textual command (e.g. "GO_AROUND").
        failsafe_active: Whether an active failsafe is engaged.
        mission_complete: Whether the landing task completed successfully.
    """

    state: LandingState
    publish_landing_target: bool = False
    abort_reason: Optional[str] = None
    confidence_gain: float = 1.0
    state_changed: bool = False
    abort_command: bool = False
    command: Optional[str] = None
    failsafe_active: bool = False
    mission_complete: bool = False


class LandingFSM:
    """
    Landing Finite State Machine.

    Manages state transitions for precision landing sequence.
    Designed to be deterministic: same inputs produce same outputs.
    """

    def __init__(
        self,
        acquire_confidence: float = 0.70,
        align_error_m: float = 0.25,
        abort_landability: float = 0.40,
        touchdown_altitude_m: float = 0.5,
        max_variance: float = 0.5,
        consecutive_frames_for_acquire: int = 5,
        target_lost_timeout_s: float = 2.0,
    ) -> None:
        """
        Initialize FSM.

        Args:
            acquire_confidence: Confidence threshold for ACQUIRE.
            align_error_m: Lateral error threshold for ALIGN.
            abort_landability: Landability threshold for ABORT.
            touchdown_altitude_m: Altitude for TOUCHDOWN.
            max_variance: Maximum variance before reducing gain.
            consecutive_frames_for_acquire: Frames needed to confirm acquire.
            target_lost_timeout_s: Timeout before returning to SEARCH.
        """
        self._state = LandingState.INIT
        self._prev_state = LandingState.INIT

        # Thresholds
        self._acquire_confidence = acquire_confidence
        self._align_error_m = align_error_m
        self._abort_landability = abort_landability
        self._touchdown_altitude_m = touchdown_altitude_m
        self._max_variance = max_variance
        self._consecutive_frames_for_acquire = consecutive_frames_for_acquire
        self._target_lost_timeout_s = target_lost_timeout_s

        # Counters
        self._consecutive_detections = 0
        self._last_target_time: Optional[float] = None
        self._abort_reason: Optional[str] = None

        # Action flags
        self._abort_command: bool = False
        self._abort_cmd_text: Optional[str] = None
        self._failsafe_active: bool = False
        self._mission_complete: bool = False

    def tick(self, inputs: SystemInputs) -> SystemOutputs:
        """
        Process one tick of the state machine.

        Args:
            inputs: Current system inputs.

        Returns:
            SystemOutputs with current state and actions.
        """
        self._prev_state = self._state

        # Check for critical failures first
        if not inputs.mavlink_connected or not inputs.camera_connected:
            self._transition_to(LandingState.FAILSAFE)
            self._abort_reason = "link_lost"
            return self._make_output()

        # External abort command
        if inputs.abort_command:
            self._transition_to(LandingState.ABORT)
            self._abort_reason = "external_command"
            return self._make_output()

        # Human present always triggers abort
        if inputs.human_present and self._state not in (
            LandingState.INIT,
            LandingState.IDLE,
            LandingState.ABORT,
            LandingState.FAILSAFE,
        ):
            self._transition_to(LandingState.ABORT)
            self._abort_reason = "human_present"
            return self._make_output()

        # State-specific logic
        if self._state == LandingState.INIT:
            self._handle_init(inputs)
        elif self._state == LandingState.IDLE:
            self._handle_idle(inputs)
        elif self._state == LandingState.SEARCH:
            self._handle_search(inputs)
        elif self._state == LandingState.ACQUIRE:
            self._handle_acquire(inputs)
        elif self._state == LandingState.ALIGN:
            self._handle_align(inputs)
        elif self._state == LandingState.DESCEND:
            self._handle_descend(inputs)
        elif self._state == LandingState.TOUCHDOWN:
            self._handle_touchdown(inputs)
        elif self._state == LandingState.ABORT:
            self._handle_abort(inputs)
        elif self._state == LandingState.FAILSAFE:
            self._handle_failsafe(inputs)

        return self._make_output()

    def _transition_to(self, new_state: LandingState) -> None:
        """Transition to new state with logging."""
        if new_state != self._state:
            logger.info(
                "fsm_transition",
                from_state=self._state.name,
                to_state=new_state.name,
            )
            self._state = new_state

    def _make_output(self) -> SystemOutputs:
        """Create output based on current state."""
        publish = self._state in (
            LandingState.ACQUIRE,
            LandingState.ALIGN,
            LandingState.DESCEND,
        )

        # Reset action flags unless state requires them
        abort_cmd = False
        cmd_text = None
        if self._state == LandingState.ABORT:
            abort_cmd = True
            cmd_text = self._abort_cmd_text or "GO_AROUND"

        failsafe = self._state == LandingState.FAILSAFE or self._failsafe_active
        mission_done = self._mission_complete

        return SystemOutputs(
            state=self._state,
            publish_landing_target=publish,
            abort_reason=self._abort_reason,
            confidence_gain=self._compute_gain(),
            state_changed=self._state != self._prev_state,
            abort_command=abort_cmd,
            command=cmd_text,
            failsafe_active=failsafe,
            mission_complete=mission_done,
        )

    def _compute_gain(self) -> float:
        """Compute confidence-based gain scaling."""
        # Reduce gain in uncertain states
        if self._state == LandingState.SEARCH:
            return 0.5
        if self._state == LandingState.ACQUIRE:
            return 0.7
        return 1.0

    def _handle_init(self, inputs: SystemInputs) -> None:
        """Handle INIT state."""
        # Wait for connections to be established
        if inputs.mavlink_connected and inputs.camera_connected:
            self._transition_to(LandingState.IDLE)

    def _handle_idle(self, inputs: SystemInputs) -> None:
        """Handle IDLE state."""
        if inputs.arm_command:
            if inputs.target_visible:
                self._transition_to(LandingState.ACQUIRE)
            else:
                self._transition_to(LandingState.SEARCH)

    def _handle_search(self, inputs: SystemInputs) -> None:
        """Handle SEARCH state."""
        if (
            inputs.target_visible
            and inputs.target_confidence >= self._acquire_confidence
        ):
            self._consecutive_detections += 1
            if self._consecutive_detections >= self._consecutive_frames_for_acquire:
                self._transition_to(LandingState.ACQUIRE)
                self._consecutive_detections = 0
        else:
            self._consecutive_detections = 0

    def _handle_acquire(self, inputs: SystemInputs) -> None:
        """Handle ACQUIRE state."""
        if not inputs.target_visible:
            self._transition_to(LandingState.SEARCH)
            return

        if inputs.landability_score < self._abort_landability:
            self._transition_to(LandingState.ABORT)
            self._abort_reason = "low_landability"
            return

        # Wait for filter to stabilize
        if inputs.variance < self._max_variance:
            self._transition_to(LandingState.ALIGN)

    def _handle_align(self, inputs: SystemInputs) -> None:
        """Handle ALIGN state."""
        if not inputs.target_visible:
            self._handle_target_lost(inputs)
            return

        if inputs.landability_score < self._abort_landability:
            self._transition_to(LandingState.ABORT)
            self._abort_reason = "low_landability"
            return

        if inputs.lateral_error_m <= self._align_error_m:
            self._transition_to(LandingState.DESCEND)

    def _handle_descend(self, inputs: SystemInputs) -> None:
        """Handle DESCEND state."""
        if not inputs.target_visible:
            self._handle_target_lost(inputs)
            return

        if inputs.landability_score < self._abort_landability:
            self._transition_to(LandingState.ABORT)
            self._abort_reason = "low_landability"
            return

        if inputs.altitude_m <= self._touchdown_altitude_m:
            self._transition_to(LandingState.TOUCHDOWN)

    def _handle_touchdown(self, inputs: SystemInputs) -> None:
        """Handle TOUCHDOWN state."""
        # Terminal state - landing complete
        logger.info("fsm_touchdown", msg="Touchdown detected; marking mission complete")
        self._mission_complete = True

    def _handle_abort(self, inputs: SystemInputs) -> None:
        """Handle ABORT state."""
        # Terminal state until reset
        logger.warning("fsm_abort", reason=self._abort_reason or "unknown")
        # Suggest go-around (fixed-wing appropriate behavior)
        self._abort_command = True
        self._abort_cmd_text = "GO_AROUND"

    def _handle_failsafe(self, inputs: SystemInputs) -> None:
        """Handle FAILSAFE state."""
        logger.critical("fsm_failsafe", reason=self._abort_reason or "failsafe_triggered")
        self._failsafe_active = True

    def _handle_target_lost(self, inputs: SystemInputs) -> None:
        """Handle temporary target loss."""
        if self._last_target_time is None:
            self._last_target_time = inputs.timestamp

        if inputs.timestamp - self._last_target_time > self._target_lost_timeout_s:
            # For fixed-wing, losing target for longer time should trigger abort/go-around
            self._transition_to(LandingState.ABORT)
            self._abort_reason = "target_lost_timeout"
            self._last_target_time = None

    @property
    def state(self) -> LandingState:
        """Current state."""
        return self._state

    def reset(self) -> None:
        """Reset FSM to initial state."""
        self._state = LandingState.INIT
        self._prev_state = LandingState.INIT
        self._consecutive_detections = 0
        self._last_target_time = None
        self._abort_reason = None
        self._abort_command = False
        self._abort_cmd_text = None
        self._failsafe_active = False
        self._mission_complete = False
