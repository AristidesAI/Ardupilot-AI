"""
Mission state machine for autonomous drone operations.

Manages mission states and transitions.
"""

import time
from dataclasses import dataclass, field
from enum import Enum, auto
from typing import Dict, List, Optional, Set, Callable

import structlog

logger = structlog.get_logger()


class MissionState(Enum):
    """Mission states."""
    # Initialization
    IDLE = auto()  # Not active
    PREFLIGHT = auto()  # System checks
    ARMING = auto()  # Arming in progress

    # Flight phases
    TAKEOFF = auto()  # Taking off
    CLIMBING = auto()  # Climbing to altitude
    TRANSIT = auto()  # Flying to location
    LOITER = auto()  # Holding position
    MISSION = auto()  # Executing mission
    SURVEY = auto()  # Survey pattern

    # Return phases
    RTL = auto()  # Return to launch
    DESCENDING = auto()  # Descending for landing
    LANDING = auto()  # Landing in progress
    LANDED = auto()  # On ground

    # Emergency
    EMERGENCY = auto()  # Emergency situation
    LOST_LINK = auto()  # Communication lost


@dataclass
class StateTransition:
    """State transition record."""
    from_state: MissionState
    to_state: MissionState
    reason: str
    timestamp: float


class MissionStateMachine:
    """
    State machine for mission management.

    Manages valid state transitions and callbacks.
    """

    def __init__(self):
        """Initialize state machine."""
        self._state = MissionState.IDLE
        self._entry_time = time.time()
        self._history: List[StateTransition] = []

        # Callbacks
        self._on_enter: Dict[MissionState, List[Callable]] = {}
        self._on_exit: Dict[MissionState, List[Callable]] = {}

        # Valid transitions
        self._transitions = self._define_transitions()

    @property
    def state(self) -> MissionState:
        """Get current state."""
        return self._state

    @property
    def time_in_state(self) -> float:
        """Get time in current state."""
        return time.time() - self._entry_time

    @property
    def history(self) -> List[StateTransition]:
        """Get state history."""
        return self._history

    def _define_transitions(self) -> Dict[MissionState, Set[MissionState]]:
        """Define valid state transitions."""
        return {
            MissionState.IDLE: {
                MissionState.PREFLIGHT,
            },
            MissionState.PREFLIGHT: {
                MissionState.ARMING,
                MissionState.IDLE,
            },
            MissionState.ARMING: {
                MissionState.TAKEOFF,
                MissionState.PREFLIGHT,
                MissionState.IDLE,
            },
            MissionState.TAKEOFF: {
                MissionState.CLIMBING,
                MissionState.LOITER,
                MissionState.EMERGENCY,
                MissionState.LANDING,
            },
            MissionState.CLIMBING: {
                MissionState.TRANSIT,
                MissionState.LOITER,
                MissionState.MISSION,
                MissionState.RTL,
                MissionState.EMERGENCY,
            },
            MissionState.TRANSIT: {
                MissionState.LOITER,
                MissionState.MISSION,
                MissionState.SURVEY,
                MissionState.RTL,
                MissionState.EMERGENCY,
            },
            MissionState.LOITER: {
                MissionState.TRANSIT,
                MissionState.MISSION,
                MissionState.SURVEY,
                MissionState.RTL,
                MissionState.EMERGENCY,
            },
            MissionState.MISSION: {
                MissionState.LOITER,
                MissionState.TRANSIT,
                MissionState.RTL,
                MissionState.EMERGENCY,
            },
            MissionState.SURVEY: {
                MissionState.LOITER,
                MissionState.TRANSIT,
                MissionState.RTL,
                MissionState.EMERGENCY,
            },
            MissionState.RTL: {
                MissionState.DESCENDING,
                MissionState.LOITER,
                MissionState.EMERGENCY,
            },
            MissionState.DESCENDING: {
                MissionState.LANDING,
                MissionState.LOITER,
                MissionState.EMERGENCY,
            },
            MissionState.LANDING: {
                MissionState.LANDED,
                MissionState.LOITER,
                MissionState.EMERGENCY,
            },
            MissionState.LANDED: {
                MissionState.IDLE,
                MissionState.PREFLIGHT,
            },
            MissionState.EMERGENCY: {
                MissionState.LANDING,
                MissionState.LANDED,
                MissionState.LOITER,
            },
            MissionState.LOST_LINK: {
                MissionState.RTL,
                MissionState.LOITER,
                MissionState.EMERGENCY,
                MissionState.LANDING,
            },
        }

    def can_transition(self, to_state: MissionState) -> bool:
        """Check if transition is valid."""
        valid = self._transitions.get(self._state, set())
        return to_state in valid

    def transition(self, to_state: MissionState, reason: str = "") -> bool:
        """
        Attempt state transition.

        Args:
            to_state: Target state
            reason: Reason for transition

        Returns:
            True if transition succeeded
        """
        if not self.can_transition(to_state):
            logger.warning(
                "Invalid state transition",
                from_state=self._state.name,
                to_state=to_state.name,
            )
            return False

        # Call exit callbacks
        self._call_exit_callbacks()

        # Record transition
        transition = StateTransition(
            from_state=self._state,
            to_state=to_state,
            reason=reason,
            timestamp=time.time(),
        )
        self._history.append(transition)

        # Keep only last 100 transitions
        if len(self._history) > 100:
            self._history = self._history[-100:]

        # Update state
        old_state = self._state
        self._state = to_state
        self._entry_time = time.time()

        logger.info(
            "State transition",
            from_state=old_state.name,
            to_state=to_state.name,
            reason=reason,
        )

        # Call enter callbacks
        self._call_enter_callbacks()

        return True

    def on_enter(self, state: MissionState, callback: Callable):
        """Register callback for state entry."""
        if state not in self._on_enter:
            self._on_enter[state] = []
        self._on_enter[state].append(callback)

    def on_exit(self, state: MissionState, callback: Callable):
        """Register callback for state exit."""
        if state not in self._on_exit:
            self._on_exit[state] = []
        self._on_exit[state].append(callback)

    def _call_enter_callbacks(self):
        """Call state entry callbacks."""
        if self._state in self._on_enter:
            for callback in self._on_enter[self._state]:
                try:
                    callback(self._state)
                except Exception as e:
                    logger.error("Enter callback error", error=str(e))

    def _call_exit_callbacks(self):
        """Call state exit callbacks."""
        if self._state in self._on_exit:
            for callback in self._on_exit[self._state]:
                try:
                    callback(self._state)
                except Exception as e:
                    logger.error("Exit callback error", error=str(e))

    def reset(self):
        """Reset to idle state."""
        self._state = MissionState.IDLE
        self._entry_time = time.time()
        self._history.clear()
        logger.info("State machine reset")

    def is_flying(self) -> bool:
        """Check if in a flying state."""
        flying_states = {
            MissionState.TAKEOFF,
            MissionState.CLIMBING,
            MissionState.TRANSIT,
            MissionState.LOITER,
            MissionState.MISSION,
            MissionState.SURVEY,
            MissionState.RTL,
            MissionState.DESCENDING,
        }
        return self._state in flying_states

    def is_emergency(self) -> bool:
        """Check if in emergency state."""
        return self._state in {MissionState.EMERGENCY, MissionState.LOST_LINK}

    def get_status_dict(self) -> dict:
        """Get status as dictionary."""
        return {
            "state": self._state.name,
            "time_in_state": round(self.time_in_state, 1),
            "is_flying": self.is_flying(),
            "is_emergency": self.is_emergency(),
            "transitions": len(self._history),
        }
