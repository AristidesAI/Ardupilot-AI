"""
Failsafe management for drone safety.

Monitors system state and triggers appropriate failsafe actions.
"""

import time
from dataclasses import dataclass
from enum import Enum, auto
from typing import Callable, Dict, List, Optional

import structlog

logger = structlog.get_logger()


class FailsafeType(Enum):
    """Type of failsafe condition."""
    BATTERY_LOW = auto()
    BATTERY_CRITICAL = auto()
    GPS_LOST = auto()
    LINK_LOST = auto()
    GEOFENCE_BREACH = auto()
    MOTOR_FAILURE = auto()
    EKF_FAILURE = auto()
    ALTITUDE_BREACH = auto()


class FailsafeAction(Enum):
    """Failsafe action to take."""
    NONE = 0
    WARN = 1
    LOITER = 2
    RTL = 3
    LAND = 4
    EMERGENCY_LAND = 5


@dataclass
class FailsafeConfig:
    """Failsafe configuration."""
    # Battery
    battery_low_percent: float = 25.0
    battery_critical_percent: float = 15.0
    battery_low_action: FailsafeAction = FailsafeAction.RTL
    battery_critical_action: FailsafeAction = FailsafeAction.LAND

    # GPS
    gps_lost_timeout_s: float = 5.0
    gps_lost_action: FailsafeAction = FailsafeAction.LAND

    # Link
    link_lost_timeout_s: float = 5.0
    link_lost_action: FailsafeAction = FailsafeAction.RTL

    # Geofence
    geofence_action: FailsafeAction = FailsafeAction.RTL

    # EKF
    ekf_failure_action: FailsafeAction = FailsafeAction.LAND


@dataclass
class FailsafeState:
    """Current failsafe state."""
    active_failsafes: List[FailsafeType]
    highest_priority_action: FailsafeAction
    is_failsafe_active: bool
    timestamp: float


class FailsafeManager:
    """
    Manages failsafe conditions and actions.

    Monitors system state and triggers failsafes when needed.
    """

    def __init__(self, config: Optional[FailsafeConfig] = None):
        """
        Initialize failsafe manager.

        Args:
            config: Failsafe configuration
        """
        self.config = config or FailsafeConfig()

        # Current state
        self._active_failsafes: Dict[FailsafeType, float] = {}  # type -> activation time
        self._last_gps_time = time.time()
        self._last_heartbeat_time = time.time()

        # Callbacks
        self._action_callbacks: Dict[FailsafeAction, List[Callable]] = {}

        # Priority order (higher = more urgent)
        self._action_priority = {
            FailsafeAction.NONE: 0,
            FailsafeAction.WARN: 1,
            FailsafeAction.LOITER: 2,
            FailsafeAction.RTL: 3,
            FailsafeAction.LAND: 4,
            FailsafeAction.EMERGENCY_LAND: 5,
        }

    def register_action_callback(
        self,
        action: FailsafeAction,
        callback: Callable,
    ):
        """
        Register callback for failsafe action.

        Args:
            action: Failsafe action
            callback: Function to call
        """
        if action not in self._action_callbacks:
            self._action_callbacks[action] = []
        self._action_callbacks[action].append(callback)

    def update(
        self,
        battery_percent: float,
        gps_ok: bool,
        link_ok: bool,
        ekf_ok: bool,
        in_geofence: bool,
        altitude_ok: bool = True,
    ) -> FailsafeState:
        """
        Update failsafe state based on current conditions.

        Args:
            battery_percent: Battery SOC
            gps_ok: GPS status
            link_ok: Communication link status
            ekf_ok: EKF health
            in_geofence: Inside geofence
            altitude_ok: Altitude within limits

        Returns:
            Current failsafe state
        """
        now = time.time()
        triggered_actions = []

        # Check battery
        if battery_percent <= self.config.battery_critical_percent:
            self._activate_failsafe(FailsafeType.BATTERY_CRITICAL)
            triggered_actions.append(self.config.battery_critical_action)
        elif battery_percent <= self.config.battery_low_percent:
            self._activate_failsafe(FailsafeType.BATTERY_LOW)
            triggered_actions.append(self.config.battery_low_action)
        else:
            self._clear_failsafe(FailsafeType.BATTERY_LOW)
            self._clear_failsafe(FailsafeType.BATTERY_CRITICAL)

        # Check GPS
        if gps_ok:
            self._last_gps_time = now
            self._clear_failsafe(FailsafeType.GPS_LOST)
        elif now - self._last_gps_time > self.config.gps_lost_timeout_s:
            self._activate_failsafe(FailsafeType.GPS_LOST)
            triggered_actions.append(self.config.gps_lost_action)

        # Check link
        if link_ok:
            self._last_heartbeat_time = now
            self._clear_failsafe(FailsafeType.LINK_LOST)
        elif now - self._last_heartbeat_time > self.config.link_lost_timeout_s:
            self._activate_failsafe(FailsafeType.LINK_LOST)
            triggered_actions.append(self.config.link_lost_action)

        # Check EKF
        if ekf_ok:
            self._clear_failsafe(FailsafeType.EKF_FAILURE)
        else:
            self._activate_failsafe(FailsafeType.EKF_FAILURE)
            triggered_actions.append(self.config.ekf_failure_action)

        # Check geofence
        if in_geofence:
            self._clear_failsafe(FailsafeType.GEOFENCE_BREACH)
        else:
            self._activate_failsafe(FailsafeType.GEOFENCE_BREACH)
            triggered_actions.append(self.config.geofence_action)

        # Check altitude
        if altitude_ok:
            self._clear_failsafe(FailsafeType.ALTITUDE_BREACH)
        else:
            self._activate_failsafe(FailsafeType.ALTITUDE_BREACH)
            triggered_actions.append(FailsafeAction.RTL)

        # Determine highest priority action
        highest_action = FailsafeAction.NONE
        for action in triggered_actions:
            if self._action_priority[action] > self._action_priority[highest_action]:
                highest_action = action

        # Execute callbacks for new actions
        if highest_action != FailsafeAction.NONE:
            self._execute_action(highest_action)

        return FailsafeState(
            active_failsafes=list(self._active_failsafes.keys()),
            highest_priority_action=highest_action,
            is_failsafe_active=len(self._active_failsafes) > 0,
            timestamp=now,
        )

    def _activate_failsafe(self, fs_type: FailsafeType):
        """Activate a failsafe."""
        if fs_type not in self._active_failsafes:
            self._active_failsafes[fs_type] = time.time()
            logger.warning("Failsafe activated", type=fs_type.name)

    def _clear_failsafe(self, fs_type: FailsafeType):
        """Clear a failsafe."""
        if fs_type in self._active_failsafes:
            del self._active_failsafes[fs_type]
            logger.info("Failsafe cleared", type=fs_type.name)

    def _execute_action(self, action: FailsafeAction):
        """Execute failsafe action callbacks."""
        if action in self._action_callbacks:
            for callback in self._action_callbacks[action]:
                try:
                    callback(action)
                except Exception as e:
                    logger.error("Failsafe callback error", error=str(e))

    def is_failsafe_active(self, fs_type: FailsafeType) -> bool:
        """Check if specific failsafe is active."""
        return fs_type in self._active_failsafes

    def get_active_failsafes(self) -> List[FailsafeType]:
        """Get list of active failsafes."""
        return list(self._active_failsafes.keys())

    def clear_all(self):
        """Clear all failsafes."""
        self._active_failsafes.clear()
        logger.info("All failsafes cleared")

    def get_status_dict(self) -> dict:
        """Get status as dictionary."""
        return {
            "active_failsafes": [fs.name for fs in self._active_failsafes.keys()],
            "is_active": len(self._active_failsafes) > 0,
        }
