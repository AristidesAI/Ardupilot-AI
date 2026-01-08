"""
Battery management for drone operations.

Monitors battery state, predicts remaining flight time, and triggers warnings.
"""

import time
from dataclasses import dataclass, field
from enum import Enum
from typing import List, Optional, Tuple

import structlog

logger = structlog.get_logger()


class BatteryState(Enum):
    """Battery state."""
    UNKNOWN = "unknown"
    FULL = "full"  # >90%
    GOOD = "good"  # 50-90%
    LOW = "low"  # 25-50%
    CRITICAL = "critical"  # <25%
    EMERGENCY = "emergency"  # <15%


@dataclass
class BatteryStatus:
    """Current battery status."""
    voltage: float = 0.0
    current: float = 0.0  # Positive = discharging
    consumed_mah: float = 0.0
    remaining_mah: float = 0.0
    soc_percent: float = 100.0
    cell_voltage: float = 0.0
    cell_count: int = 0
    state: BatteryState = BatteryState.UNKNOWN
    temperature: float = 0.0
    time_remaining_s: float = 0.0
    distance_remaining_m: float = 0.0
    timestamp: float = 0.0


@dataclass
class BatteryConfig:
    """Battery configuration."""
    capacity_mah: float = 10000.0
    cell_count: int = 6
    cell_voltage_min: float = 3.3
    cell_voltage_max: float = 4.2
    cell_voltage_nominal: float = 3.7
    critical_soc: float = 15.0
    low_soc: float = 25.0
    warning_soc: float = 35.0


class BatteryManager:
    """
    Manages battery state and predictions.

    Features:
    - Real-time voltage/current monitoring
    - State of charge estimation
    - Remaining flight time prediction
    - Critical battery warnings
    """

    def __init__(self, config: Optional[BatteryConfig] = None):
        """
        Initialize battery manager.

        Args:
            config: Battery configuration
        """
        self.config = config or BatteryConfig()
        self._status = BatteryStatus()

        # History for averaging/prediction
        self._voltage_history: List[Tuple[float, float]] = []
        self._current_history: List[Tuple[float, float]] = []

        # State tracking
        self._last_soc = 100.0
        self._soc_from_voltage = True  # Use voltage-based SOC if no coulomb counting

    @property
    def status(self) -> BatteryStatus:
        """Get current battery status."""
        return self._status

    @property
    def soc(self) -> float:
        """Get state of charge percentage."""
        return self._status.soc_percent

    @property
    def state(self) -> BatteryState:
        """Get battery state."""
        return self._status.state

    @property
    def voltage(self) -> float:
        """Get current voltage."""
        return self._status.voltage

    @property
    def current(self) -> float:
        """Get current draw (amps)."""
        return self._status.current

    @property
    def is_critical(self) -> bool:
        """Check if battery is critical."""
        return self._status.state in (BatteryState.CRITICAL, BatteryState.EMERGENCY)

    @property
    def is_low(self) -> bool:
        """Check if battery is low."""
        return self._status.state in (
            BatteryState.LOW, BatteryState.CRITICAL, BatteryState.EMERGENCY
        )

    def update(
        self,
        voltage: float,
        current: float,
        consumed_mah: float = 0,
        remaining_percent: int = -1,
        cell_count: int = 0,
        temperature: float = 0,
    ):
        """
        Update battery state from telemetry.

        Args:
            voltage: Total voltage
            current: Current draw (amps, positive = discharging)
            consumed_mah: mAh consumed
            remaining_percent: Remaining percentage (-1 = unknown)
            cell_count: Number of cells
            temperature: Battery temperature (C)
        """
        now = time.time()

        # Update basic values
        self._status.voltage = voltage
        self._status.current = current
        self._status.consumed_mah = consumed_mah
        self._status.temperature = temperature
        self._status.timestamp = now

        # Track history
        self._voltage_history.append((now, voltage))
        self._current_history.append((now, current))

        # Keep last 60 seconds
        self._voltage_history = [
            (t, v) for t, v in self._voltage_history
            if now - t < 60
        ]
        self._current_history = [
            (t, c) for t, c in self._current_history
            if now - t < 60
        ]

        # Determine cell count
        if cell_count > 0:
            self._status.cell_count = cell_count
        elif self._status.cell_count == 0:
            # Estimate from voltage
            self._status.cell_count = self._estimate_cell_count(voltage)

        # Calculate cell voltage
        if self._status.cell_count > 0:
            self._status.cell_voltage = voltage / self._status.cell_count

        # Calculate SOC
        if remaining_percent >= 0:
            self._status.soc_percent = remaining_percent
            self._soc_from_voltage = False
        elif consumed_mah > 0:
            self._status.remaining_mah = self.config.capacity_mah - consumed_mah
            self._status.soc_percent = (self._status.remaining_mah / self.config.capacity_mah) * 100
            self._soc_from_voltage = False
        else:
            # Estimate from voltage
            self._status.soc_percent = self._voltage_to_soc(self._status.cell_voltage)
            self._soc_from_voltage = True

        # Update remaining capacity
        self._status.remaining_mah = (self._status.soc_percent / 100) * self.config.capacity_mah

        # Determine state
        self._status.state = self._determine_state()

        # Predict remaining time
        self._update_predictions()

        # Log warnings
        if self._status.state != self._last_state_check:
            self._log_state_change()
            self._last_state_check = self._status.state

    _last_state_check = BatteryState.UNKNOWN

    def _estimate_cell_count(self, voltage: float) -> int:
        """Estimate cell count from voltage."""
        if voltage <= 0:
            return 0

        # Assume cells are between 3.3V and 4.2V
        for cells in [3, 4, 6, 8, 10, 12, 14]:
            v_per_cell = voltage / cells
            if 3.0 <= v_per_cell <= 4.3:
                return cells

        return self.config.cell_count

    def _voltage_to_soc(self, cell_voltage: float) -> float:
        """
        Convert cell voltage to state of charge.

        Uses a simplified LiPo discharge curve.
        """
        if cell_voltage >= 4.2:
            return 100.0
        elif cell_voltage >= 4.1:
            return 90 + (cell_voltage - 4.1) / 0.1 * 10
        elif cell_voltage >= 4.0:
            return 80 + (cell_voltage - 4.0) / 0.1 * 10
        elif cell_voltage >= 3.9:
            return 60 + (cell_voltage - 3.9) / 0.1 * 20
        elif cell_voltage >= 3.8:
            return 40 + (cell_voltage - 3.8) / 0.1 * 20
        elif cell_voltage >= 3.7:
            return 20 + (cell_voltage - 3.7) / 0.1 * 20
        elif cell_voltage >= 3.5:
            return 5 + (cell_voltage - 3.5) / 0.2 * 15
        elif cell_voltage >= 3.3:
            return (cell_voltage - 3.3) / 0.2 * 5
        else:
            return 0.0

    def _determine_state(self) -> BatteryState:
        """Determine battery state from SOC."""
        soc = self._status.soc_percent

        if soc > 90:
            return BatteryState.FULL
        elif soc > 50:
            return BatteryState.GOOD
        elif soc > self.config.low_soc:
            return BatteryState.LOW
        elif soc > self.config.critical_soc:
            return BatteryState.CRITICAL
        else:
            return BatteryState.EMERGENCY

    def _update_predictions(self):
        """Update flight time and distance predictions."""
        # Average current over last 30 seconds
        recent_current = [
            c for t, c in self._current_history
            if time.time() - t < 30
        ]

        if recent_current and len(recent_current) >= 3:
            avg_current = sum(recent_current) / len(recent_current)

            if avg_current > 0.1:  # Discharging
                # Time remaining
                remaining_ah = self._status.remaining_mah / 1000
                self._status.time_remaining_s = (remaining_ah / avg_current) * 3600

                # Estimate distance (assume 10 m/s average speed)
                avg_speed = 10.0
                self._status.distance_remaining_m = (
                    self._status.time_remaining_s * avg_speed
                )
        else:
            self._status.time_remaining_s = 0
            self._status.distance_remaining_m = 0

    def _log_state_change(self):
        """Log battery state changes."""
        state = self._status.state

        if state == BatteryState.EMERGENCY:
            logger.critical(
                "BATTERY EMERGENCY",
                soc=self._status.soc_percent,
                voltage=self._status.voltage,
            )
        elif state == BatteryState.CRITICAL:
            logger.error(
                "Battery critical",
                soc=self._status.soc_percent,
                voltage=self._status.voltage,
            )
        elif state == BatteryState.LOW:
            logger.warning(
                "Battery low",
                soc=self._status.soc_percent,
                voltage=self._status.voltage,
            )

    def get_voltage_trend(self) -> float:
        """
        Get voltage trend (V/min).

        Returns:
            Voltage change rate (negative = dropping)
        """
        if len(self._voltage_history) < 2:
            return 0.0

        recent = self._voltage_history[-10:]
        if len(recent) < 2:
            return 0.0

        t1, v1 = recent[0]
        t2, v2 = recent[-1]
        dt = t2 - t1

        if dt > 0:
            return (v2 - v1) / dt * 60  # V/min
        return 0.0

    def get_current_average(self, window_s: float = 30) -> float:
        """
        Get average current over window.

        Args:
            window_s: Averaging window in seconds

        Returns:
            Average current (amps)
        """
        now = time.time()
        recent = [c for t, c in self._current_history if now - t < window_s]

        if recent:
            return sum(recent) / len(recent)
        return self._status.current

    def predict_soc_at_time(self, future_s: float) -> float:
        """
        Predict SOC at future time.

        Args:
            future_s: Seconds in the future

        Returns:
            Predicted SOC percentage
        """
        avg_current = self.get_current_average()

        if avg_current <= 0:
            return self._status.soc_percent

        # mAh that will be consumed
        consumed_mah = avg_current * (future_s / 3600) * 1000

        # Predicted remaining
        predicted_remaining = self._status.remaining_mah - consumed_mah
        predicted_soc = (predicted_remaining / self.config.capacity_mah) * 100

        return max(0, min(100, predicted_soc))

    def can_complete_mission(
        self,
        mission_time_s: float,
        return_time_s: float,
        reserve_percent: float = 20,
    ) -> Tuple[bool, float]:
        """
        Check if battery can complete a mission.

        Args:
            mission_time_s: Estimated mission duration
            return_time_s: Time to return home
            reserve_percent: Reserve battery percentage

        Returns:
            Tuple of (can_complete, margin_percent)
        """
        total_time = mission_time_s + return_time_s
        predicted_soc = self.predict_soc_at_time(total_time)
        margin = predicted_soc - reserve_percent

        return margin >= 0, margin

    def get_status_dict(self) -> dict:
        """Get status as dictionary."""
        return {
            "voltage": round(self._status.voltage, 2),
            "current": round(self._status.current, 2),
            "soc_percent": round(self._status.soc_percent, 1),
            "cell_voltage": round(self._status.cell_voltage, 3),
            "cell_count": self._status.cell_count,
            "state": self._status.state.value,
            "time_remaining_s": round(self._status.time_remaining_s, 0),
            "distance_remaining_m": round(self._status.distance_remaining_m, 0),
        }
