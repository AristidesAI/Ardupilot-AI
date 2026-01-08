"""
Flight time prediction based on battery and flight conditions.
"""

import time
from dataclasses import dataclass
from typing import Optional, List, Tuple

import structlog

from .battery import BatteryManager

logger = structlog.get_logger()


@dataclass
class FlightPrediction:
    """Flight time/distance prediction."""
    time_remaining_s: float = 0.0
    distance_remaining_m: float = 0.0
    rtl_reserve_s: float = 0.0
    effective_range_m: float = 0.0
    confidence: float = 0.0  # 0-1


class FlightTimePredictor:
    """
    Predicts remaining flight time and range.

    Uses battery state, average consumption, and flight conditions.
    """

    def __init__(self, battery_manager: BatteryManager):
        """
        Initialize predictor.

        Args:
            battery_manager: Battery manager instance
        """
        self.battery = battery_manager

        # Flight parameters
        self._avg_speed_ms = 10.0
        self._avg_current_a = 15.0
        self._rtl_speed_ms = 8.0
        self._hover_current_a = 20.0

        # History for averaging
        self._power_history: List[Tuple[float, float]] = []
        self._speed_history: List[Tuple[float, float]] = []

    def update(
        self,
        groundspeed_ms: float,
        current_a: float,
        distance_to_home_m: float,
    ):
        """
        Update prediction with current flight data.

        Args:
            groundspeed_ms: Current ground speed
            current_a: Current battery draw
            distance_to_home_m: Distance to home position
        """
        now = time.time()

        # Track history
        self._power_history.append((now, current_a))
        self._speed_history.append((now, groundspeed_ms))

        # Keep last 5 minutes
        self._power_history = [
            (t, p) for t, p in self._power_history
            if now - t < 300
        ]
        self._speed_history = [
            (t, s) for t, s in self._speed_history
            if now - t < 300
        ]

        # Update averages
        if self._power_history:
            self._avg_current_a = sum(p for _, p in self._power_history) / len(self._power_history)
        if self._speed_history:
            self._avg_speed_ms = sum(s for _, s in self._speed_history) / len(self._speed_history)

    def predict(self, distance_to_home_m: float) -> FlightPrediction:
        """
        Get flight prediction.

        Args:
            distance_to_home_m: Current distance to home

        Returns:
            Flight prediction
        """
        battery_status = self.battery.status

        # Time remaining at current consumption
        if self._avg_current_a > 0.1:
            remaining_ah = battery_status.remaining_mah / 1000
            time_remaining = (remaining_ah / self._avg_current_a) * 3600
        else:
            time_remaining = float('inf')

        # Distance remaining
        if self._avg_speed_ms > 0.1:
            distance_remaining = time_remaining * self._avg_speed_ms
        else:
            distance_remaining = 0

        # RTL reserve (time to get home)
        if self._rtl_speed_ms > 0:
            rtl_time = distance_to_home_m / self._rtl_speed_ms
        else:
            rtl_time = 0

        # Add hover/landing time
        rtl_time += 60  # 1 minute for landing

        # Effective range (accounting for return)
        if time_remaining > rtl_time:
            available_time = (time_remaining - rtl_time) / 2  # Out and back
            effective_range = available_time * self._avg_speed_ms
        else:
            effective_range = 0

        # Confidence based on data quality
        confidence = min(1.0, len(self._power_history) / 30) * 0.5
        confidence += min(0.5, len(self._speed_history) / 30) * 0.5

        return FlightPrediction(
            time_remaining_s=time_remaining,
            distance_remaining_m=distance_remaining,
            rtl_reserve_s=rtl_time,
            effective_range_m=effective_range,
            confidence=confidence,
        )

    def should_rtl(
        self,
        distance_to_home_m: float,
        reserve_percent: float = 20,
    ) -> Tuple[bool, str]:
        """
        Check if RTL should be triggered.

        Args:
            distance_to_home_m: Distance to home
            reserve_percent: Reserve battery percentage

        Returns:
            Tuple of (should_rtl, reason)
        """
        prediction = self.predict(distance_to_home_m)
        battery_status = self.battery.status

        # Check battery SOC
        if battery_status.soc_percent <= reserve_percent:
            return True, f"Battery at {battery_status.soc_percent:.0f}%"

        # Check if we can make it home
        if prediction.rtl_reserve_s > prediction.time_remaining_s * 0.8:
            return True, "Insufficient battery for RTL"

        # Check effective range
        if prediction.effective_range_m < distance_to_home_m:
            return True, "Beyond effective range"

        return False, ""

    def get_max_range(self, reserve_percent: float = 20) -> float:
        """
        Calculate maximum one-way range.

        Args:
            reserve_percent: Reserve battery percentage

        Returns:
            Maximum range in meters
        """
        battery_status = self.battery.status
        usable_soc = battery_status.soc_percent - reserve_percent

        if usable_soc <= 0:
            return 0

        # mAh available
        usable_mah = (usable_soc / 100) * self.battery.config.capacity_mah

        # Time at average consumption
        if self._avg_current_a > 0.1:
            time_s = (usable_mah / 1000 / self._avg_current_a) * 3600
        else:
            time_s = 0

        # Range (half for return)
        one_way_time = time_s / 2
        return one_way_time * self._avg_speed_ms
