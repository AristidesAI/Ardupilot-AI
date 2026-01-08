"""
Energy and battery management module.

Provides battery monitoring, state prediction, and energy-aware decisions.
"""

from .battery import BatteryManager, BatteryState
from .predictor import FlightTimePredictor

__all__ = [
    "BatteryManager",
    "BatteryState",
    "FlightTimePredictor",
]
