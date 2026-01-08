"""
Safety and failsafe management module.

Handles geofencing, failsafes, and emergency procedures.
"""

from .geofence import Geofence, GeofenceAction
from .failsafe import FailsafeManager, FailsafeType

__all__ = [
    "Geofence",
    "GeofenceAction",
    "FailsafeManager",
    "FailsafeType",
]
