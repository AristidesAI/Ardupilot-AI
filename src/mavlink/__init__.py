"""
MAVLink communication module for ArduPilot drone.

Provides connection management, message handling, and telemetry processing.
"""

from .connection import MAVLinkConnection
from .messages import MessageHandler, Telemetry
from .commands import CommandSender

__all__ = [
    "MAVLinkConnection",
    "MessageHandler",
    "Telemetry",
    "CommandSender",
]
