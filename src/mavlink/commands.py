"""
MAVLink command sender for drone control.

Provides high-level command interface for drone operations.
"""

import asyncio
import time
from dataclasses import dataclass
from enum import Enum
from typing import Optional, Tuple

import structlog
from pymavlink import mavutil

from .connection import MAVLinkConnection
from .messages import FlightMode

logger = structlog.get_logger()


class CommandResult(Enum):
    """Command execution result."""
    SUCCESS = 0
    DENIED = 1
    FAILED = 2
    TIMEOUT = 3
    IN_PROGRESS = 4
    TEMPORARILY_REJECTED = 5
    UNSUPPORTED = 6


@dataclass
class CommandAck:
    """Command acknowledgment."""
    command: int
    result: CommandResult
    progress: int = 0
    result_param2: int = 0


class CommandSender:
    """
    High-level command interface for drone control.

    Handles command sending, acknowledgment waiting, and retries.
    """

    def __init__(self, connection: MAVLinkConnection):
        """
        Initialize command sender.

        Args:
            connection: MAVLink connection
        """
        self.connection = connection
        self._pending_ack: Optional[int] = None
        self._last_ack: Optional[CommandAck] = None
        self._ack_event = asyncio.Event()

        # Register for command acks
        connection.register_message_callback(
            "COMMAND_ACK",
            self._handle_command_ack,
        )

    def _handle_command_ack(self, msg):
        """Handle COMMAND_ACK message."""
        result = CommandResult(min(msg.result, 6))
        self._last_ack = CommandAck(
            command=msg.command,
            result=result,
            progress=getattr(msg, 'progress', 0),
            result_param2=getattr(msg, 'result_param2', 0),
        )

        if self._pending_ack == msg.command:
            self._ack_event.set()

    async def send_command(
        self,
        command: int,
        param1: float = 0,
        param2: float = 0,
        param3: float = 0,
        param4: float = 0,
        param5: float = 0,
        param6: float = 0,
        param7: float = 0,
        timeout: float = 5.0,
        retries: int = 3,
    ) -> Tuple[bool, Optional[CommandResult]]:
        """
        Send command and wait for acknowledgment.

        Args:
            command: MAV_CMD command ID
            param1-7: Command parameters
            timeout: Timeout per attempt
            retries: Number of retry attempts

        Returns:
            Tuple of (success, result)
        """
        for attempt in range(retries):
            self._pending_ack = command
            self._ack_event.clear()
            self._last_ack = None

            # Send command
            sent = await self.connection.send_command_long(
                command,
                param1, param2, param3, param4, param5, param6, param7,
                confirmation=attempt,
            )

            if not sent:
                continue

            # Wait for ack
            try:
                await asyncio.wait_for(
                    self._ack_event.wait(),
                    timeout=timeout,
                )

                if self._last_ack and self._last_ack.command == command:
                    result = self._last_ack.result
                    if result == CommandResult.SUCCESS:
                        return True, result
                    elif result == CommandResult.IN_PROGRESS:
                        # Wait for completion
                        continue
                    else:
                        logger.warning(
                            "Command rejected",
                            command=command,
                            result=result.name,
                        )
                        return False, result

            except asyncio.TimeoutError:
                logger.warning(
                    "Command timeout",
                    command=command,
                    attempt=attempt + 1,
                )

        self._pending_ack = None
        return False, CommandResult.TIMEOUT

    async def arm(self, force: bool = False) -> bool:
        """
        Arm the vehicle.

        Args:
            force: Force arm (bypass pre-arm checks)

        Returns:
            True if armed successfully
        """
        logger.info("Arming vehicle", force=force)

        success, result = await self.send_command(
            400,  # MAV_CMD_COMPONENT_ARM_DISARM
            param1=1,  # Arm
            param2=21196 if force else 0,
        )

        if success:
            logger.info("Vehicle armed")
        else:
            logger.error("Arm failed", result=result.name if result else "timeout")

        return success

    async def disarm(self, force: bool = False) -> bool:
        """
        Disarm the vehicle.

        Args:
            force: Force disarm

        Returns:
            True if disarmed successfully
        """
        logger.info("Disarming vehicle", force=force)

        success, result = await self.send_command(
            400,  # MAV_CMD_COMPONENT_ARM_DISARM
            param1=0,  # Disarm
            param2=21196 if force else 0,
        )

        if success:
            logger.info("Vehicle disarmed")
        else:
            logger.error("Disarm failed", result=result.name if result else "timeout")

        return success

    async def set_mode(self, mode: FlightMode) -> bool:
        """
        Set flight mode.

        Args:
            mode: Target flight mode

        Returns:
            True if mode set successfully
        """
        logger.info("Setting mode", mode=mode.name)

        success, result = await self.send_command(
            176,  # MAV_CMD_DO_SET_MODE
            param1=1,  # MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
            param2=mode.value,
        )

        if success:
            logger.info("Mode set", mode=mode.name)
        else:
            logger.error("Mode change failed", result=result.name if result else "timeout")

        return success

    async def takeoff(self, altitude_m: float) -> bool:
        """
        Command takeoff.

        Args:
            altitude_m: Target altitude (meters AGL)

        Returns:
            True if takeoff command accepted
        """
        logger.info("Taking off", altitude=altitude_m)

        # First set to GUIDED mode
        if not await self.set_mode(FlightMode.GUIDED):
            logger.error("Failed to set GUIDED mode for takeoff")
            return False

        await asyncio.sleep(0.5)

        # Send takeoff command
        success, result = await self.send_command(
            22,  # MAV_CMD_NAV_TAKEOFF
            param7=altitude_m,
        )

        if success:
            logger.info("Takeoff initiated", altitude=altitude_m)
        else:
            logger.error("Takeoff failed", result=result.name if result else "timeout")

        return success

    async def land(self) -> bool:
        """
        Command landing.

        Returns:
            True if land command accepted
        """
        logger.info("Landing")

        success, result = await self.send_command(21)  # MAV_CMD_NAV_LAND

        if success:
            logger.info("Land command accepted")
        else:
            # Try setting LAND mode instead
            logger.warning("Land command failed, trying LAND mode")
            return await self.set_mode(FlightMode.LAND)

        return success

    async def rtl(self) -> bool:
        """
        Return to launch.

        Returns:
            True if RTL initiated
        """
        logger.info("Returning to launch")
        return await self.set_mode(FlightMode.RTL)

    async def loiter(self) -> bool:
        """
        Enter loiter/hold position.

        Returns:
            True if loiter mode set
        """
        logger.info("Entering loiter")
        return await self.set_mode(FlightMode.LOITER)

    async def goto(
        self,
        lat: float,
        lon: float,
        alt: float,
        speed: float = 0,
    ) -> bool:
        """
        Go to position.

        Args:
            lat: Target latitude (degrees)
            lon: Target longitude (degrees)
            alt: Target altitude (meters AGL)
            speed: Ground speed (0 = default)

        Returns:
            True if command sent
        """
        logger.info("Going to position", lat=lat, lon=lon, alt=alt)

        # Ensure GUIDED mode
        if not await self.set_mode(FlightMode.GUIDED):
            return False

        # Send position command
        return await self.connection.goto(lat, lon, alt, speed)

    async def set_position_ned(
        self,
        north_m: float,
        east_m: float,
        down_m: float,
        yaw_deg: Optional[float] = None,
    ) -> bool:
        """
        Set target position in local NED frame.

        Args:
            north_m: North offset from home (meters)
            east_m: East offset from home (meters)
            down_m: Down offset (negative = up) (meters)
            yaw_deg: Optional target yaw (degrees)

        Returns:
            True if command sent
        """
        if not self.connection.connected or not self.connection.mav:
            return False

        try:
            type_mask = 0b0000111111111000  # Position only

            if yaw_deg is not None:
                type_mask = 0b0000011111111000  # Position + yaw
                yaw_rad = yaw_deg * 3.14159 / 180.0
            else:
                yaw_rad = 0

            self.connection.mav._connection.mav.set_position_target_local_ned_send(
                0,  # time_boot_ms
                self.connection.target_system,
                self.connection.target_component,
                mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                type_mask,
                north_m, east_m, down_m,
                0, 0, 0,  # velocity
                0, 0, 0,  # acceleration
                yaw_rad, 0,  # yaw, yaw_rate
            )
            return True
        except Exception as e:
            logger.error("Set position failed", error=str(e))
            return False

    async def set_velocity(
        self,
        vx_ms: float,
        vy_ms: float,
        vz_ms: float,
        yaw_rate_dps: float = 0,
    ) -> bool:
        """
        Set velocity in body frame.

        Args:
            vx_ms: Forward velocity (m/s)
            vy_ms: Right velocity (m/s)
            vz_ms: Down velocity (m/s, negative = up)
            yaw_rate_dps: Yaw rate (degrees/second)

        Returns:
            True if command sent
        """
        return await self.connection.set_velocity(
            vx_ms, vy_ms, vz_ms,
            yaw_rate_dps * 3.14159 / 180.0,
        )

    async def set_altitude(self, altitude_m: float) -> bool:
        """
        Change altitude (hold current position).

        Args:
            altitude_m: Target altitude (meters AGL)

        Returns:
            True if command sent
        """
        success, result = await self.send_command(
            113,  # MAV_CMD_CONDITION_CHANGE_ALT
            param1=10,  # descent rate (ignored in copter)
            param7=altitude_m,
        )
        return success

    async def set_yaw(
        self,
        heading_deg: float,
        rate_dps: float = 0,
        relative: bool = False,
    ) -> bool:
        """
        Set yaw heading.

        Args:
            heading_deg: Target heading (degrees)
            rate_dps: Yaw rate (0 = default)
            relative: If True, heading is relative

        Returns:
            True if command sent
        """
        success, result = await self.send_command(
            115,  # MAV_CMD_CONDITION_YAW
            param1=heading_deg,
            param2=rate_dps,
            param3=1 if heading_deg >= 0 else -1,
            param4=1 if relative else 0,
        )
        return success

    async def set_roi(
        self,
        lat: float,
        lon: float,
        alt: float,
    ) -> bool:
        """
        Set region of interest (camera/gimbal target).

        Args:
            lat: Target latitude
            lon: Target longitude
            alt: Target altitude (MSL)

        Returns:
            True if command sent
        """
        success, result = await self.send_command(
            80,  # MAV_CMD_NAV_ROI
            param1=0,  # ROI mode (location)
            param5=lat,
            param6=lon,
            param7=alt,
        )
        return success

    async def clear_roi(self) -> bool:
        """
        Clear region of interest.

        Returns:
            True if command sent
        """
        success, result = await self.send_command(
            80,  # MAV_CMD_NAV_ROI
            param1=0,
        )
        return success

    async def set_speed(self, speed_ms: float, speed_type: int = 1) -> bool:
        """
        Set target speed.

        Args:
            speed_ms: Speed in m/s
            speed_type: 0=airspeed, 1=groundspeed, 2=climb, 3=descent

        Returns:
            True if command sent
        """
        success, result = await self.send_command(
            178,  # MAV_CMD_DO_CHANGE_SPEED
            param1=speed_type,
            param2=speed_ms,
            param3=-1,  # Throttle (no change)
        )
        return success

    async def pause_mission(self) -> bool:
        """
        Pause current mission.

        Returns:
            True if command sent
        """
        return await self.loiter()

    async def resume_mission(self) -> bool:
        """
        Resume paused mission.

        Returns:
            True if command sent
        """
        return await self.set_mode(FlightMode.AUTO)

    async def reboot_autopilot(self) -> bool:
        """
        Reboot the autopilot (use with caution!).

        Returns:
            True if command sent
        """
        logger.warning("Rebooting autopilot!")

        success, result = await self.send_command(
            246,  # MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN
            param1=1,  # Reboot autopilot
        )
        return success
