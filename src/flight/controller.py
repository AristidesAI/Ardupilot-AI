"""
High-level flight controller for drone operations.

Provides simplified interface for common flight operations.
"""

import asyncio
import time
from dataclasses import dataclass
from enum import Enum
from typing import Optional, Tuple

import structlog

from ..mavlink.connection import MAVLinkConnection
from ..mavlink.messages import MessageHandler, FlightMode, Telemetry
from ..mavlink.commands import CommandSender

logger = structlog.get_logger()


class FlightState(Enum):
    """High-level flight state."""
    DISARMED = "disarmed"
    ARMED = "armed"
    TAKING_OFF = "taking_off"
    FLYING = "flying"
    LANDING = "landing"
    RETURNING = "returning"
    EMERGENCY = "emergency"


@dataclass
class FlightStatus:
    """Current flight status."""
    state: FlightState = FlightState.DISARMED
    mode: FlightMode = FlightMode.STABILIZE
    altitude_m: float = 0.0
    groundspeed_ms: float = 0.0
    heading_deg: float = 0.0
    distance_to_home_m: float = 0.0
    is_armed: bool = False
    gps_ok: bool = False
    battery_ok: bool = True
    timestamp: float = 0.0


class FlightController:
    """
    High-level flight control interface.

    Provides simplified methods for:
    - Arming/disarming
    - Takeoff/landing
    - Navigation
    - Mode control
    """

    def __init__(
        self,
        connection: MAVLinkConnection,
        message_handler: MessageHandler,
    ):
        """
        Initialize flight controller.

        Args:
            connection: MAVLink connection
            message_handler: Message handler for telemetry
        """
        self.connection = connection
        self.messages = message_handler
        self.commands = CommandSender(connection)

        self._state = FlightState.DISARMED
        self._target_altitude = 0.0
        self._takeoff_complete = asyncio.Event()
        self._landing_complete = asyncio.Event()

    @property
    def telemetry(self) -> Telemetry:
        """Get current telemetry."""
        return self.messages.telemetry

    @property
    def state(self) -> FlightState:
        """Get current flight state."""
        return self._state

    @property
    def mode(self) -> FlightMode:
        """Get current flight mode."""
        return self.telemetry.system.mode

    @property
    def is_armed(self) -> bool:
        """Check if armed."""
        return self.telemetry.system.armed

    @property
    def is_flying(self) -> bool:
        """Check if flying."""
        return self.telemetry.system.is_flying

    @property
    def altitude(self) -> float:
        """Get current altitude (meters AGL)."""
        return self.telemetry.position.altitude_rel

    @property
    def position(self) -> Tuple[float, float]:
        """Get current position (lat, lon)."""
        pos = self.telemetry.position
        return pos.latitude, pos.longitude

    def get_status(self) -> FlightStatus:
        """Get comprehensive flight status."""
        telem = self.telemetry

        return FlightStatus(
            state=self._state,
            mode=telem.system.mode,
            altitude_m=telem.position.altitude_rel,
            groundspeed_ms=telem.velocity.groundspeed,
            heading_deg=telem.position.heading,
            distance_to_home_m=telem.distance_to_home(),
            is_armed=telem.system.armed,
            gps_ok=telem.system.gps_ok,
            battery_ok=telem.battery.remaining_percent > 20,
            timestamp=time.time(),
        )

    async def arm(self, preflight_check: bool = True) -> bool:
        """
        Arm the drone.

        Args:
            preflight_check: Run preflight checks before arming

        Returns:
            True if armed successfully
        """
        if self.is_armed:
            logger.info("Already armed")
            return True

        if preflight_check:
            if not self._preflight_check():
                return False

        logger.info("Arming drone")

        # Switch to GUIDED mode for arming
        if not await self.commands.set_mode(FlightMode.GUIDED):
            logger.error("Failed to set GUIDED mode")
            return False

        await asyncio.sleep(0.5)

        # Arm
        if await self.commands.arm():
            self._state = FlightState.ARMED
            logger.info("Drone armed successfully")
            return True

        return False

    async def disarm(self, force: bool = False) -> bool:
        """
        Disarm the drone.

        Args:
            force: Force disarm even if flying

        Returns:
            True if disarmed successfully
        """
        if not self.is_armed:
            logger.info("Already disarmed")
            return True

        if self.is_flying and not force:
            logger.error("Cannot disarm while flying")
            return False

        if await self.commands.disarm(force=force):
            self._state = FlightState.DISARMED
            logger.info("Drone disarmed")
            return True

        return False

    async def takeoff(self, altitude_m: float) -> bool:
        """
        Take off to specified altitude.

        Args:
            altitude_m: Target altitude (meters AGL)

        Returns:
            True if takeoff initiated
        """
        if not self.is_armed:
            logger.error("Cannot takeoff - not armed")
            return False

        if self.is_flying:
            logger.warning("Already flying")
            return True

        logger.info("Taking off", altitude=altitude_m)
        self._target_altitude = altitude_m
        self._state = FlightState.TAKING_OFF
        self._takeoff_complete.clear()

        # Send takeoff command
        if not await self.commands.takeoff(altitude_m):
            self._state = FlightState.ARMED
            return False

        # Monitor takeoff
        asyncio.create_task(self._monitor_takeoff(altitude_m))

        return True

    async def wait_takeoff(self, timeout: float = 60.0) -> bool:
        """
        Wait for takeoff to complete.

        Args:
            timeout: Timeout in seconds

        Returns:
            True if takeoff completed
        """
        try:
            await asyncio.wait_for(
                self._takeoff_complete.wait(),
                timeout=timeout,
            )
            return True
        except asyncio.TimeoutError:
            logger.error("Takeoff timeout")
            return False

    async def land(self) -> bool:
        """
        Land the drone.

        Returns:
            True if land command accepted
        """
        if not self.is_flying:
            logger.warning("Not flying")
            return True

        logger.info("Landing")
        self._state = FlightState.LANDING
        self._landing_complete.clear()

        if not await self.commands.land():
            return False

        # Monitor landing
        asyncio.create_task(self._monitor_landing())

        return True

    async def wait_landing(self, timeout: float = 120.0) -> bool:
        """
        Wait for landing to complete.

        Args:
            timeout: Timeout in seconds

        Returns:
            True if landing completed
        """
        try:
            await asyncio.wait_for(
                self._landing_complete.wait(),
                timeout=timeout,
            )
            return True
        except asyncio.TimeoutError:
            logger.error("Landing timeout")
            return False

    async def rtl(self) -> bool:
        """
        Return to launch.

        Returns:
            True if RTL initiated
        """
        logger.info("Returning to launch")
        self._state = FlightState.RETURNING

        if await self.commands.rtl():
            logger.info("RTL initiated")
            return True

        return False

    async def goto(
        self,
        lat: float,
        lon: float,
        alt: Optional[float] = None,
        speed: float = 0,
    ) -> bool:
        """
        Go to position.

        Args:
            lat: Target latitude
            lon: Target longitude
            alt: Target altitude (None = current)
            speed: Ground speed (0 = default)

        Returns:
            True if command accepted
        """
        if alt is None:
            alt = self.altitude

        logger.info("Going to", lat=lat, lon=lon, alt=alt)
        self._state = FlightState.FLYING

        return await self.commands.goto(lat, lon, alt, speed)

    async def fly_velocity(
        self,
        forward_ms: float = 0,
        right_ms: float = 0,
        down_ms: float = 0,
        yaw_rate_dps: float = 0,
    ) -> bool:
        """
        Fly at specified velocity.

        Args:
            forward_ms: Forward velocity (m/s)
            right_ms: Right velocity (m/s)
            down_ms: Down velocity (m/s)
            yaw_rate_dps: Yaw rate (deg/s)

        Returns:
            True if command sent
        """
        return await self.commands.set_velocity(
            forward_ms, right_ms, down_ms, yaw_rate_dps
        )

    async def hold(self) -> bool:
        """
        Hold current position.

        Returns:
            True if loiter mode set
        """
        logger.info("Holding position")
        return await self.commands.loiter()

    async def set_altitude(self, altitude_m: float) -> bool:
        """
        Change altitude.

        Args:
            altitude_m: Target altitude (meters AGL)

        Returns:
            True if command sent
        """
        logger.info("Setting altitude", altitude=altitude_m)
        return await self.commands.set_altitude(altitude_m)

    async def set_heading(
        self,
        heading_deg: float,
        rate_dps: float = 0,
    ) -> bool:
        """
        Set heading.

        Args:
            heading_deg: Target heading (degrees)
            rate_dps: Yaw rate (0 = default)

        Returns:
            True if command sent
        """
        logger.info("Setting heading", heading=heading_deg)
        return await self.commands.set_yaw(heading_deg, rate_dps)

    async def set_mode(self, mode: FlightMode) -> bool:
        """
        Set flight mode.

        Args:
            mode: Target mode

        Returns:
            True if mode set
        """
        return await self.commands.set_mode(mode)

    def _preflight_check(self) -> bool:
        """Run preflight checks."""
        telem = self.telemetry

        errors = []

        # GPS check
        if not telem.system.gps_ok:
            errors.append("GPS not ready")

        # Battery check
        if telem.battery.remaining_percent >= 0:
            if telem.battery.remaining_percent < 20:
                errors.append(f"Battery low: {telem.battery.remaining_percent}%")

        # EKF check
        if not telem.system.ekf_ok:
            errors.append("EKF not healthy")

        if errors:
            for error in errors:
                logger.error("Preflight check failed", error=error)
            return False

        logger.info("Preflight checks passed")
        return True

    async def _monitor_takeoff(self, target_alt: float):
        """Monitor takeoff progress."""
        start_time = time.time()
        threshold = target_alt * 0.9  # 90% of target

        while time.time() - start_time < 60:
            if self.altitude >= threshold:
                self._state = FlightState.FLYING
                self._takeoff_complete.set()
                logger.info(
                    "Takeoff complete",
                    altitude=self.altitude,
                    target=target_alt,
                )
                return

            await asyncio.sleep(0.5)

        logger.warning("Takeoff monitoring timeout")

    async def _monitor_landing(self):
        """Monitor landing progress."""
        start_time = time.time()

        while time.time() - start_time < 120:
            if self.altitude < 0.5 and not self.is_armed:
                self._state = FlightState.DISARMED
                self._landing_complete.set()
                logger.info("Landing complete")
                return

            await asyncio.sleep(0.5)

        logger.warning("Landing monitoring timeout")

    async def emergency_stop(self) -> bool:
        """
        Emergency stop (land immediately).

        Returns:
            True if command sent
        """
        logger.critical("EMERGENCY STOP")
        self._state = FlightState.EMERGENCY
        return await self.commands.land()
