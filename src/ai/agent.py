"""
AI Agent for autonomous drone control.

Coordinates all subsystems for autonomous flight operations.
"""

import asyncio
import time
from dataclasses import dataclass
from enum import Enum
from typing import Optional, Callable, List

import structlog

from .state_machine import MissionStateMachine, MissionState
from ..mavlink.connection import MAVLinkConnection
from ..mavlink.messages import MessageHandler, FlightMode
from ..mavlink.commands import CommandSender
from ..flight.controller import FlightController
from ..flight.navigation import Navigator, Waypoint
from ..energy.battery import BatteryManager
from ..energy.predictor import FlightTimePredictor
from ..safety.geofence import Geofence, GeofenceAction
from ..safety.failsafe import FailsafeManager, FailsafeAction
from ..config import DroneConfig

logger = structlog.get_logger()


class AgentState(Enum):
    """Agent operational state."""
    STOPPED = "stopped"
    STARTING = "starting"
    RUNNING = "running"
    PAUSED = "paused"
    STOPPING = "stopping"


@dataclass
class MissionTarget:
    """Mission target location."""
    latitude: float
    longitude: float
    altitude: float
    hold_time_s: float = 0
    action: Optional[str] = None


class DroneAgent:
    """
    Autonomous drone control agent.

    Integrates all subsystems for intelligent flight operations:
    - Flight control and navigation
    - Battery management
    - Safety and geofencing
    - Mission execution
    """

    def __init__(
        self,
        config: Optional[DroneConfig] = None,
        connection_string: str = "udp:127.0.0.1:14550",
    ):
        """
        Initialize drone agent.

        Args:
            config: Drone configuration
            connection_string: MAVLink connection string
        """
        self.config = config or DroneConfig()

        # Core components
        self.connection = MAVLinkConnection(connection_string)
        self.messages = MessageHandler()
        self.commands: Optional[CommandSender] = None
        self.flight: Optional[FlightController] = None

        # Subsystems
        self.navigator = Navigator()
        self.battery = BatteryManager()
        self.predictor: Optional[FlightTimePredictor] = None
        self.geofence = Geofence()
        self.failsafe = FailsafeManager()
        self.state_machine = MissionStateMachine()

        # Agent state
        self._state = AgentState.STOPPED
        self._running = False
        self._update_task: Optional[asyncio.Task] = None
        self._update_rate = self.config.ai.update_rate_hz

        # Mission state
        self._mission_target: Optional[MissionTarget] = None
        self._waypoint_queue: List[Waypoint] = []

        # Callbacks
        self._on_state_change: List[Callable] = []
        self._on_telemetry: List[Callable] = []

    @property
    def state(self) -> AgentState:
        """Get agent state."""
        return self._state

    @property
    def telemetry(self):
        """Get current telemetry."""
        return self.messages.telemetry

    @property
    def is_connected(self) -> bool:
        """Check if connected to vehicle."""
        return self.connection.connected

    @property
    def is_flying(self) -> bool:
        """Check if vehicle is flying."""
        return self.state_machine.is_flying()

    async def start(self) -> bool:
        """
        Start the agent.

        Returns:
            True if started successfully
        """
        if self._state != AgentState.STOPPED:
            logger.warning("Agent already running")
            return False

        logger.info("Starting drone agent")
        self._state = AgentState.STARTING

        # Connect to vehicle
        if not await self.connection.connect():
            logger.error("Failed to connect to vehicle")
            self._state = AgentState.STOPPED
            return False

        # Register message handler
        self.connection.register_all_messages_callback(
            self.messages.handle_message
        )

        # Initialize components
        self.commands = CommandSender(self.connection)
        self.flight = FlightController(self.connection, self.messages)
        self.predictor = FlightTimePredictor(self.battery)

        # Request data streams
        await self.connection.request_all_data_streams()

        # Register failsafe callbacks
        self._setup_failsafe_callbacks()

        # Start update loop
        self._running = True
        self._update_task = asyncio.create_task(self._update_loop())

        self._state = AgentState.RUNNING
        logger.info("Drone agent started")

        return True

    async def stop(self):
        """Stop the agent."""
        if self._state == AgentState.STOPPED:
            return

        logger.info("Stopping drone agent")
        self._state = AgentState.STOPPING
        self._running = False

        # Cancel update task
        if self._update_task:
            self._update_task.cancel()
            try:
                await self._update_task
            except asyncio.CancelledError:
                pass

        # Disconnect
        await self.connection.disconnect()

        self._state = AgentState.STOPPED
        logger.info("Drone agent stopped")

    def pause(self):
        """Pause agent operations."""
        if self._state == AgentState.RUNNING:
            self._state = AgentState.PAUSED
            logger.info("Agent paused")

    def resume(self):
        """Resume agent operations."""
        if self._state == AgentState.PAUSED:
            self._state = AgentState.RUNNING
            logger.info("Agent resumed")

    async def arm_and_takeoff(self, altitude_m: float) -> bool:
        """
        Arm and takeoff to altitude.

        Args:
            altitude_m: Target altitude (meters AGL)

        Returns:
            True if successful
        """
        if not self.flight:
            return False

        # Transition to preflight
        self.state_machine.transition(MissionState.PREFLIGHT, "Arm and takeoff")

        # Arm
        self.state_machine.transition(MissionState.ARMING, "Arming")
        if not await self.flight.arm():
            self.state_machine.transition(MissionState.IDLE, "Arm failed")
            return False

        # Takeoff
        self.state_machine.transition(MissionState.TAKEOFF, "Taking off")
        if not await self.flight.takeoff(altitude_m):
            self.state_machine.transition(MissionState.IDLE, "Takeoff failed")
            return False

        # Wait for takeoff
        if await self.flight.wait_takeoff():
            self.state_machine.transition(MissionState.LOITER, "Takeoff complete")
            return True

        return False

    async def goto(
        self,
        lat: float,
        lon: float,
        alt: Optional[float] = None,
    ) -> bool:
        """
        Go to position.

        Args:
            lat: Target latitude
            lon: Target longitude
            alt: Target altitude (None = current)

        Returns:
            True if command accepted
        """
        if not self.flight:
            return False

        self._mission_target = MissionTarget(
            latitude=lat,
            longitude=lon,
            altitude=alt or self.telemetry.position.altitude_rel,
        )

        self.state_machine.transition(MissionState.TRANSIT, f"Going to {lat:.4f}, {lon:.4f}")
        return await self.flight.goto(lat, lon, alt)

    async def rtl(self) -> bool:
        """
        Return to launch.

        Returns:
            True if RTL initiated
        """
        if not self.flight:
            return False

        self.state_machine.transition(MissionState.RTL, "Return to launch")
        return await self.flight.rtl()

    async def land(self) -> bool:
        """
        Land the drone.

        Returns:
            True if landing initiated
        """
        if not self.flight:
            return False

        self.state_machine.transition(MissionState.LANDING, "Landing")
        return await self.flight.land()

    async def hold(self) -> bool:
        """
        Hold current position.

        Returns:
            True if holding
        """
        if not self.flight:
            return False

        self.state_machine.transition(MissionState.LOITER, "Holding position")
        return await self.flight.hold()

    def set_home_fence(
        self,
        radius_m: float,
        max_altitude_m: float = 120,
    ):
        """
        Set home geofence.

        Args:
            radius_m: Maximum distance from home
            max_altitude_m: Maximum altitude
        """
        telem = self.telemetry
        self.geofence.set_home_fence(
            telem.home.latitude,
            telem.home.longitude,
            radius_m,
            max_altitude_m,
        )

    async def execute_survey(
        self,
        center_lat: float,
        center_lon: float,
        width_m: float,
        height_m: float,
        altitude_m: float,
        spacing_m: float = 10,
    ) -> bool:
        """
        Execute survey pattern.

        Args:
            center_lat: Survey center latitude
            center_lon: Survey center longitude
            width_m: Survey width
            height_m: Survey height
            altitude_m: Survey altitude
            spacing_m: Line spacing

        Returns:
            True if survey started
        """
        # Generate waypoints
        waypoints = self.navigator.create_survey_pattern(
            center_lat, center_lon,
            width_m, height_m,
            altitude_m, spacing_m,
        )

        self._waypoint_queue = waypoints
        self.state_machine.transition(MissionState.SURVEY, "Survey mission")

        # Start with first waypoint
        if waypoints:
            wp = waypoints[0]
            return await self.goto(wp.latitude, wp.longitude, wp.altitude)

        return False

    def _setup_failsafe_callbacks(self):
        """Setup failsafe action callbacks."""

        async def on_rtl(action):
            logger.warning("Failsafe RTL triggered")
            await self.rtl()

        async def on_land(action):
            logger.warning("Failsafe land triggered")
            await self.land()

        # Register callbacks (sync wrappers for async)
        def rtl_wrapper(action):
            asyncio.create_task(on_rtl(action))

        def land_wrapper(action):
            asyncio.create_task(on_land(action))

        self.failsafe.register_action_callback(FailsafeAction.RTL, rtl_wrapper)
        self.failsafe.register_action_callback(FailsafeAction.LAND, land_wrapper)

    async def _update_loop(self):
        """Main update loop."""
        interval = 1.0 / self._update_rate

        while self._running:
            try:
                if self._state == AgentState.RUNNING:
                    await self._update()

                await asyncio.sleep(interval)

            except Exception as e:
                logger.error("Update loop error", error=str(e))
                await asyncio.sleep(1.0)

    async def _update(self):
        """Single update cycle."""
        telem = self.telemetry

        # Update battery manager
        self.battery.update(
            voltage=telem.battery.voltage,
            current=telem.battery.current,
            consumed_mah=telem.battery.consumed_mah,
            remaining_percent=telem.battery.remaining_percent,
            cell_count=telem.battery.cell_count,
        )

        # Update flight predictor
        if self.predictor:
            self.predictor.update(
                groundspeed_ms=telem.velocity.groundspeed,
                current_a=telem.battery.current,
                distance_to_home_m=telem.distance_to_home(),
            )

        # Check geofence
        fence_status = self.geofence.check(
            telem.position.latitude,
            telem.position.longitude,
            telem.position.altitude_rel,
        )

        # Update failsafe manager
        self.failsafe.update(
            battery_percent=self.battery.soc,
            gps_ok=telem.system.gps_ok,
            link_ok=self.connection.connected,
            ekf_ok=telem.system.ekf_ok,
            in_geofence=fence_status.is_inside,
        )

        # Check for RTL condition
        if self.predictor:
            should_rtl, reason = self.predictor.should_rtl(
                telem.distance_to_home(),
            )
            if should_rtl and self.state_machine.state not in (
                MissionState.RTL, MissionState.LANDING, MissionState.LANDED
            ):
                logger.warning("Auto RTL triggered", reason=reason)
                await self.rtl()

        # Process waypoint queue
        await self._process_waypoints()

        # Call telemetry callbacks
        for callback in self._on_telemetry:
            try:
                callback(telem)
            except Exception as e:
                logger.error("Telemetry callback error", error=str(e))

    async def _process_waypoints(self):
        """Process waypoint queue."""
        if not self._waypoint_queue:
            return

        if self.state_machine.state != MissionState.SURVEY:
            return

        telem = self.telemetry
        current_wp = self._waypoint_queue[0]

        # Check if reached waypoint
        distance = self.navigator.distance_to_waypoint(
            telem.position.latitude,
            telem.position.longitude,
            current_wp,
        )

        if distance < current_wp.accept_radius_m:
            # Remove completed waypoint
            self._waypoint_queue.pop(0)
            logger.info(
                "Waypoint reached",
                remaining=len(self._waypoint_queue),
            )

            # Go to next waypoint
            if self._waypoint_queue:
                wp = self._waypoint_queue[0]
                await self.goto(wp.latitude, wp.longitude, wp.altitude)
            else:
                # Survey complete
                self.state_machine.transition(MissionState.LOITER, "Survey complete")

    def on_state_change(self, callback: Callable):
        """Register state change callback."""
        self._on_state_change.append(callback)

    def on_telemetry(self, callback: Callable):
        """Register telemetry callback."""
        self._on_telemetry.append(callback)

    def get_status(self) -> dict:
        """Get comprehensive status."""
        return {
            "agent": {
                "state": self._state.value,
                "connected": self.is_connected,
            },
            "mission": self.state_machine.get_status_dict(),
            "battery": self.battery.get_status_dict(),
            "geofence": self.geofence.get_status_dict(),
            "failsafe": self.failsafe.get_status_dict(),
            "telemetry": self.telemetry.to_dict() if self.telemetry else None,
        }
