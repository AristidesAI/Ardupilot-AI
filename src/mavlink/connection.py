"""
MAVLink connection handler for ArduPilot drone.

Manages connection, heartbeat, and message routing.
"""

import asyncio
import time
from dataclasses import dataclass
from typing import Callable, Dict, List, Optional, Any

import structlog
from pymavlink import mavutil

logger = structlog.get_logger()


@dataclass
class ConnectionStatus:
    """Connection status information."""
    connected: bool = False
    target_system: int = 0
    target_component: int = 0
    last_heartbeat: float = 0.0
    message_rate: float = 0.0
    link_quality: float = 1.0


class MAVLinkConnection:
    """
    Manages MAVLink connection to ArduPilot flight controller.

    Supports UDP, TCP, and serial connections.
    """

    def __init__(
        self,
        connection_string: str = "udp:127.0.0.1:14550",
        source_system: int = 255,
        source_component: int = 0,
    ):
        """
        Initialize MAVLink connection.

        Args:
            connection_string: MAVLink connection string
            source_system: Source system ID for outgoing messages
            source_component: Source component ID for outgoing messages
        """
        self.connection_string = connection_string
        self.source_system = source_system
        self.source_component = source_component

        self._connection: Optional[mavutil.mavlink_connection] = None
        self._connected = False
        self._target_system = 0
        self._target_component = 0

        # Message callbacks
        self._message_callbacks: Dict[str, List[Callable]] = {}
        self._all_message_callbacks: List[Callable] = []

        # Statistics
        self._last_heartbeat_time = 0.0
        self._message_count = 0
        self._message_rate_window: List[float] = []

        # Background tasks
        self._receive_task: Optional[asyncio.Task] = None
        self._heartbeat_task: Optional[asyncio.Task] = None
        self._running = False

    @property
    def connected(self) -> bool:
        """Check if connected."""
        return self._connected

    @property
    def target_system(self) -> int:
        """Get target system ID."""
        return self._target_system

    @property
    def target_component(self) -> int:
        """Get target component ID."""
        return self._target_component

    @property
    def heartbeat(self) -> Optional[Dict]:
        """Get last heartbeat info."""
        if not self._connected:
            return None
        return {
            "system_id": self._target_system,
            "component_id": self._target_component,
            "last_time": self._last_heartbeat_time,
            "age": time.time() - self._last_heartbeat_time,
        }

    @property
    def mav(self):
        """Get MAVLink connection object."""
        return self._connection

    async def connect(self, timeout: float = 30.0) -> bool:
        """
        Establish MAVLink connection.

        Args:
            timeout: Connection timeout in seconds

        Returns:
            True if connected successfully
        """
        try:
            logger.info(
                "Connecting to MAVLink",
                connection_string=self.connection_string,
            )

            # Create connection
            self._connection = mavutil.mavlink_connection(
                self.connection_string,
                source_system=self.source_system,
                source_component=self.source_component,
            )

            # Wait for heartbeat
            start_time = time.time()
            while time.time() - start_time < timeout:
                msg = self._connection.recv_match(
                    type="HEARTBEAT",
                    blocking=True,
                    timeout=1.0,
                )
                if msg and msg.get_srcSystem() != self.source_system:
                    self._target_system = msg.get_srcSystem()
                    self._target_component = msg.get_srcComponent()
                    self._last_heartbeat_time = time.time()
                    self._connected = True

                    logger.info(
                        "Connected to vehicle",
                        system_id=self._target_system,
                        component_id=self._target_component,
                        vehicle_type=msg.type,
                        autopilot=msg.autopilot,
                    )

                    # Start background tasks
                    self._running = True
                    self._receive_task = asyncio.create_task(self._receive_loop())
                    self._heartbeat_task = asyncio.create_task(self._heartbeat_loop())

                    return True

            logger.error("Connection timeout - no heartbeat received")
            return False

        except Exception as e:
            logger.error("Connection failed", error=str(e))
            return False

    async def disconnect(self):
        """Disconnect from vehicle."""
        self._running = False

        if self._receive_task:
            self._receive_task.cancel()
            try:
                await self._receive_task
            except asyncio.CancelledError:
                pass

        if self._heartbeat_task:
            self._heartbeat_task.cancel()
            try:
                await self._heartbeat_task
            except asyncio.CancelledError:
                pass

        if self._connection:
            self._connection.close()

        self._connected = False
        logger.info("Disconnected from vehicle")

    def register_message_callback(
        self,
        msg_type: str,
        callback: Callable,
    ):
        """
        Register callback for specific message type.

        Args:
            msg_type: MAVLink message type (e.g., "HEARTBEAT")
            callback: Function to call with message
        """
        if msg_type not in self._message_callbacks:
            self._message_callbacks[msg_type] = []
        self._message_callbacks[msg_type].append(callback)

    def register_all_messages_callback(self, callback: Callable):
        """
        Register callback for all messages.

        Args:
            callback: Function to call with each message
        """
        self._all_message_callbacks.append(callback)

    async def send_message(self, msg) -> bool:
        """
        Send MAVLink message.

        Args:
            msg: MAVLink message to send

        Returns:
            True if sent successfully
        """
        if not self._connected or not self._connection:
            return False

        try:
            self._connection.mav.send(msg)
            return True
        except Exception as e:
            logger.error("Failed to send message", error=str(e))
            return False

    async def send_command_long(
        self,
        command: int,
        param1: float = 0,
        param2: float = 0,
        param3: float = 0,
        param4: float = 0,
        param5: float = 0,
        param6: float = 0,
        param7: float = 0,
        confirmation: int = 0,
    ) -> bool:
        """
        Send COMMAND_LONG message.

        Args:
            command: MAV_CMD command ID
            param1-7: Command parameters
            confirmation: Confirmation count

        Returns:
            True if sent successfully
        """
        if not self._connected or not self._connection:
            return False

        try:
            self._connection.mav.command_long_send(
                self._target_system,
                self._target_component,
                command,
                confirmation,
                param1, param2, param3, param4, param5, param6, param7,
            )
            return True
        except Exception as e:
            logger.error("Failed to send command", command=command, error=str(e))
            return False

    async def set_mode(self, mode: int) -> bool:
        """
        Set flight mode.

        Args:
            mode: Flight mode number

        Returns:
            True if mode change initiated
        """
        if not self._connected:
            return False

        # MAV_CMD_DO_SET_MODE
        return await self.send_command_long(
            command=176,  # MAV_CMD_DO_SET_MODE
            param1=1,     # MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
            param2=mode,
        )

    async def arm(self, force: bool = False) -> bool:
        """
        Arm the vehicle.

        Args:
            force: Force arming (bypass checks)

        Returns:
            True if arm command sent
        """
        if not self._connected:
            return False

        # MAV_CMD_COMPONENT_ARM_DISARM
        return await self.send_command_long(
            command=400,
            param1=1,  # 1 = arm
            param2=21196 if force else 0,  # Force arm magic number
        )

    async def disarm(self, force: bool = False) -> bool:
        """
        Disarm the vehicle.

        Args:
            force: Force disarming

        Returns:
            True if disarm command sent
        """
        if not self._connected:
            return False

        return await self.send_command_long(
            command=400,
            param1=0,  # 0 = disarm
            param2=21196 if force else 0,
        )

    async def takeoff(self, altitude_m: float) -> bool:
        """
        Command takeoff.

        Args:
            altitude_m: Target altitude in meters

        Returns:
            True if takeoff command sent
        """
        if not self._connected:
            return False

        # MAV_CMD_NAV_TAKEOFF
        return await self.send_command_long(
            command=22,
            param7=altitude_m,
        )

    async def land(self) -> bool:
        """
        Command landing.

        Returns:
            True if land command sent
        """
        if not self._connected:
            return False

        # MAV_CMD_NAV_LAND
        return await self.send_command_long(command=21)

    async def rtl(self) -> bool:
        """
        Return to launch.

        Returns:
            True if RTL command sent
        """
        # Set mode to RTL (mode 6 for copter)
        return await self.set_mode(6)

    async def goto(
        self,
        lat: float,
        lon: float,
        alt: float,
        groundspeed: float = 0,
    ) -> bool:
        """
        Go to position.

        Args:
            lat: Latitude in degrees
            lon: Longitude in degrees
            alt: Altitude in meters (relative)
            groundspeed: Ground speed in m/s (0 = default)

        Returns:
            True if command sent
        """
        if not self._connected or not self._connection:
            return False

        try:
            self._connection.mav.mission_item_int_send(
                self._target_system,
                self._target_component,
                0,  # seq
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                2,  # current = 2 means guided mode goto
                1,  # autocontinue
                0, 0, 0, 0,  # params 1-4
                int(lat * 1e7),
                int(lon * 1e7),
                alt,
            )
            return True
        except Exception as e:
            logger.error("Goto failed", error=str(e))
            return False

    async def set_velocity(
        self,
        vx: float,
        vy: float,
        vz: float,
        yaw_rate: float = 0,
    ) -> bool:
        """
        Set velocity in body frame.

        Args:
            vx: Forward velocity (m/s)
            vy: Right velocity (m/s)
            vz: Down velocity (m/s)
            yaw_rate: Yaw rate (rad/s)

        Returns:
            True if command sent
        """
        if not self._connected or not self._connection:
            return False

        try:
            self._connection.mav.set_position_target_local_ned_send(
                0,  # time_boot_ms
                self._target_system,
                self._target_component,
                mavutil.mavlink.MAV_FRAME_BODY_NED,
                0b0000011111000111,  # type_mask (velocity only)
                0, 0, 0,  # position (ignored)
                vx, vy, vz,
                0, 0, 0,  # acceleration (ignored)
                0, yaw_rate,
            )
            return True
        except Exception as e:
            logger.error("Set velocity failed", error=str(e))
            return False

    async def set_yaw(self, heading_deg: float, relative: bool = False) -> bool:
        """
        Set yaw heading.

        Args:
            heading_deg: Target heading in degrees
            relative: If True, heading is relative to current

        Returns:
            True if command sent
        """
        if not self._connected:
            return False

        # MAV_CMD_CONDITION_YAW
        return await self.send_command_long(
            command=115,
            param1=heading_deg,
            param2=0,  # yaw speed (0 = default)
            param3=1 if heading_deg >= 0 else -1,  # direction
            param4=1 if relative else 0,  # 0 = absolute, 1 = relative
        )

    async def request_data_stream(
        self,
        stream_id: int,
        rate_hz: int,
        start: bool = True,
    ):
        """
        Request MAVLink data stream.

        Args:
            stream_id: MAV_DATA_STREAM ID
            rate_hz: Stream rate in Hz
            start: Start or stop stream
        """
        if not self._connected or not self._connection:
            return

        self._connection.mav.request_data_stream_send(
            self._target_system,
            self._target_component,
            stream_id,
            rate_hz,
            1 if start else 0,
        )

    async def request_all_data_streams(self, rate_hz: int = 4):
        """
        Request all standard data streams.

        Args:
            rate_hz: Stream rate in Hz
        """
        streams = [
            (mavutil.mavlink.MAV_DATA_STREAM_ALL, rate_hz),
            (mavutil.mavlink.MAV_DATA_STREAM_POSITION, rate_hz * 2),
            (mavutil.mavlink.MAV_DATA_STREAM_EXTRA1, rate_hz * 2),
            (mavutil.mavlink.MAV_DATA_STREAM_EXTRA2, rate_hz),
            (mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS, 2),
        ]

        for stream_id, rate in streams:
            await self.request_data_stream(stream_id, rate)
            await asyncio.sleep(0.1)

    def get_status(self) -> ConnectionStatus:
        """Get connection status."""
        # Calculate message rate
        now = time.time()
        self._message_rate_window = [
            t for t in self._message_rate_window
            if now - t < 1.0
        ]
        rate = len(self._message_rate_window)

        # Calculate link quality based on heartbeat age
        hb_age = now - self._last_heartbeat_time if self._last_heartbeat_time else float('inf')
        link_quality = max(0, min(1, 1 - (hb_age / 5.0)))

        return ConnectionStatus(
            connected=self._connected,
            target_system=self._target_system,
            target_component=self._target_component,
            last_heartbeat=self._last_heartbeat_time,
            message_rate=rate,
            link_quality=link_quality,
        )

    async def _receive_loop(self):
        """Background task to receive messages."""
        while self._running:
            try:
                if self._connection:
                    msg = self._connection.recv_match(blocking=False)
                    if msg:
                        self._process_message(msg)
                        self._message_count += 1
                        self._message_rate_window.append(time.time())

                await asyncio.sleep(0.001)  # 1ms loop

            except Exception as e:
                logger.error("Receive error", error=str(e))
                await asyncio.sleep(0.1)

    async def _heartbeat_loop(self):
        """Background task to send heartbeats."""
        while self._running:
            try:
                if self._connection:
                    self._connection.mav.heartbeat_send(
                        mavutil.mavlink.MAV_TYPE_GCS,
                        mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                        0, 0, 0,
                    )
                await asyncio.sleep(1.0)

            except Exception as e:
                logger.error("Heartbeat error", error=str(e))
                await asyncio.sleep(1.0)

    def _process_message(self, msg):
        """Process received message."""
        msg_type = msg.get_type()

        # Update heartbeat time
        if msg_type == "HEARTBEAT":
            if msg.get_srcSystem() == self._target_system:
                self._last_heartbeat_time = time.time()

        # Call type-specific callbacks
        if msg_type in self._message_callbacks:
            for callback in self._message_callbacks[msg_type]:
                try:
                    callback(msg)
                except Exception as e:
                    logger.error(
                        "Callback error",
                        msg_type=msg_type,
                        error=str(e),
                    )

        # Call all-message callbacks
        for callback in self._all_message_callbacks:
            try:
                callback(msg)
            except Exception as e:
                logger.error("Callback error", error=str(e))
