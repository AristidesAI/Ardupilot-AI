"""
MAVLink message handling and telemetry processing.

Processes incoming MAVLink messages and maintains vehicle state.
"""

import math
import time
from dataclasses import dataclass, field
from enum import Enum
from typing import Dict, Optional, List

import structlog

logger = structlog.get_logger()


class FlightMode(Enum):
    """ArduCopter flight modes."""
    STABILIZE = 0
    ACRO = 1
    ALT_HOLD = 2
    AUTO = 3
    GUIDED = 4
    LOITER = 5
    RTL = 6
    CIRCLE = 7
    LAND = 9
    DRIFT = 11
    SPORT = 13
    FLIP = 14
    AUTOTUNE = 15
    POSHOLD = 16
    BRAKE = 17
    THROW = 18
    AVOID_ADSB = 19
    GUIDED_NOGPS = 20
    SMART_RTL = 21
    FLOWHOLD = 22
    FOLLOW = 23
    ZIGZAG = 24
    SYSTEMID = 25
    AUTOROTATE = 26
    AUTO_RTL = 27

    @classmethod
    def from_int(cls, value: int) -> Optional["FlightMode"]:
        """Get mode from integer value."""
        for mode in cls:
            if mode.value == value:
                return mode
        return None


class ArmingState(Enum):
    """Vehicle arming state."""
    DISARMED = 0
    ARMED = 1


class GPSFixType(Enum):
    """GPS fix type."""
    NO_GPS = 0
    NO_FIX = 1
    FIX_2D = 2
    FIX_3D = 3
    DGPS = 4
    RTK_FLOAT = 5
    RTK_FIXED = 6


@dataclass
class Position:
    """Vehicle position."""
    latitude: float = 0.0  # Degrees
    longitude: float = 0.0  # Degrees
    altitude_msl: float = 0.0  # Meters above sea level
    altitude_rel: float = 0.0  # Meters above home
    altitude_terrain: float = 0.0  # Meters above terrain
    heading: float = 0.0  # Degrees (0-360)
    timestamp: float = 0.0


@dataclass
class Velocity:
    """Vehicle velocity."""
    vx: float = 0.0  # m/s North
    vy: float = 0.0  # m/s East
    vz: float = 0.0  # m/s Down
    airspeed: float = 0.0  # m/s
    groundspeed: float = 0.0  # m/s
    climb_rate: float = 0.0  # m/s (positive up)
    timestamp: float = 0.0


@dataclass
class Attitude:
    """Vehicle attitude."""
    roll: float = 0.0  # Radians
    pitch: float = 0.0  # Radians
    yaw: float = 0.0  # Radians
    roll_speed: float = 0.0  # rad/s
    pitch_speed: float = 0.0  # rad/s
    yaw_speed: float = 0.0  # rad/s
    timestamp: float = 0.0

    @property
    def roll_deg(self) -> float:
        """Roll in degrees."""
        return math.degrees(self.roll)

    @property
    def pitch_deg(self) -> float:
        """Pitch in degrees."""
        return math.degrees(self.pitch)

    @property
    def yaw_deg(self) -> float:
        """Yaw in degrees."""
        return math.degrees(self.yaw)


@dataclass
class Battery:
    """Battery state."""
    voltage: float = 0.0  # Volts
    current: float = 0.0  # Amps
    consumed_mah: float = 0.0  # mAh consumed
    remaining_percent: int = -1  # -1 = unknown
    cell_count: int = 0
    temperature: float = 0.0  # Celsius
    timestamp: float = 0.0

    @property
    def cell_voltage(self) -> float:
        """Average cell voltage."""
        if self.cell_count > 0:
            return self.voltage / self.cell_count
        return 0.0


@dataclass
class GPS:
    """GPS state."""
    fix_type: GPSFixType = GPSFixType.NO_GPS
    satellites: int = 0
    hdop: float = 99.9
    vdop: float = 99.9
    latitude: float = 0.0
    longitude: float = 0.0
    altitude: float = 0.0
    velocity: float = 0.0
    heading: float = 0.0
    timestamp: float = 0.0


@dataclass
class RCInput:
    """RC input channels."""
    channels: List[int] = field(default_factory=lambda: [0] * 18)
    rssi: int = 0  # 0-255
    timestamp: float = 0.0


@dataclass
class SystemStatus:
    """System status."""
    mode: FlightMode = FlightMode.STABILIZE
    armed: bool = False
    is_flying: bool = False
    ekf_ok: bool = False
    gps_ok: bool = False
    battery_ok: bool = True
    rc_ok: bool = True
    prearm_ok: bool = False
    system_status: int = 0  # MAV_STATE
    cpu_load: int = 0  # 0-1000 (0.1%)
    errors: List[str] = field(default_factory=list)
    timestamp: float = 0.0


@dataclass
class HomePosition:
    """Home/launch position."""
    latitude: float = 0.0
    longitude: float = 0.0
    altitude: float = 0.0
    is_set: bool = False
    timestamp: float = 0.0


@dataclass
class Telemetry:
    """Complete vehicle telemetry."""
    position: Position = field(default_factory=Position)
    velocity: Velocity = field(default_factory=Velocity)
    attitude: Attitude = field(default_factory=Attitude)
    battery: Battery = field(default_factory=Battery)
    gps: GPS = field(default_factory=GPS)
    rc: RCInput = field(default_factory=RCInput)
    system: SystemStatus = field(default_factory=SystemStatus)
    home: HomePosition = field(default_factory=HomePosition)

    def distance_to_home(self) -> float:
        """Calculate distance to home in meters."""
        if not self.home.is_set:
            return 0.0

        return haversine_distance(
            self.position.latitude,
            self.position.longitude,
            self.home.latitude,
            self.home.longitude,
        )

    def to_dict(self) -> Dict:
        """Convert to dictionary for serialization."""
        return {
            "position": {
                "lat": self.position.latitude,
                "lon": self.position.longitude,
                "alt_msl": self.position.altitude_msl,
                "alt_rel": self.position.altitude_rel,
                "heading": self.position.heading,
            },
            "velocity": {
                "groundspeed": self.velocity.groundspeed,
                "airspeed": self.velocity.airspeed,
                "climb_rate": self.velocity.climb_rate,
            },
            "attitude": {
                "roll": self.attitude.roll_deg,
                "pitch": self.attitude.pitch_deg,
                "yaw": self.attitude.yaw_deg,
            },
            "battery": {
                "voltage": self.battery.voltage,
                "current": self.battery.current,
                "remaining": self.battery.remaining_percent,
            },
            "gps": {
                "fix": self.gps.fix_type.name,
                "satellites": self.gps.satellites,
            },
            "system": {
                "mode": self.system.mode.name,
                "armed": self.system.armed,
                "flying": self.system.is_flying,
            },
        }


class MessageHandler:
    """
    Handles MAVLink message processing and state tracking.
    """

    def __init__(self):
        """Initialize message handler."""
        self._telemetry = Telemetry()
        self._message_counts: Dict[str, int] = {}
        self._last_update = time.time()

    @property
    def telemetry(self) -> Telemetry:
        """Get current telemetry."""
        return self._telemetry

    def handle_message(self, msg):
        """
        Process incoming MAVLink message.

        Args:
            msg: MAVLink message
        """
        msg_type = msg.get_type()
        self._message_counts[msg_type] = self._message_counts.get(msg_type, 0) + 1

        handler = getattr(self, f"_handle_{msg_type.lower()}", None)
        if handler:
            try:
                handler(msg)
            except Exception as e:
                logger.error("Message handler error", msg_type=msg_type, error=str(e))

    def _handle_heartbeat(self, msg):
        """Handle HEARTBEAT message."""
        now = time.time()

        # Extract flight mode
        mode = FlightMode.from_int(msg.custom_mode)
        if mode:
            self._telemetry.system.mode = mode

        # Extract armed state
        self._telemetry.system.armed = (msg.base_mode & 128) != 0

        # System status
        self._telemetry.system.system_status = msg.system_status
        self._telemetry.system.timestamp = now

        # Determine if flying
        self._telemetry.system.is_flying = (
            self._telemetry.system.armed and
            self._telemetry.position.altitude_rel > 0.5
        )

    def _handle_global_position_int(self, msg):
        """Handle GLOBAL_POSITION_INT message."""
        now = time.time()
        self._telemetry.position.latitude = msg.lat / 1e7
        self._telemetry.position.longitude = msg.lon / 1e7
        self._telemetry.position.altitude_msl = msg.alt / 1000.0
        self._telemetry.position.altitude_rel = msg.relative_alt / 1000.0
        self._telemetry.position.heading = msg.hdg / 100.0
        self._telemetry.position.timestamp = now

        # Velocity from this message
        self._telemetry.velocity.vx = msg.vx / 100.0
        self._telemetry.velocity.vy = msg.vy / 100.0
        self._telemetry.velocity.vz = msg.vz / 100.0
        self._telemetry.velocity.groundspeed = math.sqrt(
            self._telemetry.velocity.vx ** 2 +
            self._telemetry.velocity.vy ** 2
        )
        self._telemetry.velocity.climb_rate = -self._telemetry.velocity.vz
        self._telemetry.velocity.timestamp = now

    def _handle_attitude(self, msg):
        """Handle ATTITUDE message."""
        now = time.time()
        self._telemetry.attitude.roll = msg.roll
        self._telemetry.attitude.pitch = msg.pitch
        self._telemetry.attitude.yaw = msg.yaw
        self._telemetry.attitude.roll_speed = msg.rollspeed
        self._telemetry.attitude.pitch_speed = msg.pitchspeed
        self._telemetry.attitude.yaw_speed = msg.yawspeed
        self._telemetry.attitude.timestamp = now

    def _handle_vfr_hud(self, msg):
        """Handle VFR_HUD message."""
        now = time.time()
        self._telemetry.velocity.airspeed = msg.airspeed
        self._telemetry.velocity.groundspeed = msg.groundspeed
        self._telemetry.velocity.climb_rate = msg.climb
        self._telemetry.position.heading = msg.heading
        self._telemetry.velocity.timestamp = now

    def _handle_battery_status(self, msg):
        """Handle BATTERY_STATUS message."""
        now = time.time()

        # Calculate voltage from cell voltages
        voltages = [v for v in msg.voltages if v != 65535]
        if voltages:
            self._telemetry.battery.voltage = sum(voltages) / 1000.0
            self._telemetry.battery.cell_count = len(voltages)

        self._telemetry.battery.current = msg.current_battery / 100.0 if msg.current_battery >= 0 else 0
        self._telemetry.battery.consumed_mah = msg.current_consumed
        self._telemetry.battery.remaining_percent = msg.battery_remaining
        self._telemetry.battery.temperature = msg.temperature / 100.0 if hasattr(msg, 'temperature') else 0
        self._telemetry.battery.timestamp = now

    def _handle_sys_status(self, msg):
        """Handle SYS_STATUS message."""
        now = time.time()

        # Battery from SYS_STATUS
        if msg.voltage_battery > 0:
            self._telemetry.battery.voltage = msg.voltage_battery / 1000.0
        if msg.current_battery >= 0:
            self._telemetry.battery.current = msg.current_battery / 100.0
        if msg.battery_remaining >= 0:
            self._telemetry.battery.remaining_percent = msg.battery_remaining

        # CPU load
        self._telemetry.system.cpu_load = msg.load

        # Sensor status
        self._telemetry.system.gps_ok = (msg.onboard_control_sensors_health & 1) != 0

        self._telemetry.battery.timestamp = now

    def _handle_gps_raw_int(self, msg):
        """Handle GPS_RAW_INT message."""
        now = time.time()
        self._telemetry.gps.fix_type = GPSFixType(min(msg.fix_type, 6))
        self._telemetry.gps.satellites = msg.satellites_visible
        self._telemetry.gps.hdop = msg.eph / 100.0 if msg.eph != 65535 else 99.9
        self._telemetry.gps.vdop = msg.epv / 100.0 if msg.epv != 65535 else 99.9
        self._telemetry.gps.latitude = msg.lat / 1e7
        self._telemetry.gps.longitude = msg.lon / 1e7
        self._telemetry.gps.altitude = msg.alt / 1000.0
        self._telemetry.gps.velocity = msg.vel / 100.0 if msg.vel != 65535 else 0
        self._telemetry.gps.heading = msg.cog / 100.0 if msg.cog != 65535 else 0
        self._telemetry.gps.timestamp = now

        # Update GPS status
        self._telemetry.system.gps_ok = self._telemetry.gps.fix_type.value >= 3

    def _handle_rc_channels(self, msg):
        """Handle RC_CHANNELS message."""
        now = time.time()
        channels = [
            msg.chan1_raw, msg.chan2_raw, msg.chan3_raw, msg.chan4_raw,
            msg.chan5_raw, msg.chan6_raw, msg.chan7_raw, msg.chan8_raw,
            msg.chan9_raw, msg.chan10_raw, msg.chan11_raw, msg.chan12_raw,
            msg.chan13_raw, msg.chan14_raw, msg.chan15_raw, msg.chan16_raw,
            msg.chan17_raw, msg.chan18_raw,
        ]
        self._telemetry.rc.channels = channels
        self._telemetry.rc.rssi = msg.rssi
        self._telemetry.rc.timestamp = now

        # Update RC status
        self._telemetry.system.rc_ok = msg.rssi > 0

    def _handle_home_position(self, msg):
        """Handle HOME_POSITION message."""
        now = time.time()
        self._telemetry.home.latitude = msg.latitude / 1e7
        self._telemetry.home.longitude = msg.longitude / 1e7
        self._telemetry.home.altitude = msg.altitude / 1000.0
        self._telemetry.home.is_set = True
        self._telemetry.home.timestamp = now

    def _handle_ekf_status_report(self, msg):
        """Handle EKF_STATUS_REPORT message."""
        # EKF is OK if flags indicate good health
        self._telemetry.system.ekf_ok = (msg.flags & 0x0F) == 0x0F

    def _handle_statustext(self, msg):
        """Handle STATUSTEXT message."""
        text = msg.text.rstrip('\x00')
        severity = msg.severity

        if severity <= 3:  # ERROR or worse
            self._telemetry.system.errors.append(text)
            # Keep only last 10 errors
            self._telemetry.system.errors = self._telemetry.system.errors[-10:]

        logger.info("StatusText", severity=severity, text=text)

    def get_message_stats(self) -> Dict[str, int]:
        """Get message count statistics."""
        return self._message_counts.copy()


def haversine_distance(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """
    Calculate great-circle distance between two points.

    Args:
        lat1, lon1: First point (degrees)
        lat2, lon2: Second point (degrees)

    Returns:
        Distance in meters
    """
    R = 6371000  # Earth radius in meters

    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    delta_phi = math.radians(lat2 - lat1)
    delta_lambda = math.radians(lon2 - lon1)

    a = (math.sin(delta_phi / 2) ** 2 +
         math.cos(phi1) * math.cos(phi2) *
         math.sin(delta_lambda / 2) ** 2)
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

    return R * c
