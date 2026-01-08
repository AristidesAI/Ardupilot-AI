"""
ArduPilot Drone AI - Configuration Module

Manages all configuration for the autonomous drone AI agent.
"""

from dataclasses import dataclass, field
from enum import Enum
from pathlib import Path
from typing import Optional
import yaml


class DroneType(Enum):
    """Supported drone types."""
    QUADCOPTER = "quadcopter"
    HEXACOPTER = "hexacopter"
    OCTOCOPTER = "octocopter"
    VTOL = "vtol"


@dataclass
class MAVLinkConfig:
    """MAVLink connection configuration."""
    connection_string: str = "udp:127.0.0.1:14550"
    baud_rate: int = 57600
    source_system: int = 255
    source_component: int = 0
    heartbeat_interval: float = 1.0
    timeout: float = 30.0
    retry_count: int = 3


@dataclass
class FlightConfig:
    """Flight parameters configuration."""
    # Altitude limits (meters AGL)
    altitude_min_m: float = 2.0
    altitude_max_m: float = 120.0  # Common regulatory limit
    altitude_default_m: float = 30.0

    # Speed limits (m/s)
    speed_max_horizontal_ms: float = 15.0
    speed_max_vertical_ms: float = 3.0
    speed_cruise_ms: float = 10.0
    speed_rtl_ms: float = 8.0

    # Acceleration limits (m/s²)
    accel_max_horizontal: float = 3.0
    accel_max_vertical: float = 2.5

    # Navigation
    waypoint_radius_m: float = 2.0
    loiter_radius_m: float = 5.0
    heading_hold_deg: float = 5.0

    # Attitude limits
    max_tilt_deg: float = 35.0
    max_yaw_rate_dps: float = 90.0


@dataclass
class BatteryConfig:
    """Battery configuration."""
    # Cell configuration
    cell_count: int = 6  # 6S LiPo
    cell_voltage_min: float = 3.3  # Minimum safe
    cell_voltage_max: float = 4.2  # Fully charged
    cell_voltage_nominal: float = 3.7

    # Capacity
    capacity_mah: float = 10000.0  # 10Ah

    # Thresholds (%)
    critical_soc_percent: float = 15.0
    low_soc_percent: float = 25.0
    rtl_soc_percent: float = 30.0

    # Failsafe voltages
    critical_voltage: float = 19.8  # 3.3V * 6
    low_voltage: float = 21.6  # 3.6V * 6

    @property
    def capacity_wh(self) -> float:
        """Get capacity in watt-hours."""
        return (self.capacity_mah / 1000) * (self.cell_count * self.cell_voltage_nominal)


@dataclass
class SafetyConfig:
    """Safety and failsafe configuration."""
    # Geofence
    geofence_enabled: bool = True
    geofence_radius_m: float = 1000.0
    geofence_altitude_max_m: float = 120.0
    geofence_action: str = "rtl"  # rtl, land, loiter

    # Link loss
    link_loss_timeout_s: float = 5.0
    link_loss_action: str = "rtl"  # rtl, land, continue

    # Battery failsafe
    low_battery_action: str = "rtl"
    critical_battery_action: str = "land"

    # GPS failsafe
    gps_loss_action: str = "land"
    min_satellites: int = 6

    # Motor/ESC failsafe
    motor_fail_action: str = "land"

    # Maximum flight time (seconds)
    max_flight_time_s: float = 1800.0  # 30 minutes


@dataclass
class MissionConfig:
    """Mission planning configuration."""
    # Default mission parameters
    default_altitude_m: float = 30.0
    default_speed_ms: float = 10.0

    # Survey parameters
    survey_overlap_percent: float = 70.0
    survey_sidelap_percent: float = 65.0

    # Waypoint behavior
    auto_continue: bool = True
    loiter_time_s: float = 0.0

    # RTL behavior
    rtl_altitude_m: float = 40.0
    rtl_speed_ms: float = 8.0


@dataclass
class FPVConfig:
    """FPV and camera configuration."""
    # Video
    video_enabled: bool = True
    video_resolution: str = "1080p"
    video_fps: int = 60
    video_bitrate_mbps: float = 25.0

    # Camera
    camera_tilt_min_deg: float = -90.0
    camera_tilt_max_deg: float = 30.0

    # OSD
    osd_enabled: bool = True

    # Recording
    recording_enabled: bool = True
    recording_path: str = "/recordings"


@dataclass
class AIConfig:
    """AI agent configuration."""
    # Decision making
    update_rate_hz: float = 10.0
    planning_horizon_s: float = 30.0

    # Obstacle avoidance
    obstacle_avoidance_enabled: bool = True
    min_obstacle_distance_m: float = 5.0

    # Autonomous behavior
    auto_rtl_on_low_battery: bool = True
    auto_avoid_no_fly_zones: bool = True

    # Tracking
    target_tracking_enabled: bool = False
    tracking_prediction_s: float = 2.0


@dataclass
class HomeLocation:
    """Home/launch location."""
    latitude: float = 0.0
    longitude: float = 0.0
    altitude_m: float = 0.0
    set_from_gps: bool = True


@dataclass
class DroneConfig:
    """Complete drone configuration."""
    # Drone identification
    name: str = "DroneAI"
    drone_type: DroneType = DroneType.QUADCOPTER

    # Sub-configurations
    mavlink: MAVLinkConfig = field(default_factory=MAVLinkConfig)
    flight: FlightConfig = field(default_factory=FlightConfig)
    battery: BatteryConfig = field(default_factory=BatteryConfig)
    safety: SafetyConfig = field(default_factory=SafetyConfig)
    mission: MissionConfig = field(default_factory=MissionConfig)
    fpv: FPVConfig = field(default_factory=FPVConfig)
    ai: AIConfig = field(default_factory=AIConfig)
    home: HomeLocation = field(default_factory=HomeLocation)

    def validate(self) -> bool:
        """Validate configuration."""
        errors = []

        # Flight limits
        if self.flight.altitude_min_m >= self.flight.altitude_max_m:
            errors.append("altitude_min must be less than altitude_max")

        # Battery thresholds
        if self.battery.critical_soc_percent >= self.battery.low_soc_percent:
            errors.append("critical_soc must be less than low_soc")

        # Safety
        if self.safety.geofence_radius_m <= 0:
            errors.append("geofence_radius must be positive")

        if errors:
            raise ValueError(f"Configuration errors: {errors}")

        return True


def load_config(path: Optional[Path] = None) -> DroneConfig:
    """
    Load configuration from YAML file.

    Args:
        path: Path to YAML config file

    Returns:
        DroneConfig instance
    """
    config = DroneConfig()

    if path and path.exists():
        with open(path) as f:
            data = yaml.safe_load(f)

        if data:
            # Update MAVLink config
            if "mavlink" in data:
                for key, value in data["mavlink"].items():
                    if hasattr(config.mavlink, key):
                        setattr(config.mavlink, key, value)

            # Update flight config
            if "flight" in data:
                for key, value in data["flight"].items():
                    if hasattr(config.flight, key):
                        setattr(config.flight, key, value)

            # Update battery config
            if "battery" in data:
                for key, value in data["battery"].items():
                    if hasattr(config.battery, key):
                        setattr(config.battery, key, value)

            # Update safety config
            if "safety" in data:
                for key, value in data["safety"].items():
                    if hasattr(config.safety, key):
                        setattr(config.safety, key, value)

            # Update mission config
            if "mission" in data:
                for key, value in data["mission"].items():
                    if hasattr(config.mission, key):
                        setattr(config.mission, key, value)

            # Update FPV config
            if "fpv" in data:
                for key, value in data["fpv"].items():
                    if hasattr(config.fpv, key):
                        setattr(config.fpv, key, value)

            # Update AI config
            if "ai" in data:
                for key, value in data["ai"].items():
                    if hasattr(config.ai, key):
                        setattr(config.ai, key, value)

            # Update home location
            if "home" in data:
                for key, value in data["home"].items():
                    if hasattr(config.home, key):
                        setattr(config.home, key, value)

    config.validate()
    return config


def get_default_config() -> DroneConfig:
    """Get default configuration."""
    return DroneConfig()
