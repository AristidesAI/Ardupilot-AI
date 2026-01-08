# ArduPilot Drone AI - Configuration Guide

## Overview

Configuration is managed through YAML files and Python dataclasses. All settings can be loaded from files or set programmatically.

## Configuration Files

### File Locations

```
config/
├── default.yaml           # Default configuration
└── mission_profiles/      # Pre-defined mission configurations
    ├── survey.yaml
    └── fpv_cruise.yaml
```

### Loading Configuration

```python
from config import DroneConfig, load_config
from pathlib import Path

# Load from file
config = load_config(Path("config/default.yaml"))

# Use defaults
config = DroneConfig()

# Override specific values
config.mavlink.connection_string = "tcp:192.168.1.100:5760"
config.flight.altitude_max_m = 100.0
```

## Configuration Sections

### MAVLink Configuration

Connection settings for ArduPilot communication.

```yaml
mavlink:
  # Connection string (UDP, TCP, or serial)
  connection_string: "udp:127.0.0.1:14550"

  # Serial baud rate (if using serial)
  baud_rate: 57600

  # Source system/component ID for outgoing messages
  source_system: 255
  source_component: 0

  # Heartbeat interval in seconds
  heartbeat_interval: 1.0

  # Connection timeout in seconds
  timeout: 30.0

  # Command retry count
  retry_count: 3
```

#### Connection String Formats

| Format | Example | Use Case |
|--------|---------|----------|
| UDP | `udp:127.0.0.1:14550` | SITL, local GCS |
| TCP Client | `tcp:192.168.1.100:5760` | Remote FC |
| TCP Server | `tcpin:0.0.0.0:5760` | FC connects to us |
| Serial | `/dev/ttyUSB0` | Direct serial |
| Serial (Jetson) | `/dev/ttyTHS1` | Jetson UART |
| Serial (RPi) | `/dev/ttyAMA0` | Raspberry Pi UART |

### Flight Configuration

Flight envelope and performance limits.

```yaml
flight:
  # Altitude limits (meters AGL)
  altitude_min_m: 2.0      # Minimum safe altitude
  altitude_max_m: 120.0    # Maximum altitude (regulatory)
  altitude_default_m: 30.0 # Default operating altitude

  # Speed limits (m/s)
  speed_max_horizontal_ms: 15.0  # Maximum horizontal speed
  speed_max_vertical_ms: 3.0     # Maximum climb/descent rate
  speed_cruise_ms: 10.0          # Optimal cruise speed
  speed_rtl_ms: 8.0              # RTL speed

  # Acceleration limits (m/s²)
  accel_max_horizontal: 3.0
  accel_max_vertical: 2.5

  # Navigation parameters
  waypoint_radius_m: 2.0   # Waypoint acceptance radius
  loiter_radius_m: 5.0     # Loiter orbit radius
  heading_hold_deg: 5.0    # Heading tolerance

  # Attitude limits
  max_tilt_deg: 35.0       # Maximum tilt angle
  max_yaw_rate_dps: 90.0   # Maximum yaw rate
```

### Battery Configuration

Battery and power management settings.

```yaml
battery:
  # Cell configuration
  cell_count: 6              # Number of cells (e.g., 6S LiPo)
  cell_voltage_min: 3.3      # Minimum safe cell voltage
  cell_voltage_max: 4.2      # Fully charged cell voltage
  cell_voltage_nominal: 3.7  # Nominal cell voltage

  # Capacity
  capacity_mah: 10000.0      # Battery capacity in mAh

  # SOC thresholds (%)
  critical_soc_percent: 15.0 # Trigger emergency land
  low_soc_percent: 25.0      # Trigger RTL warning
  rtl_soc_percent: 30.0      # Recommended RTL threshold
```

#### Common Battery Configurations

| Battery | Cells | Nominal V | Config |
|---------|-------|-----------|--------|
| 3S LiPo | 3 | 11.1V | `cell_count: 3` |
| 4S LiPo | 4 | 14.8V | `cell_count: 4` |
| 6S LiPo | 6 | 22.2V | `cell_count: 6` |
| 8S LiPo | 8 | 29.6V | `cell_count: 8` |
| 12S LiPo | 12 | 44.4V | `cell_count: 12` |

### Safety Configuration

Failsafe and emergency settings.

```yaml
safety:
  # Geofencing
  geofence_enabled: true
  geofence_radius_m: 1000.0      # Maximum distance from home
  geofence_altitude_max_m: 120.0 # Maximum altitude
  geofence_action: "rtl"         # rtl, land, loiter

  # Link loss handling
  link_loss_timeout_s: 5.0       # Seconds before failsafe
  link_loss_action: "rtl"        # rtl, land, continue

  # Battery failsafes
  low_battery_action: "rtl"      # Action on low battery
  critical_battery_action: "land" # Action on critical

  # GPS failsafe
  gps_loss_action: "land"
  min_satellites: 6              # Minimum GPS satellites

  # Motor/ESC failsafe
  motor_fail_action: "land"

  # Maximum flight time (seconds)
  max_flight_time_s: 1800.0      # 30 minutes
```

#### Failsafe Actions

| Action | Description |
|--------|-------------|
| `rtl` | Return to launch position |
| `land` | Land immediately at current position |
| `loiter` | Hold position until resolved |
| `continue` | Continue mission autonomously |

### Mission Configuration

Mission planning parameters.

```yaml
mission:
  # Default mission parameters
  default_altitude_m: 30.0
  default_speed_ms: 10.0

  # Survey parameters
  survey_overlap_percent: 70.0   # Photo overlap
  survey_sidelap_percent: 65.0   # Photo sidelap

  # Waypoint behavior
  auto_continue: true            # Auto-advance waypoints
  loiter_time_s: 0.0             # Default hold time

  # RTL behavior
  rtl_altitude_m: 40.0           # RTL altitude
  rtl_speed_ms: 8.0              # RTL speed
```

### FPV Configuration

Camera and video settings.

```yaml
fpv:
  # Video settings
  video_enabled: true
  video_resolution: "1080p"
  video_fps: 60
  video_bitrate_mbps: 25.0

  # Camera
  camera_tilt_min_deg: -90.0     # Maximum down tilt
  camera_tilt_max_deg: 30.0      # Maximum up tilt

  # OSD
  osd_enabled: true

  # Recording
  recording_enabled: true
  recording_path: "/recordings"
```

### AI Agent Configuration

Autonomous behavior settings.

```yaml
ai:
  # Update rate
  update_rate_hz: 10.0           # Main loop rate

  # Planning
  planning_horizon_s: 30.0       # Look-ahead time

  # Obstacle avoidance
  obstacle_avoidance_enabled: true
  min_obstacle_distance_m: 5.0

  # Autonomous behavior
  auto_rtl_on_low_battery: true
  auto_avoid_no_fly_zones: true

  # Target tracking
  target_tracking_enabled: false
  tracking_prediction_s: 2.0
```

### Home Location

Default home/launch position.

```yaml
home:
  latitude: 0.0
  longitude: 0.0
  altitude_m: 0.0
  set_from_gps: true             # Use first GPS fix as home
```

## Environment Variables

Override configuration via environment:

```bash
# Connection string
export DRONE_AI_CONNECTION="tcp:192.168.1.100:5760"

# Log level
export DRONE_AI_LOG_LEVEL="DEBUG"

# Config file
export DRONE_AI_CONFIG="/etc/drone-ai/production.yaml"
```

## Platform-Specific Configuration

### NVIDIA Jetson

```yaml
mavlink:
  connection_string: "/dev/ttyTHS1"  # Jetson UART
  baud_rate: 921600

# Enable GPU acceleration
compute:
  use_gpu: true
  cuda_device: 0
```

### Raspberry Pi

```yaml
mavlink:
  connection_string: "/dev/ttyAMA0"  # RPi UART
  baud_rate: 57600

# Disable GPU
compute:
  use_gpu: false
```

### Ground Station PC

```yaml
mavlink:
  connection_string: "udp:0.0.0.0:14550"  # Listen for telemetry

# Higher update rate for ground station
ai:
  update_rate_hz: 20.0
```

## Mission Profiles

### Survey Profile

```yaml
# config/mission_profiles/survey.yaml
extends: default

flight:
  altitude_default_m: 50.0
  speed_cruise_ms: 8.0           # Slower for photos

mission:
  survey_overlap_percent: 80.0
  survey_sidelap_percent: 70.0
```

### FPV Cruise Profile

```yaml
# config/mission_profiles/fpv_cruise.yaml
extends: default

flight:
  speed_max_horizontal_ms: 20.0
  speed_cruise_ms: 15.0
  max_tilt_deg: 45.0

fpv:
  video_fps: 120                 # High FPS for FPV
  video_bitrate_mbps: 35.0

ai:
  update_rate_hz: 20.0           # Faster response
```

### Long Range Profile

```yaml
# config/mission_profiles/long_range.yaml
extends: default

flight:
  speed_cruise_ms: 8.0           # Efficient cruise

battery:
  rtl_soc_percent: 40.0          # Higher reserve

safety:
  geofence_radius_m: 5000.0      # 5km range
  link_loss_action: "continue"   # Continue if link lost
```

## Configuration Validation

The system validates configuration on load:

```python
from config import DroneConfig

config = DroneConfig()

# Validation happens automatically
# Invalid values raise ConfigurationError

# Manual validation
config.validate()  # Raises if invalid
```

### Validation Rules

- `altitude_min_m` < `altitude_max_m`
- `speed_cruise_ms` <= `speed_max_horizontal_ms`
- `critical_soc_percent` < `low_soc_percent`
- `cell_voltage_min` < `cell_voltage_max`
- `geofence_radius_m` > 0
- Connection string must be valid format

## ArduPilot Parameters

### Essential Parameters

These ArduPilot parameters should be configured for optimal AI agent operation:

```
# Flight modes (for GUIDED control)
FLTMODE1 = 4  # GUIDED
FLTMODE6 = 6  # RTL

# Failsafes
FS_THR_ENABLE = 1
FS_THR_VALUE = 975
FS_GCS_ENABLE = 2   # RTL on GCS failsafe

# Battery
BATT_MONITOR = 4    # Voltage + Current
BATT_CAPACITY = 10000
BATT_LOW_VOLT = 21.6
BATT_CRT_VOLT = 19.8
BATT_FS_LOW_ACT = 2  # RTL
BATT_FS_CRT_ACT = 1  # Land

# Geofence
FENCE_ENABLE = 1
FENCE_TYPE = 7      # Altitude + Circle
FENCE_ALT_MAX = 120
FENCE_RADIUS = 1000
FENCE_ACTION = 1    # RTL

# Telemetry rates
SR0_POSITION = 10
SR0_EXTRA1 = 10
SR0_EXTRA2 = 4
SR0_EXT_STAT = 2
```

### Loading Parameters

```bash
# In MAVProxy
param load /path/to/drone_ai.param

# Or via Python
await connection.set_param("FENCE_ENABLE", 1)
```
