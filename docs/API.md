# ArduPilot Drone AI - API Reference

## Table of Contents

- [DroneAgent](#droneagent)
- [FlightController](#flightcontroller)
- [Navigator](#navigator)
- [BatteryManager](#batterymanager)
- [Geofence](#geofence)
- [FailsafeManager](#failsafemanager)
- [MissionStateMachine](#missionstatemachine)
- [MAVLinkConnection](#mavlinkconnection)
- [Data Structures](#data-structures)
- [Enumerations](#enumerations)

---

## DroneAgent

Main autonomous control agent. Coordinates all subsystems for intelligent flight operations.

```python
from ai.agent import DroneAgent
```

### Constructor

```python
DroneAgent(
    config: Optional[DroneConfig] = None,
    connection_string: str = "udp:127.0.0.1:14550"
)
```

**Parameters:**
| Parameter | Type | Description |
|-----------|------|-------------|
| `config` | `DroneConfig` | Configuration object |
| `connection_string` | `str` | MAVLink connection string |

### Properties

| Property | Type | Description |
|----------|------|-------------|
| `state` | `AgentState` | Current agent state |
| `telemetry` | `Telemetry` | Current telemetry data |
| `is_connected` | `bool` | Connection status |
| `is_flying` | `bool` | Flying status |

### Methods

#### start()

```python
async def start() -> bool
```

Start the agent, connect to vehicle, initialize subsystems.

**Returns:** `True` if started successfully

#### stop()

```python
async def stop()
```

Stop the agent and disconnect from vehicle.

#### arm_and_takeoff()

```python
async def arm_and_takeoff(altitude_m: float) -> bool
```

Arm the vehicle and takeoff to specified altitude.

**Parameters:**
| Parameter | Type | Description |
|-----------|------|-------------|
| `altitude_m` | `float` | Target altitude in meters AGL |

**Returns:** `True` if successful

#### goto()

```python
async def goto(
    lat: float,
    lon: float,
    alt: Optional[float] = None
) -> bool
```

Navigate to position.

**Parameters:**
| Parameter | Type | Description |
|-----------|------|-------------|
| `lat` | `float` | Target latitude (degrees) |
| `lon` | `float` | Target longitude (degrees) |
| `alt` | `float` | Target altitude (None = current) |

#### rtl()

```python
async def rtl() -> bool
```

Return to launch position.

#### land()

```python
async def land() -> bool
```

Land at current position.

#### hold()

```python
async def hold() -> bool
```

Hold current position (enter LOITER mode).

#### set_home_fence()

```python
def set_home_fence(
    radius_m: float,
    max_altitude_m: float = 120
)
```

Set geofence around home position.

#### execute_survey()

```python
async def execute_survey(
    center_lat: float,
    center_lon: float,
    width_m: float,
    height_m: float,
    altitude_m: float,
    spacing_m: float = 10
) -> bool
```

Execute survey pattern mission.

#### get_status()

```python
def get_status() -> dict
```

Get comprehensive status dictionary.

**Returns:**
```python
{
    "agent": {"state": "running", "connected": True},
    "mission": {"state": "LOITER", "is_flying": True},
    "battery": {"voltage": 22.2, "soc_percent": 75},
    "geofence": {"enabled": True},
    "failsafe": {"active_failsafes": []},
    "telemetry": {...}
}
```

---

## FlightController

High-level flight control interface.

```python
from flight.controller import FlightController
```

### Constructor

```python
FlightController(
    connection: MAVLinkConnection,
    message_handler: MessageHandler
)
```

### Properties

| Property | Type | Description |
|----------|------|-------------|
| `telemetry` | `Telemetry` | Current telemetry |
| `state` | `FlightState` | Flight state |
| `mode` | `FlightMode` | Current mode |
| `is_armed` | `bool` | Armed status |
| `is_flying` | `bool` | Flying status |
| `altitude` | `float` | Current altitude (m) |
| `position` | `tuple` | (lat, lon) |

### Methods

#### arm()

```python
async def arm(preflight_check: bool = True) -> bool
```

Arm the vehicle with optional preflight checks.

#### disarm()

```python
async def disarm(force: bool = False) -> bool
```

Disarm the vehicle.

#### takeoff()

```python
async def takeoff(altitude_m: float) -> bool
```

Command takeoff to altitude.

#### wait_takeoff()

```python
async def wait_takeoff(timeout: float = 60.0) -> bool
```

Wait for takeoff to complete.

#### land()

```python
async def land() -> bool
```

Command landing.

#### wait_landing()

```python
async def wait_landing(timeout: float = 120.0) -> bool
```

Wait for landing to complete.

#### rtl()

```python
async def rtl() -> bool
```

Return to launch.

#### goto()

```python
async def goto(
    lat: float,
    lon: float,
    alt: Optional[float] = None,
    speed: float = 0
) -> bool
```

Navigate to position.

#### fly_velocity()

```python
async def fly_velocity(
    forward_ms: float = 0,
    right_ms: float = 0,
    down_ms: float = 0,
    yaw_rate_dps: float = 0
) -> bool
```

Fly at specified velocity (body frame).

#### hold()

```python
async def hold() -> bool
```

Enter position hold (LOITER).

#### set_altitude()

```python
async def set_altitude(altitude_m: float) -> bool
```

Change altitude.

#### set_heading()

```python
async def set_heading(
    heading_deg: float,
    rate_dps: float = 0
) -> bool
```

Set yaw heading.

#### set_mode()

```python
async def set_mode(mode: FlightMode) -> bool
```

Set flight mode.

#### emergency_stop()

```python
async def emergency_stop() -> bool
```

Emergency stop (land immediately).

---

## Navigator

Mission planning and pattern generation.

```python
from flight.navigation import Navigator, Waypoint
```

### Constructor

```python
Navigator()
```

### Properties

| Property | Type | Description |
|----------|------|-------------|
| `mission` | `Mission` | Current mission |
| `is_navigating` | `bool` | Navigation active |

### Methods

#### set_home()

```python
def set_home(lat: float, lon: float, alt: float = 0)
```

Set home/launch position.

#### add_waypoint()

```python
def add_waypoint(
    lat: float,
    lon: float,
    alt: float,
    hold_time_s: float = 0,
    yaw_deg: Optional[float] = None
)
```

Add waypoint to mission.

#### create_survey_pattern()

```python
def create_survey_pattern(
    center_lat: float,
    center_lon: float,
    width_m: float,
    height_m: float,
    altitude_m: float,
    spacing_m: float,
    heading_deg: float = 0
) -> List[Waypoint]
```

Create survey/lawn-mower pattern.

**Returns:** List of waypoints

#### create_orbit_pattern()

```python
def create_orbit_pattern(
    center_lat: float,
    center_lon: float,
    radius_m: float,
    altitude_m: float,
    num_points: int = 12,
    start_heading_deg: float = 0
) -> List[Waypoint]
```

Create circular orbit pattern.

#### create_spiral_pattern()

```python
def create_spiral_pattern(
    center_lat: float,
    center_lon: float,
    start_radius_m: float,
    end_radius_m: float,
    altitude_m: float,
    turns: float = 3,
    points_per_turn: int = 8
) -> List[Waypoint]
```

Create spiral pattern.

#### distance_to_waypoint()

```python
def distance_to_waypoint(
    current_lat: float,
    current_lon: float,
    waypoint: Waypoint
) -> float
```

Calculate distance to waypoint in meters.

#### bearing_to_waypoint()

```python
def bearing_to_waypoint(
    current_lat: float,
    current_lon: float,
    waypoint: Waypoint
) -> float
```

Calculate bearing to waypoint in degrees.

#### total_mission_distance()

```python
def total_mission_distance() -> float
```

Calculate total mission distance in meters.

#### clear_mission()

```python
def clear_mission()
```

Clear all waypoints.

---

## BatteryManager

Battery state tracking and prediction.

```python
from energy.battery import BatteryManager, BatteryState
```

### Constructor

```python
BatteryManager(config: Optional[BatteryConfig] = None)
```

### Properties

| Property | Type | Description |
|----------|------|-------------|
| `status` | `BatteryStatus` | Full status |
| `soc` | `float` | State of charge (%) |
| `state` | `BatteryState` | Battery state |
| `voltage` | `float` | Current voltage |
| `current` | `float` | Current draw (A) |
| `is_critical` | `bool` | Critical state |
| `is_low` | `bool` | Low state |

### Methods

#### update()

```python
def update(
    voltage: float,
    current: float,
    consumed_mah: float = 0,
    remaining_percent: int = -1,
    cell_count: int = 0,
    temperature: float = 0
)
```

Update battery state from telemetry.

#### get_voltage_trend()

```python
def get_voltage_trend() -> float
```

Get voltage trend (V/min).

#### get_current_average()

```python
def get_current_average(window_s: float = 30) -> float
```

Get average current over window.

#### predict_soc_at_time()

```python
def predict_soc_at_time(future_s: float) -> float
```

Predict SOC at future time.

#### can_complete_mission()

```python
def can_complete_mission(
    mission_time_s: float,
    return_time_s: float,
    reserve_percent: float = 20
) -> Tuple[bool, float]
```

Check if battery can complete mission.

**Returns:** `(can_complete, margin_percent)`

#### get_status_dict()

```python
def get_status_dict() -> dict
```

Get status as dictionary.

---

## Geofence

Virtual boundary management.

```python
from safety.geofence import Geofence, GeofenceAction
```

### Constructor

```python
Geofence()
```

### Properties

| Property | Type | Description |
|----------|------|-------------|
| `enabled` | `bool` | Geofence enabled |

### Methods

#### enable() / disable()

```python
def enable()
def disable()
```

Enable/disable geofencing.

#### set_home_fence()

```python
def set_home_fence(
    lat: float,
    lon: float,
    radius_m: float,
    max_altitude_m: float = 120,
    action: GeofenceAction = GeofenceAction.RTL
)
```

Set circular fence around home.

#### add_circular_fence()

```python
def add_circular_fence(fence: CircularGeofence)
```

Add circular geofence.

#### add_polygon_fence()

```python
def add_polygon_fence(fence: PolygonGeofence)
```

Add polygon geofence.

#### add_no_fly_zone()

```python
def add_no_fly_zone(
    vertices: List[Tuple[float, float]],
    max_altitude_m: float = 120,
    action: GeofenceAction = GeofenceAction.RTL
)
```

Add no-fly zone (exclusion polygon).

#### check()

```python
def check(
    lat: float,
    lon: float,
    altitude_m: float
) -> GeofenceStatus
```

Check position against all geofences.

**Returns:**
```python
GeofenceStatus(
    is_inside=True,
    distance_to_boundary_m=450.0,
    breach_type=None,
    recommended_action=GeofenceAction.NONE
)
```

#### distance_to_boundary()

```python
def distance_to_boundary(lat: float, lon: float) -> float
```

Distance to nearest fence boundary.

#### clear_fences()

```python
def clear_fences()
```

Clear all geofences.

---

## FailsafeManager

Failsafe condition monitoring and handling.

```python
from safety.failsafe import FailsafeManager, FailsafeType, FailsafeAction
```

### Constructor

```python
FailsafeManager(config: Optional[FailsafeConfig] = None)
```

### Methods

#### register_action_callback()

```python
def register_action_callback(
    action: FailsafeAction,
    callback: Callable
)
```

Register callback for failsafe action.

#### update()

```python
def update(
    battery_percent: float,
    gps_ok: bool,
    link_ok: bool,
    ekf_ok: bool,
    in_geofence: bool,
    altitude_ok: bool = True
) -> FailsafeState
```

Update failsafe state.

**Returns:**
```python
FailsafeState(
    active_failsafes=[FailsafeType.BATTERY_LOW],
    highest_priority_action=FailsafeAction.RTL,
    is_failsafe_active=True,
    timestamp=1234567890.0
)
```

#### is_failsafe_active()

```python
def is_failsafe_active(fs_type: FailsafeType) -> bool
```

Check if specific failsafe is active.

#### get_active_failsafes()

```python
def get_active_failsafes() -> List[FailsafeType]
```

Get list of active failsafes.

#### clear_all()

```python
def clear_all()
```

Clear all failsafes.

---

## MissionStateMachine

Mission state management.

```python
from ai.state_machine import MissionStateMachine, MissionState
```

### Constructor

```python
MissionStateMachine()
```

### Properties

| Property | Type | Description |
|----------|------|-------------|
| `state` | `MissionState` | Current state |
| `time_in_state` | `float` | Seconds in current state |
| `history` | `List[StateTransition]` | State history |

### Methods

#### can_transition()

```python
def can_transition(to_state: MissionState) -> bool
```

Check if transition is valid.

#### transition()

```python
def transition(
    to_state: MissionState,
    reason: str = ""
) -> bool
```

Attempt state transition.

#### on_enter() / on_exit()

```python
def on_enter(state: MissionState, callback: Callable)
def on_exit(state: MissionState, callback: Callable)
```

Register state entry/exit callbacks.

#### is_flying()

```python
def is_flying() -> bool
```

Check if in flying state.

#### is_emergency()

```python
def is_emergency() -> bool
```

Check if in emergency state.

#### reset()

```python
def reset()
```

Reset to IDLE state.

---

## MAVLinkConnection

Low-level MAVLink connection.

```python
from mavlink.connection import MAVLinkConnection
```

### Constructor

```python
MAVLinkConnection(
    connection_string: str = "udp:127.0.0.1:14550",
    source_system: int = 255,
    source_component: int = 0
)
```

### Properties

| Property | Type | Description |
|----------|------|-------------|
| `connected` | `bool` | Connection status |
| `target_system` | `int` | Target system ID |
| `heartbeat` | `dict` | Last heartbeat info |

### Methods

#### connect()

```python
async def connect(timeout: float = 30.0) -> bool
```

Establish connection.

#### disconnect()

```python
async def disconnect()
```

Disconnect from vehicle.

#### send_command_long()

```python
async def send_command_long(
    command: int,
    param1-7: float,
    confirmation: int = 0
) -> bool
```

Send MAV_CMD command.

#### set_mode()

```python
async def set_mode(mode: int) -> bool
```

Set flight mode.

#### arm() / disarm()

```python
async def arm(force: bool = False) -> bool
async def disarm(force: bool = False) -> bool
```

Arm/disarm vehicle.

#### takeoff()

```python
async def takeoff(altitude_m: float) -> bool
```

Command takeoff.

#### land()

```python
async def land() -> bool
```

Command landing.

#### goto()

```python
async def goto(
    lat: float,
    lon: float,
    alt: float,
    groundspeed: float = 0
) -> bool
```

Go to position.

#### set_velocity()

```python
async def set_velocity(
    vx: float,
    vy: float,
    vz: float,
    yaw_rate: float = 0
) -> bool
```

Set velocity (body frame).

---

## Data Structures

### Waypoint

```python
@dataclass
class Waypoint:
    latitude: float
    longitude: float
    altitude: float
    wp_type: WaypointType = WaypointType.WAYPOINT
    hold_time_s: float = 0
    accept_radius_m: float = 2.0
    yaw_deg: Optional[float] = None
    speed_ms: Optional[float] = None
```

### Telemetry

```python
@dataclass
class Telemetry:
    position: Position
    velocity: Velocity
    attitude: Attitude
    battery: Battery
    gps: GPS
    rc: RCInput
    system: SystemStatus
    home: HomePosition
```

### BatteryStatus

```python
@dataclass
class BatteryStatus:
    voltage: float
    current: float
    consumed_mah: float
    remaining_mah: float
    soc_percent: float
    cell_voltage: float
    cell_count: int
    state: BatteryState
    time_remaining_s: float
    distance_remaining_m: float
```

---

## Enumerations

### FlightMode

```python
class FlightMode(Enum):
    STABILIZE = 0
    ACRO = 1
    ALT_HOLD = 2
    AUTO = 3
    GUIDED = 4
    LOITER = 5
    RTL = 6
    CIRCLE = 7
    LAND = 9
    POSHOLD = 16
    SMART_RTL = 21
```

### MissionState

```python
class MissionState(Enum):
    IDLE = auto()
    PREFLIGHT = auto()
    ARMING = auto()
    TAKEOFF = auto()
    CLIMBING = auto()
    TRANSIT = auto()
    LOITER = auto()
    MISSION = auto()
    SURVEY = auto()
    RTL = auto()
    DESCENDING = auto()
    LANDING = auto()
    LANDED = auto()
    EMERGENCY = auto()
    LOST_LINK = auto()
```

### BatteryState

```python
class BatteryState(Enum):
    UNKNOWN = "unknown"
    FULL = "full"      # >90%
    GOOD = "good"      # 50-90%
    LOW = "low"        # 25-50%
    CRITICAL = "critical"  # 15-25%
    EMERGENCY = "emergency"  # <15%
```

### FailsafeType

```python
class FailsafeType(Enum):
    BATTERY_LOW = auto()
    BATTERY_CRITICAL = auto()
    GPS_LOST = auto()
    LINK_LOST = auto()
    GEOFENCE_BREACH = auto()
    EKF_FAILURE = auto()
```

### GeofenceAction

```python
class GeofenceAction(Enum):
    NONE = 0
    WARN = 1
    LOITER = 2
    RTL = 3
    LAND = 4
```
