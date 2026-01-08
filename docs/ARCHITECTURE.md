# ArduPilot Drone AI - Architecture Guide

## System Overview

The ArduPilot Drone AI system is designed as a modular, layered architecture that separates concerns between communication, control, safety, and decision-making.

```
┌─────────────────────────────────────────────────────────────────┐
│                        AI AGENT LAYER                           │
│  ┌─────────────────────────────────────────────────────────┐   │
│  │                    DroneAgent                            │   │
│  │  - Coordinates all subsystems                           │   │
│  │  - High-level decision making                           │   │
│  │  - Mission management                                    │   │
│  └─────────────────────────────────────────────────────────┘   │
├─────────────────────────────────────────────────────────────────┤
│                      CONTROL LAYER                              │
│  ┌──────────────┐ ┌──────────────┐ ┌──────────────────────┐   │
│  │   Flight     │ │   Safety     │ │     Mission          │   │
│  │  Controller  │ │   Manager    │ │     Planner          │   │
│  │              │ │              │ │                      │   │
│  │ - Takeoff    │ │ - Geofence   │ │ - Waypoints          │   │
│  │ - Landing    │ │ - Failsafes  │ │ - Patterns           │   │
│  │ - Navigation │ │ - Emergencies│ │ - Surveys            │   │
│  └──────────────┘ └──────────────┘ └──────────────────────┘   │
├─────────────────────────────────────────────────────────────────┤
│                      SERVICE LAYER                              │
│  ┌──────────────┐ ┌──────────────┐ ┌──────────────────────┐   │
│  │   Battery    │ │    State     │ │     Navigator        │   │
│  │   Manager    │ │   Machine    │ │                      │   │
│  │              │ │              │ │                      │   │
│  │ - SOC Track  │ │ - Transitions│ │ - Path Planning      │   │
│  │ - Prediction │ │ - Callbacks  │ │ - Distance Calc      │   │
│  └──────────────┘ └──────────────┘ └──────────────────────┘   │
├─────────────────────────────────────────────────────────────────┤
│                   COMMUNICATION LAYER                           │
│  ┌──────────────┐ ┌──────────────┐ ┌──────────────────────┐   │
│  │  MAVLink     │ │   Message    │ │     Command          │   │
│  │  Connection  │ │   Handler    │ │     Sender           │   │
│  │              │ │              │ │                      │   │
│  │ - UDP/Serial │ │ - Telemetry  │ │ - Mode Changes       │   │
│  │ - Heartbeat  │ │ - Parsing    │ │ - Navigation         │   │
│  └──────────────┘ └──────────────┘ └──────────────────────┘   │
├─────────────────────────────────────────────────────────────────┤
│                      HARDWARE LAYER                             │
│  ┌─────────────────────────────────────────────────────────┐   │
│  │            ArduPilot Flight Controller                   │   │
│  │  - Attitude Control  - Motor Mixing  - Sensor Fusion    │   │
│  └─────────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────────┘
```

## Module Architecture

### 1. MAVLink Module (`src/mavlink/`)

Handles all communication with the ArduPilot flight controller.

```
mavlink/
├── connection.py    # Connection management
├── messages.py      # Message processing
└── commands.py      # Command interface
```

#### Connection (`connection.py`)

```python
class MAVLinkConnection:
    """
    Manages MAVLink connection lifecycle.

    Features:
    - UDP, TCP, and Serial support
    - Automatic heartbeat
    - Message routing
    - Connection monitoring
    """
```

**Key Methods:**
- `connect()` - Establish connection, wait for heartbeat
- `disconnect()` - Clean disconnection
- `send_command_long()` - Send MAV_CMD commands
- `set_mode()` - Change flight mode
- `arm()` / `disarm()` - Arm control
- `goto()` - Position commands
- `set_velocity()` - Velocity commands

#### Messages (`messages.py`)

```python
class MessageHandler:
    """
    Processes incoming MAVLink messages.

    Maintains:
    - Position (GLOBAL_POSITION_INT)
    - Attitude (ATTITUDE)
    - Battery (BATTERY_STATUS, SYS_STATUS)
    - GPS (GPS_RAW_INT)
    - RC (RC_CHANNELS)
    - System status (HEARTBEAT)
    """
```

**Telemetry Structure:**
```python
@dataclass
class Telemetry:
    position: Position      # Lat, lon, altitude
    velocity: Velocity      # Speed, climb rate
    attitude: Attitude      # Roll, pitch, yaw
    battery: Battery        # Voltage, current, SOC
    gps: GPS               # Fix, satellites
    rc: RCInput            # RC channels
    system: SystemStatus   # Mode, armed, health
    home: HomePosition     # Home location
```

#### Commands (`commands.py`)

```python
class CommandSender:
    """
    High-level command interface with acknowledgment.

    Features:
    - Automatic retries
    - Command acknowledgment waiting
    - Timeout handling
    """
```

### 2. Flight Module (`src/flight/`)

High-level flight control abstraction.

```
flight/
├── controller.py    # Flight control interface
└── navigation.py    # Navigation and patterns
```

#### Controller (`controller.py`)

```python
class FlightController:
    """
    Simplified flight control interface.

    States:
    - DISARMED
    - ARMED
    - TAKING_OFF
    - FLYING
    - LANDING
    - RETURNING
    - EMERGENCY
    """
```

**Key Operations:**
```python
# Takeoff sequence
await controller.arm()
await controller.takeoff(30)  # 30m altitude
await controller.wait_takeoff()

# Navigation
await controller.goto(lat, lon, alt)
await controller.hold()  # Loiter

# Return
await controller.rtl()
await controller.land()
```

#### Navigation (`navigation.py`)

```python
class Navigator:
    """
    Mission planning and pattern generation.

    Patterns:
    - Survey (lawn-mower)
    - Orbit (circular)
    - Spiral
    - Custom waypoints
    """
```

**Pattern Generation:**
```python
# Survey pattern
waypoints = navigator.create_survey_pattern(
    center_lat=-33.87,
    center_lon=151.21,
    width_m=100,
    height_m=100,
    altitude_m=50,
    spacing_m=10,
)

# Orbit pattern
waypoints = navigator.create_orbit_pattern(
    center_lat=-33.87,
    center_lon=151.21,
    radius_m=50,
    altitude_m=30,
    num_points=12,
)
```

### 3. Energy Module (`src/energy/`)

Battery monitoring and flight time prediction.

```
energy/
├── battery.py       # Battery state tracking
└── predictor.py     # Flight time prediction
```

#### Battery Manager (`battery.py`)

```python
class BatteryManager:
    """
    Battery state tracking and prediction.

    States:
    - FULL (>90%)
    - GOOD (50-90%)
    - LOW (25-50%)
    - CRITICAL (15-25%)
    - EMERGENCY (<15%)
    """
```

**Features:**
- Voltage-based SOC estimation
- Cell voltage monitoring
- Current averaging
- Time/distance remaining prediction

#### Flight Predictor (`predictor.py`)

```python
class FlightTimePredictor:
    """
    Predicts remaining flight capability.

    Calculates:
    - Time remaining
    - Distance remaining
    - RTL reserve
    - Effective range
    """
```

### 4. Safety Module (`src/safety/`)

Geofencing and failsafe management.

```
safety/
├── geofence.py      # Boundary management
└── failsafe.py      # Failsafe handling
```

#### Geofence (`geofence.py`)

```python
class Geofence:
    """
    Virtual boundary management.

    Types:
    - Circular (radius from center)
    - Polygon (arbitrary shape)
    - Altitude limits

    Actions:
    - NONE (report only)
    - WARN
    - LOITER
    - RTL
    - LAND
    """
```

**Geofence Setup:**
```python
# Home-centered fence
geofence.set_home_fence(
    lat=-33.87,
    lon=151.21,
    radius_m=1000,
    max_altitude_m=120,
)

# No-fly zone
geofence.add_no_fly_zone([
    (-33.88, 151.20),
    (-33.88, 151.22),
    (-33.86, 151.22),
    (-33.86, 151.20),
])
```

#### Failsafe Manager (`failsafe.py`)

```python
class FailsafeManager:
    """
    Monitors conditions and triggers failsafes.

    Conditions:
    - BATTERY_LOW / BATTERY_CRITICAL
    - GPS_LOST
    - LINK_LOST
    - GEOFENCE_BREACH
    - EKF_FAILURE
    """
```

### 5. AI Module (`src/ai/`)

Autonomous decision-making and mission management.

```
ai/
├── agent.py         # Main AI agent
└── state_machine.py # Mission state machine
```

#### State Machine (`state_machine.py`)

```python
class MissionStateMachine:
    """
    Mission state management.

    States:
    - IDLE → PREFLIGHT → ARMING → TAKEOFF
    - CLIMBING → TRANSIT → LOITER/MISSION/SURVEY
    - RTL → DESCENDING → LANDING → LANDED
    - EMERGENCY (from any flying state)
    """
```

**State Diagram:**
```
IDLE ──► PREFLIGHT ──► ARMING ──► TAKEOFF
                                     │
              ┌──────────────────────┘
              ▼
         CLIMBING ──► LOITER ◄──► TRANSIT
              │          │            │
              │          ▼            │
              │      MISSION ◄────────┤
              │          │            │
              │          ▼            │
              │      SURVEY ◄─────────┘
              │          │
              ▼          ▼
           RTL ◄─────────┘
              │
              ▼
        DESCENDING
              │
              ▼
         LANDING
              │
              ▼
          LANDED ──► IDLE

    EMERGENCY (accessible from any flying state)
```

#### Drone Agent (`agent.py`)

```python
class DroneAgent:
    """
    Main autonomous control agent.

    Coordinates:
    - Flight controller
    - Battery manager
    - Geofence
    - Failsafe manager
    - State machine
    - Navigator
    """
```

**Agent Lifecycle:**
```python
agent = DroneAgent(connection_string="udp:127.0.0.1:14550")

# Start (connects and initializes)
await agent.start()

# Operations
await agent.arm_and_takeoff(30)
await agent.goto(lat, lon, alt)
await agent.execute_survey(...)
await agent.rtl()

# Stop
await agent.stop()
```

## Data Flow

### Telemetry Flow
```
Flight Controller → MAVLink → MessageHandler → Telemetry
                                    ↓
                            BatteryManager
                                    ↓
                            FlightPredictor
                                    ↓
                            FailsafeManager
                                    ↓
                              DroneAgent
```

### Command Flow
```
DroneAgent → FlightController → CommandSender → MAVLink → FC
                                      ↑
                              Acknowledgment
```

### Decision Flow
```
Telemetry ──► Battery Check ──► Geofence Check ──► Failsafe Check
                                                         │
                                                         ▼
                                                   State Machine
                                                         │
                                                         ▼
                                                   Execute Action
```

## Threading Model

The system uses Python's `asyncio` for concurrent operations:

```python
# Main update loop
async def _update_loop(self):
    while self._running:
        await self._update()  # Process telemetry, check safety
        await asyncio.sleep(1/update_rate)

# Background tasks
self._receive_task = asyncio.create_task(self._receive_loop())
self._heartbeat_task = asyncio.create_task(self._heartbeat_loop())
```

## Error Handling

### Connection Errors
- Automatic reconnection attempts
- Heartbeat timeout detection
- Link loss failsafe triggering

### Command Errors
- Retry with confirmation counter
- Timeout handling
- Error logging

### Safety Errors
- Immediate failsafe activation
- State machine transition to emergency
- Automatic RTL/LAND

## Extension Points

### Adding New Flight Patterns
```python
# In navigation.py
def create_custom_pattern(self, params) -> List[Waypoint]:
    waypoints = []
    # Generate pattern
    return waypoints
```

### Adding New Failsafes
```python
# In failsafe.py
class FailsafeType(Enum):
    CUSTOM_CONDITION = auto()

# In update()
if custom_condition_triggered:
    self._activate_failsafe(FailsafeType.CUSTOM_CONDITION)
```

### Adding New Commands
```python
# In commands.py
async def custom_command(self, params) -> bool:
    return await self.send_command(
        MAV_CMD_CUSTOM,
        param1=value,
    )
```

## Performance Considerations

- Update rate: 10 Hz default (configurable)
- Message processing: Non-blocking async
- Memory: Bounded history buffers
- Latency: <100ms command response typical
