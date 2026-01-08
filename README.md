# ArduPilot Drone AI

Autonomous AI agent for ArduPilot multicopter/drone operations. Enables long-range FPV missions with AI as the controller.

## Overview

ArduPilot Drone AI is an intelligent companion computer software that integrates with ArduPilot-based flight controllers to provide autonomous decision-making capabilities for multicopter drones. It handles:

- **Autonomous Flight Control** - Takeoff, navigation, landing with intelligent decision-making
- **Battery Management** - Real-time monitoring, flight time prediction, automatic RTL
- **Safety Systems** - Geofencing, failsafes, emergency procedures
- **Mission Execution** - Survey patterns, waypoint missions, orbit patterns
- **FPV Integration** - Long-range mission support with AI oversight

## Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    DRONE AI SYSTEM                          │
├─────────────────────────────────────────────────────────────┤
│  ┌─────────────────┐    MAVLink     ┌─────────────────────┐ │
│  │   ArduPilot     │◄──────────────►│   Companion Computer│ │
│  │   (Copter)      │                │   (AI Agent)        │ │
│  │                 │                │                     │ │
│  │  - Stabilization│                │  - Decision Making  │ │
│  │  - Attitude Ctrl│                │  - Battery Mgmt     │ │
│  │  - Basic Nav    │                │  - Safety/Geofence  │ │
│  │  - Failsafes    │                │  - Mission Planning │ │
│  └─────────────────┘                └─────────────────────┘ │
└─────────────────────────────────────────────────────────────┘
```

## Features

### Flight Control
- Arm/disarm with preflight checks
- Automated takeoff and landing
- GUIDED mode navigation
- Velocity control for FPV
- Multiple flight modes (LOITER, RTL, AUTO)

### Battery Management
- Real-time voltage and current monitoring
- State of charge estimation
- Flight time prediction
- Automatic RTL on low battery
- Range calculation

### Safety Systems
- Circular and polygon geofences
- Altitude limits
- Link loss failsafe
- GPS loss failsafe
- EKF health monitoring

### Mission Planning
- Survey/lawn-mower patterns
- Circular orbit patterns
- Spiral patterns
- Waypoint missions
- Auto-continue navigation

## Quick Start

### Prerequisites

- Python 3.8+
- ArduPilot SITL or compatible flight controller
- pymavlink

### Installation

```bash
# Clone repository
cd /path/to/ardupilot-AI

# Create virtual environment
python3 -m venv venv
source venv/bin/activate

# Install dependencies
pip install -r requirements.txt
```

### Running with SITL

**Terminal 1 - Start SITL:**
```bash
cd ~/ardupilot
sim_vehicle.py -v ArduCopter --console --map
```

**Terminal 2 - Start AI Agent:**
```bash
cd /path/to/ardupilot-AI
source venv/bin/activate
python src/main.py --sitl
```

### Basic Usage

```python
import asyncio
from ai.agent import DroneAgent

async def main():
    # Create agent
    agent = DroneAgent(connection_string="udp:127.0.0.1:14550")

    # Start agent
    await agent.start()

    # Arm and takeoff
    await agent.arm_and_takeoff(altitude_m=30)

    # Go to position
    await agent.goto(lat=-33.87, lon=151.21, alt=50)

    # Return to launch
    await agent.rtl()

    # Stop agent
    await agent.stop()

asyncio.run(main())
```

## Project Structure

```
ardupilot-AI/
├── src/
│   ├── mavlink/           # MAVLink communication
│   │   ├── connection.py  # Connection management
│   │   ├── messages.py    # Telemetry processing
│   │   └── commands.py    # Command interface
│   ├── flight/            # Flight control
│   │   ├── controller.py  # High-level control
│   │   └── navigation.py  # Navigation & patterns
│   ├── energy/            # Power management
│   │   ├── battery.py     # Battery monitoring
│   │   └── predictor.py   # Flight time prediction
│   ├── safety/            # Safety systems
│   │   ├── geofence.py    # Geofence management
│   │   └── failsafe.py    # Failsafe handling
│   ├── ai/                # AI agent
│   │   ├── agent.py       # Main agent
│   │   └── state_machine.py # Mission states
│   ├── config.py          # Configuration
│   └── main.py            # Entry point
├── config/
│   └── default.yaml       # Default configuration
├── tests/
│   └── test_modules.py    # Module tests
├── docs/                  # Documentation
├── requirements.txt
└── README.md
```

## Configuration

Configuration is managed via YAML files. See `config/default.yaml` for all options.

Key settings:

```yaml
mavlink:
  connection_string: "udp:127.0.0.1:14550"

flight:
  altitude_max_m: 120.0
  speed_cruise_ms: 10.0

battery:
  capacity_mah: 10000.0
  critical_soc_percent: 15.0

safety:
  geofence_radius_m: 1000.0
  link_loss_action: "rtl"
```

## Testing

```bash
# Run all tests
python tests/test_modules.py

# Run with pytest
pytest tests/ -v
```

## Documentation

- [Architecture Guide](docs/ARCHITECTURE.md) - System design and components
- [API Reference](docs/API.md) - Complete API documentation
- [Configuration Guide](docs/CONFIGURATION.md) - Configuration options
- [Testing Guide](docs/TESTING.md) - Testing with SITL

## Supported Hardware

### Flight Controllers
- Pixhawk series (1/2/4/5/6)
- Cube (Orange/Black/Blue)
- Kakute F7/H7
- Any ArduCopter-compatible FC

### Companion Computers
- NVIDIA Jetson (Nano/Xavier/Orin)
- Raspberry Pi 4/5
- Intel NUC
- Any Linux system with Python 3.8+

## Flight Modes

| Mode | Value | Description |
|------|-------|-------------|
| STABILIZE | 0 | Manual with self-leveling |
| ALT_HOLD | 2 | Altitude hold |
| AUTO | 3 | Autonomous mission |
| GUIDED | 4 | External control (AI) |
| LOITER | 5 | Position hold |
| RTL | 6 | Return to launch |
| LAND | 9 | Autonomous landing |
| POSHOLD | 16 | GPS position hold |
| SMART_RTL | 21 | Retrace path home |

## Safety Notes

- Always test in SITL before real flights
- Set appropriate geofence limits
- Configure battery failsafes
- Maintain visual line of sight for FPV
- Follow local regulations

## License

MIT License - See LICENSE file for details.

## Contributing

Contributions welcome! Please read the contributing guidelines first.
