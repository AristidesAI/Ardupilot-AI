# ArduPilot Drone AI - Testing Guide

## Overview

This guide covers testing the Drone AI system using ArduPilot's Software-In-The-Loop (SITL) simulator and unit tests.

## Prerequisites

### ArduPilot SITL Installation

#### Ubuntu/Debian

```bash
# Clone ArduPilot
git clone https://github.com/ArduPilot/ardupilot.git
cd ardupilot
git submodule update --init --recursive

# Install prerequisites
Tools/environment_install/install-prereqs-ubuntu.sh -y

# Reload profile
. ~/.profile

# Build ArduCopter
./waf configure --board sitl
./waf copter
```

#### macOS

```bash
# Install dependencies
brew install python3 ccache gawk

# Clone and setup ArduPilot
git clone https://github.com/ArduPilot/ardupilot.git
cd ardupilot
git submodule update --init --recursive

# Install Python dependencies
pip3 install -r Tools/environment_install/install-prereqs-mac.txt

# Build
./waf configure --board sitl
./waf copter
```

### Drone AI Installation

```bash
cd /path/to/ardupilot-AI

# Create virtual environment
python3 -m venv venv
source venv/bin/activate

# Install dependencies
pip install -r requirements.txt
```

## Running SITL

### Basic SITL Launch

```bash
# From ArduPilot directory
cd ardupilot

# Launch ArduCopter SITL
sim_vehicle.py -v ArduCopter --console --map
```

This opens:
- MAVProxy console (command line)
- Map window showing vehicle position

### SITL with Custom Location

```bash
# Sydney, Australia
sim_vehicle.py -v ArduCopter \
    --console --map \
    --custom-location=-33.8688,151.2093,0,0
```

### SITL for Drone AI Testing

```bash
# Launch with multiple output ports
sim_vehicle.py -v ArduCopter \
    --console --map \
    --out=udp:127.0.0.1:14550 \
    --out=udp:127.0.0.1:14551 \
    -w  # Wipe EEPROM for clean start
```

## Connecting Drone AI

### Terminal 1: SITL

```bash
cd ardupilot
sim_vehicle.py -v ArduCopter --console --map
```

### Terminal 2: Drone AI

```bash
cd ardupilot-AI
source venv/bin/activate

# Connect to SITL
python src/main.py --sitl

# Or with custom connection
python src/main.py --connection "udp:127.0.0.1:14550"
```

## SITL Commands (MAVProxy)

### Basic Commands

```bash
# In MAVProxy console:

# Arm the vehicle
arm throttle

# Disarm
disarm

# Set mode
mode GUIDED
mode LOITER
mode RTL
mode LAND

# Takeoff (when armed in GUIDED)
takeoff 30

# Check status
status
```

### Guided Mode Navigation

```bash
# Go to position (lat, lon, alt)
guided -33.870 151.210 50

# Set airspeed
speed 10

# Go to altitude
setwp 0 0 0 50  # Relative altitude
```

### Simulating Conditions

```bash
# Set wind speed and direction
param set SIM_WIND_SPD 10    # 10 m/s wind
param set SIM_WIND_DIR 270   # From west

# Simulate GPS glitch
param set SIM_GPS_GLITCH_X 0.001

# Simulate battery drain
param set SIM_BATT_VOLTAGE 21.5
```

## Test Scenarios

### Scenario 1: Basic Connection Test

**Objective**: Verify MAVLink connection and telemetry reception.

```python
# test_connection.py
import asyncio
from mavlink.connection import MAVLinkConnection

async def test_connection():
    conn = MAVLinkConnection("udp:127.0.0.1:14550")

    if await conn.connect():
        print(f"Connected to system {conn.target_system}")
        print(f"Heartbeat: {conn.heartbeat}")
        await asyncio.sleep(5)
        await conn.disconnect()
        print("Test PASSED")
    else:
        print("Test FAILED: Could not connect")

asyncio.run(test_connection())
```

### Scenario 2: Telemetry Test

**Objective**: Verify all telemetry streams are received.

```python
# test_telemetry.py
import asyncio
from mavlink.connection import MAVLinkConnection
from mavlink.messages import MessageHandler

async def test_telemetry():
    conn = MAVLinkConnection("udp:127.0.0.1:14550")
    handler = MessageHandler()

    conn.register_all_messages_callback(handler.handle_message)

    if await conn.connect():
        # Wait for telemetry
        await conn.request_all_data_streams()
        await asyncio.sleep(5)

        t = handler.telemetry
        print(f"Position: {t.position.latitude}, {t.position.longitude}")
        print(f"Altitude: {t.position.altitude_rel}m")
        print(f"Battery: {t.battery.voltage}V, {t.battery.remaining_percent}%")
        print(f"Mode: {t.system.mode}")
        print(f"GPS: {t.gps.fix_type}, {t.gps.satellites} sats")

        await conn.disconnect()

asyncio.run(test_telemetry())
```

### Scenario 3: Arm and Takeoff Test

**Objective**: Test arming, takeoff, and landing.

```python
# test_takeoff.py
import asyncio
from ai.agent import DroneAgent

async def test_takeoff():
    agent = DroneAgent(connection_string="udp:127.0.0.1:14550")

    if not await agent.start():
        print("Connection failed")
        return

    try:
        # Arm and takeoff
        print("Arming and taking off...")
        if await agent.arm_and_takeoff(30):
            print(f"At altitude: {agent.telemetry.position.altitude_rel}m")

            # Hold for 10 seconds
            await asyncio.sleep(10)

            # Land
            print("Landing...")
            await agent.land()

            print("Test PASSED")
        else:
            print("Takeoff failed")

    finally:
        await agent.stop()

asyncio.run(test_takeoff())
```

### Scenario 4: Navigation Test

**Objective**: Test waypoint navigation.

```python
# test_navigation.py
import asyncio
from ai.agent import DroneAgent

async def test_navigation():
    agent = DroneAgent(connection_string="udp:127.0.0.1:14550")

    if not await agent.start():
        return

    try:
        await agent.arm_and_takeoff(30)

        # Go to waypoint
        target_lat = -33.871
        target_lon = 151.211

        print(f"Going to {target_lat}, {target_lon}")
        await agent.goto(target_lat, target_lon, 30)

        # Wait to arrive
        while True:
            pos = agent.telemetry.position
            dist = ((pos.latitude - target_lat)**2 +
                   (pos.longitude - target_lon)**2)**0.5 * 111000

            print(f"Distance: {dist:.1f}m")

            if dist < 5:
                print("Arrived!")
                break

            await asyncio.sleep(1)

        # Return home
        await agent.rtl()

    finally:
        await agent.stop()

asyncio.run(test_navigation())
```

### Scenario 5: Survey Pattern Test

**Objective**: Test automated survey mission.

```python
# test_survey.py
import asyncio
from ai.agent import DroneAgent

async def test_survey():
    agent = DroneAgent(connection_string="udp:127.0.0.1:14550")

    if not await agent.start():
        return

    try:
        await agent.arm_and_takeoff(50)

        # Execute survey
        print("Starting survey...")
        await agent.execute_survey(
            center_lat=-33.87,
            center_lon=151.21,
            width_m=100,
            height_m=100,
            altitude_m=50,
            spacing_m=20,
        )

        # Monitor progress
        while agent.state_machine.state.name == "SURVEY":
            print(f"State: {agent.state_machine.state.name}")
            await asyncio.sleep(5)

        print("Survey complete!")
        await agent.rtl()

    finally:
        await agent.stop()

asyncio.run(test_survey())
```

### Scenario 6: Geofence Test

**Objective**: Test geofence enforcement.

```python
# test_geofence.py
import asyncio
from ai.agent import DroneAgent

async def test_geofence():
    agent = DroneAgent(connection_string="udp:127.0.0.1:14550")

    if not await agent.start():
        return

    try:
        # Set small geofence
        agent.set_home_fence(
            radius_m=100,
            max_altitude_m=50,
        )
        print("Geofence set: 100m radius, 50m max alt")

        await agent.arm_and_takeoff(30)

        # Try to fly outside fence
        print("Flying towards fence...")
        await agent.goto(-33.872, 151.212, 30)

        # Monitor for RTL
        await asyncio.sleep(30)

        print(f"Final state: {agent.state_machine.state.name}")

    finally:
        await agent.stop()

asyncio.run(test_geofence())
```

### Scenario 7: Battery Failsafe Test

**Objective**: Test low battery RTL.

```bash
# In MAVProxy, simulate low battery
param set SIM_BATT_VOLTAGE 20.5
```

```python
# test_battery_failsafe.py
import asyncio
from ai.agent import DroneAgent

async def test_battery_failsafe():
    agent = DroneAgent(connection_string="udp:127.0.0.1:14550")

    if not await agent.start():
        return

    try:
        await agent.arm_and_takeoff(30)

        print("Monitoring battery...")
        print("Reduce battery voltage in MAVProxy:")
        print("  param set SIM_BATT_VOLTAGE 20.5")

        # Monitor for failsafe
        while True:
            status = agent.get_status()
            battery = status["battery"]
            mission = status["mission"]

            print(f"Battery: {battery['soc_percent']:.0f}%, "
                  f"State: {mission['state']}")

            if mission["state"] == "RTL":
                print("RTL triggered!")
                break

            await asyncio.sleep(2)

    finally:
        await agent.stop()

asyncio.run(test_battery_failsafe())
```

## Running Unit Tests

```bash
cd ardupilot-AI

# Run all module tests
python tests/test_modules.py

# Run with pytest
pytest tests/ -v

# Run with coverage
pytest tests/ --cov=src --cov-report=html
```

## Debugging

### Enable Debug Logging

```bash
# Set log level
export DRONE_AI_LOG_LEVEL=DEBUG

# Run with verbose output
python src/main.py --sitl -v
```

### MAVLink Message Debugging

```python
# Log all messages
def log_message(msg):
    print(f"[{msg.get_type()}] {msg}")

connection.register_all_messages_callback(log_message)
```

### Common SITL Issues

#### "Pre-arm: Throttle not zero"

In MAVProxy:
```bash
rc 3 1000  # Set throttle to minimum
```

#### "Pre-arm: GPS not healthy"

Wait for GPS lock (simulated, takes ~10 seconds).

#### "Pre-arm: Compass not calibrated"

```bash
param set COMPASS_LEARN 3  # Auto-learn compass
```

#### "Pre-arm: Check Battery"

```bash
param set ARMING_CHECK 0  # Disable arming checks (testing only!)
```

## Performance Testing

### Message Rate Test

```python
import time
import asyncio

messages_received = 0
start_time = time.time()

def count_messages(msg):
    global messages_received
    messages_received += 1

connection.register_all_messages_callback(count_messages)

# Run for 10 seconds
await asyncio.sleep(10)

elapsed = time.time() - start_time
rate = messages_received / elapsed
print(f"Message rate: {rate:.1f} msg/sec")
```

### CPU Usage Monitoring

```bash
# While running Drone AI
top -p $(pgrep -f "python.*main.py")
```

## Test Checklist

### Pre-Flight Checklist

- [ ] SITL running and connected
- [ ] GPS lock acquired (simulated)
- [ ] Battery > 50%
- [ ] No pre-arm errors
- [ ] Telemetry streaming
- [ ] Geofence configured

### Connection Tests

- [ ] UDP connection works
- [ ] TCP connection works
- [ ] Heartbeat received
- [ ] All telemetry types received
- [ ] Commands acknowledged

### Flight Control Tests

- [ ] Mode changes work
- [ ] Arming/disarming works
- [ ] Takeoff command works
- [ ] Landing command works
- [ ] Goto commands work
- [ ] RTL works
- [ ] Velocity control works

### Safety Tests

- [ ] Geofence breach detected
- [ ] Geofence RTL works
- [ ] Battery low detected
- [ ] Battery RTL works
- [ ] Link loss detected
- [ ] GPS loss handled

### Mission Tests

- [ ] Waypoint navigation works
- [ ] Survey pattern works
- [ ] Orbit pattern works
- [ ] Mission completion detected

## Continuous Integration

### GitHub Actions Example

```yaml
# .github/workflows/test.yml
name: Tests

on: [push, pull_request]

jobs:
  test:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v3

      - name: Set up Python
        uses: actions/setup-python@v4
        with:
          python-version: '3.10'

      - name: Install dependencies
        run: |
          pip install -r requirements.txt
          pip install pytest pytest-cov

      - name: Run tests
        run: |
          python tests/test_modules.py
          pytest tests/ -v --cov=src
```
