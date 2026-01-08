#!/usr/bin/env python3
"""
ArduPilot Drone AI - Module Tests

Tests all core modules without requiring a live MAVLink connection.
"""

import sys
from pathlib import Path

# Add src to path
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

test_results = {}


def log_test(name: str, passed: bool, message: str = ""):
    """Log test result."""
    status = "PASS" if passed else "FAIL"
    test_results[name] = passed
    print(f"  [{status}] {name}")
    if message:
        print(f"        {message}")


def test_configuration():
    """Test configuration module."""
    print("\n" + "=" * 60)
    print("Testing Configuration Module")
    print("=" * 60)

    try:
        from config import DroneConfig, DroneType

        config = DroneConfig()

        # Check defaults
        log_test("Default config creation", True)
        log_test("Drone type", config.drone_type == DroneType.QUADCOPTER)
        log_test("MAVLink connection", config.mavlink.connection_string == "udp:127.0.0.1:14550")
        log_test("Altitude limits", config.flight.altitude_max_m == 120.0)
        log_test("Battery capacity", config.battery.capacity_mah == 10000.0)

        # Validate
        config.validate()
        log_test("Config validation", True)

        print(f"  Battery Wh: {config.battery.capacity_wh:.1f}")

    except Exception as e:
        log_test("Configuration", False, str(e))


def test_battery_manager():
    """Test battery manager."""
    print("\n" + "=" * 60)
    print("Testing Battery Manager")
    print("=" * 60)

    try:
        from energy.battery import BatteryManager, BatteryState

        battery = BatteryManager()

        # Full battery
        battery.update(voltage=25.2, current=5.0, remaining_percent=95)
        log_test("Full battery state", battery.state == BatteryState.FULL)

        # Discharging
        battery.update(voltage=22.2, current=15.0, remaining_percent=60)
        log_test("Good battery state", battery.state == BatteryState.GOOD)

        # Low battery
        battery.update(voltage=21.0, current=10.0, remaining_percent=22)
        log_test("Low battery state", battery.state == BatteryState.CRITICAL)
        log_test("Is critical", battery.is_critical)

        # Check predictions
        battery.update(voltage=23.0, current=10.0, remaining_percent=75)
        trend = battery.get_voltage_trend()
        log_test("Voltage trend", isinstance(trend, float))

        print(f"  SOC: {battery.soc:.1f}%")
        print(f"  State: {battery.state.value}")

    except Exception as e:
        log_test("Battery Manager", False, str(e))


def test_flight_predictor():
    """Test flight time predictor."""
    print("\n" + "=" * 60)
    print("Testing Flight Time Predictor")
    print("=" * 60)

    try:
        from energy.battery import BatteryManager
        from energy.predictor import FlightTimePredictor

        battery = BatteryManager()
        battery.update(voltage=22.2, current=15.0, remaining_percent=75)

        predictor = FlightTimePredictor(battery)

        # Update with flight data
        predictor.update(
            groundspeed_ms=10.0,
            current_a=15.0,
            distance_to_home_m=500.0,
        )

        # Get prediction
        prediction = predictor.predict(500.0)
        log_test("Prediction created", prediction is not None)
        log_test("Time remaining", prediction.time_remaining_s > 0)

        print(f"  Time remaining: {prediction.time_remaining_s:.0f}s")
        print(f"  Effective range: {prediction.effective_range_m:.0f}m")

        # Check RTL
        should_rtl, reason = predictor.should_rtl(500.0)
        log_test("RTL check", isinstance(should_rtl, bool))

    except Exception as e:
        log_test("Flight Predictor", False, str(e))


def test_navigator():
    """Test navigation module."""
    print("\n" + "=" * 60)
    print("Testing Navigator")
    print("=" * 60)

    try:
        # Import directly from module file to avoid relative import issues
        import importlib.util
        spec = importlib.util.spec_from_file_location(
            "navigation",
            str(Path(__file__).parent.parent / "src" / "flight" / "navigation.py")
        )
        nav_module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(nav_module)
        Navigator = nav_module.Navigator
        Waypoint = nav_module.Waypoint

        nav = Navigator()
        nav.set_home(-33.87, 151.21, 0)

        # Add waypoints
        nav.add_waypoint(-33.871, 151.211, 30)
        nav.add_waypoint(-33.872, 151.212, 30)
        log_test("Waypoints added", len(nav.mission.waypoints) == 2)

        # Survey pattern
        pattern = nav.create_survey_pattern(
            center_lat=-33.87,
            center_lon=151.21,
            width_m=100,
            height_m=100,
            altitude_m=50,
            spacing_m=20,
        )
        log_test("Survey pattern", len(pattern) > 0)
        print(f"  Survey waypoints: {len(pattern)}")

        # Orbit pattern
        orbit = nav.create_orbit_pattern(
            center_lat=-33.87,
            center_lon=151.21,
            radius_m=50,
            altitude_m=30,
            num_points=8,
        )
        log_test("Orbit pattern", len(orbit) == 8)

        # Distance calculation
        wp = Waypoint(-33.871, 151.211, 30)
        distance = nav.distance_to_waypoint(-33.87, 151.21, wp)
        log_test("Distance calculation", distance > 0)
        print(f"  Distance to waypoint: {distance:.1f}m")

        # Bearing
        bearing = nav.bearing_to_waypoint(-33.87, 151.21, wp)
        log_test("Bearing calculation", 0 <= bearing < 360)

    except Exception as e:
        log_test("Navigator", False, str(e))


def test_geofence():
    """Test geofence module."""
    print("\n" + "=" * 60)
    print("Testing Geofence")
    print("=" * 60)

    try:
        from safety.geofence import Geofence, GeofenceAction

        fence = Geofence()

        # Set home fence
        fence.set_home_fence(
            lat=-33.87,
            lon=151.21,
            radius_m=500,
            max_altitude_m=100,
        )
        log_test("Home fence set", True)

        # Check inside
        status = fence.check(-33.87, 151.21, 50)
        log_test("Inside fence", status.is_inside)

        # Check outside (radius)
        status = fence.check(-33.88, 151.22, 50)
        log_test("Outside radius", not status.is_inside)
        log_test("Breach type", status.breach_type == "radius")

        # Check altitude breach
        status = fence.check(-33.87, 151.21, 150)
        log_test("Altitude breach", not status.is_inside)

        # Distance to boundary
        dist = fence.distance_to_boundary(-33.87, 151.21)
        log_test("Distance to boundary", dist > 0)
        print(f"  Distance to boundary: {dist:.1f}m")

    except Exception as e:
        log_test("Geofence", False, str(e))


def test_failsafe():
    """Test failsafe module."""
    print("\n" + "=" * 60)
    print("Testing Failsafe Manager")
    print("=" * 60)

    try:
        from safety.failsafe import FailsafeManager, FailsafeType, FailsafeAction

        failsafe = FailsafeManager()

        # Normal conditions
        state = failsafe.update(
            battery_percent=75,
            gps_ok=True,
            link_ok=True,
            ekf_ok=True,
            in_geofence=True,
        )
        log_test("Normal - no failsafe", not state.is_failsafe_active)

        # Low battery
        state = failsafe.update(
            battery_percent=20,
            gps_ok=True,
            link_ok=True,
            ekf_ok=True,
            in_geofence=True,
        )
        log_test("Low battery failsafe", FailsafeType.BATTERY_LOW in state.active_failsafes)

        # Critical battery
        state = failsafe.update(
            battery_percent=10,
            gps_ok=True,
            link_ok=True,
            ekf_ok=True,
            in_geofence=True,
        )
        log_test("Critical battery failsafe", FailsafeType.BATTERY_CRITICAL in state.active_failsafes)

        # Geofence breach
        state = failsafe.update(
            battery_percent=75,
            gps_ok=True,
            link_ok=True,
            ekf_ok=True,
            in_geofence=False,
        )
        log_test("Geofence failsafe", FailsafeType.GEOFENCE_BREACH in state.active_failsafes)

        print(f"  Active failsafes: {[fs.name for fs in state.active_failsafes]}")

    except Exception as e:
        log_test("Failsafe Manager", False, str(e))


def test_state_machine():
    """Test mission state machine."""
    print("\n" + "=" * 60)
    print("Testing State Machine")
    print("=" * 60)

    try:
        # Import directly from module file to avoid relative import issues
        import importlib.util
        spec = importlib.util.spec_from_file_location(
            "state_machine",
            str(Path(__file__).parent.parent / "src" / "ai" / "state_machine.py")
        )
        sm_module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(sm_module)
        MissionStateMachine = sm_module.MissionStateMachine
        MissionState = sm_module.MissionState

        sm = MissionStateMachine()

        # Initial state
        log_test("Initial state", sm.state == MissionState.IDLE)

        # Valid transitions
        sm.transition(MissionState.PREFLIGHT, "Starting")
        log_test("Preflight transition", sm.state == MissionState.PREFLIGHT)

        sm.transition(MissionState.ARMING, "Arming")
        log_test("Arming transition", sm.state == MissionState.ARMING)

        sm.transition(MissionState.TAKEOFF, "Takeoff")
        log_test("Takeoff transition", sm.state == MissionState.TAKEOFF)

        sm.transition(MissionState.LOITER, "Hover")
        log_test("Loiter transition", sm.state == MissionState.LOITER)

        # Invalid transition
        result = sm.transition(MissionState.LANDED, "Invalid")
        log_test("Invalid transition blocked", not result)

        # RTL
        sm.transition(MissionState.RTL, "Return")
        log_test("RTL transition", sm.state == MissionState.RTL)

        # Flying check - RTL is still considered flying
        log_test("Is flying", sm.is_flying())

        print(f"  State history: {len(sm.history)} transitions")

    except Exception as e:
        log_test("State Machine", False, str(e))


def test_telemetry_structures():
    """Test telemetry data structures."""
    print("\n" + "=" * 60)
    print("Testing Telemetry Structures")
    print("=" * 60)

    try:
        from mavlink.messages import (
            Telemetry, Position, Velocity, Attitude, Battery, GPS,
            FlightMode, GPSFixType
        )

        telem = Telemetry()

        # Set position
        telem.position = Position(
            latitude=-33.87,
            longitude=151.21,
            altitude_msl=100,
            altitude_rel=50,
            heading=90,
        )
        log_test("Position set", telem.position.altitude_rel == 50)

        # Set attitude
        telem.attitude = Attitude(
            roll=0.1,
            pitch=0.05,
            yaw=1.57,
        )
        log_test("Attitude set", abs(telem.attitude.roll_deg - 5.7) < 0.1)

        # Set battery
        telem.battery = Battery(
            voltage=22.2,
            current=15.0,
            remaining_percent=75,
            cell_count=6,
        )
        log_test("Battery set", abs(telem.battery.cell_voltage - 3.7) < 0.01)

        # Set GPS
        telem.gps = GPS(
            fix_type=GPSFixType.FIX_3D,
            satellites=12,
            hdop=1.2,
        )
        log_test("GPS set", telem.gps.fix_type == GPSFixType.FIX_3D)

        # Flight modes
        log_test("STABILIZE mode", FlightMode.STABILIZE.value == 0)
        log_test("GUIDED mode", FlightMode.GUIDED.value == 4)
        log_test("RTL mode", FlightMode.RTL.value == 6)

        # Convert to dict
        data = telem.to_dict()
        log_test("To dict", "position" in data and "battery" in data)

        print(f"  Position: {telem.position.latitude:.4f}, {telem.position.longitude:.4f}")
        print(f"  Altitude: {telem.position.altitude_rel}m")

    except Exception as e:
        log_test("Telemetry Structures", False, str(e))


def print_summary():
    """Print test summary."""
    print("\n" + "=" * 60)
    print("TEST SUMMARY")
    print("=" * 60)

    passed = sum(1 for v in test_results.values() if v)
    total = len(test_results)

    for name, result in test_results.items():
        status = "PASS" if result else "FAIL"
        print(f"  [{status}] {name}")

    print("\n" + "-" * 60)
    print(f"Results: {passed}/{total} tests passed")

    if passed == total:
        print("\nAll tests passed!")
        return 0
    else:
        print(f"\n{total - passed} tests failed")
        return 1


if __name__ == "__main__":
    print("=" * 60)
    print("ARDUPILOT DRONE AI - MODULE TESTS")
    print("=" * 60)

    test_configuration()
    test_battery_manager()
    test_flight_predictor()
    test_navigator()
    test_geofence()
    test_failsafe()
    test_state_machine()
    test_telemetry_structures()

    sys.exit(print_summary())
