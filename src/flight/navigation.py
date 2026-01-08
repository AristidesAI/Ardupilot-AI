"""
Navigation and waypoint management.

Handles waypoint missions, survey patterns, and path planning.
"""

import math
from dataclasses import dataclass, field
from enum import Enum
from typing import List, Optional, Tuple

import structlog

logger = structlog.get_logger()


class WaypointType(Enum):
    """Waypoint types."""
    WAYPOINT = 16  # MAV_CMD_NAV_WAYPOINT
    LOITER = 17  # MAV_CMD_NAV_LOITER_UNLIM
    LOITER_TURNS = 18  # MAV_CMD_NAV_LOITER_TURNS
    LOITER_TIME = 19  # MAV_CMD_NAV_LOITER_TIME
    RTL = 20  # MAV_CMD_NAV_RETURN_TO_LAUNCH
    LAND = 21  # MAV_CMD_NAV_LAND
    TAKEOFF = 22  # MAV_CMD_NAV_TAKEOFF
    SPLINE = 82  # MAV_CMD_NAV_SPLINE_WAYPOINT
    DELAY = 93  # MAV_CMD_NAV_DELAY


@dataclass
class Waypoint:
    """Waypoint definition."""
    latitude: float
    longitude: float
    altitude: float  # Meters (relative to home)
    wp_type: WaypointType = WaypointType.WAYPOINT
    hold_time_s: float = 0  # Time to hold at waypoint
    accept_radius_m: float = 2.0  # Waypoint acceptance radius
    yaw_deg: Optional[float] = None  # Target yaw (None = auto)
    speed_ms: Optional[float] = None  # Speed to waypoint

    def to_tuple(self) -> Tuple[float, float, float]:
        """Get as (lat, lon, alt) tuple."""
        return (self.latitude, self.longitude, self.altitude)


@dataclass
class Mission:
    """Mission definition."""
    waypoints: List[Waypoint] = field(default_factory=list)
    current_index: int = 0
    auto_continue: bool = True
    home_lat: float = 0.0
    home_lon: float = 0.0
    home_alt: float = 0.0

    @property
    def is_complete(self) -> bool:
        """Check if mission is complete."""
        return self.current_index >= len(self.waypoints)

    @property
    def current_waypoint(self) -> Optional[Waypoint]:
        """Get current waypoint."""
        if 0 <= self.current_index < len(self.waypoints):
            return self.waypoints[self.current_index]
        return None

    def add_waypoint(self, wp: Waypoint):
        """Add waypoint to mission."""
        self.waypoints.append(wp)

    def clear(self):
        """Clear all waypoints."""
        self.waypoints.clear()
        self.current_index = 0


class Navigator:
    """
    Navigation manager.

    Handles mission planning, waypoint generation, and navigation commands.
    """

    def __init__(self):
        """Initialize navigator."""
        self._mission = Mission()
        self._is_navigating = False

    @property
    def mission(self) -> Mission:
        """Get current mission."""
        return self._mission

    @property
    def is_navigating(self) -> bool:
        """Check if actively navigating."""
        return self._is_navigating

    def set_home(self, lat: float, lon: float, alt: float = 0):
        """
        Set home position.

        Args:
            lat: Home latitude
            lon: Home longitude
            alt: Home altitude
        """
        self._mission.home_lat = lat
        self._mission.home_lon = lon
        self._mission.home_alt = alt

    def add_waypoint(
        self,
        lat: float,
        lon: float,
        alt: float,
        hold_time_s: float = 0,
        yaw_deg: Optional[float] = None,
    ):
        """
        Add waypoint to mission.

        Args:
            lat: Waypoint latitude
            lon: Waypoint longitude
            alt: Waypoint altitude (meters AGL)
            hold_time_s: Time to hold at waypoint
            yaw_deg: Target yaw (None = auto)
        """
        wp = Waypoint(
            latitude=lat,
            longitude=lon,
            altitude=alt,
            hold_time_s=hold_time_s,
            yaw_deg=yaw_deg,
        )
        self._mission.add_waypoint(wp)
        logger.debug("Waypoint added", lat=lat, lon=lon, alt=alt)

    def create_survey_pattern(
        self,
        center_lat: float,
        center_lon: float,
        width_m: float,
        height_m: float,
        altitude_m: float,
        spacing_m: float,
        heading_deg: float = 0,
    ) -> List[Waypoint]:
        """
        Create survey/lawn-mower pattern.

        Args:
            center_lat: Center latitude
            center_lon: Center longitude
            width_m: Survey width (meters)
            height_m: Survey height (meters)
            altitude_m: Survey altitude
            spacing_m: Line spacing (meters)
            heading_deg: Pattern heading (degrees)

        Returns:
            List of waypoints
        """
        waypoints = []

        # Calculate number of lines
        num_lines = int(height_m / spacing_m) + 1

        # Generate pattern
        for i in range(num_lines):
            y_offset = -height_m / 2 + i * spacing_m

            if i % 2 == 0:
                # Left to right
                x_offsets = [-width_m / 2, width_m / 2]
            else:
                # Right to left
                x_offsets = [width_m / 2, -width_m / 2]

            for x_offset in x_offsets:
                # Rotate by heading
                heading_rad = math.radians(heading_deg)
                rot_x = x_offset * math.cos(heading_rad) - y_offset * math.sin(heading_rad)
                rot_y = x_offset * math.sin(heading_rad) + y_offset * math.cos(heading_rad)

                # Convert to lat/lon
                lat, lon = self._offset_to_latlon(
                    center_lat, center_lon, rot_x, rot_y
                )

                wp = Waypoint(
                    latitude=lat,
                    longitude=lon,
                    altitude=altitude_m,
                )
                waypoints.append(wp)

        logger.info(
            "Survey pattern created",
            waypoints=len(waypoints),
            width=width_m,
            height=height_m,
        )

        return waypoints

    def create_orbit_pattern(
        self,
        center_lat: float,
        center_lon: float,
        radius_m: float,
        altitude_m: float,
        num_points: int = 12,
        start_heading_deg: float = 0,
    ) -> List[Waypoint]:
        """
        Create circular orbit pattern.

        Args:
            center_lat: Center latitude
            center_lon: Center longitude
            radius_m: Orbit radius (meters)
            altitude_m: Orbit altitude
            num_points: Number of waypoints
            start_heading_deg: Starting heading

        Returns:
            List of waypoints
        """
        waypoints = []

        for i in range(num_points):
            angle = start_heading_deg + (360 / num_points) * i
            angle_rad = math.radians(angle)

            x_offset = radius_m * math.sin(angle_rad)
            y_offset = radius_m * math.cos(angle_rad)

            lat, lon = self._offset_to_latlon(
                center_lat, center_lon, x_offset, y_offset
            )

            wp = Waypoint(
                latitude=lat,
                longitude=lon,
                altitude=altitude_m,
                yaw_deg=angle + 180,  # Face center
            )
            waypoints.append(wp)

        logger.info(
            "Orbit pattern created",
            waypoints=len(waypoints),
            radius=radius_m,
        )

        return waypoints

    def create_spiral_pattern(
        self,
        center_lat: float,
        center_lon: float,
        start_radius_m: float,
        end_radius_m: float,
        altitude_m: float,
        turns: float = 3,
        points_per_turn: int = 8,
    ) -> List[Waypoint]:
        """
        Create spiral pattern.

        Args:
            center_lat: Center latitude
            center_lon: Center longitude
            start_radius_m: Starting radius
            end_radius_m: Ending radius
            altitude_m: Spiral altitude
            turns: Number of turns
            points_per_turn: Points per revolution

        Returns:
            List of waypoints
        """
        waypoints = []
        total_points = int(turns * points_per_turn)

        for i in range(total_points + 1):
            t = i / total_points
            angle = t * turns * 360
            radius = start_radius_m + t * (end_radius_m - start_radius_m)

            angle_rad = math.radians(angle)
            x_offset = radius * math.sin(angle_rad)
            y_offset = radius * math.cos(angle_rad)

            lat, lon = self._offset_to_latlon(
                center_lat, center_lon, x_offset, y_offset
            )

            wp = Waypoint(
                latitude=lat,
                longitude=lon,
                altitude=altitude_m,
            )
            waypoints.append(wp)

        logger.info(
            "Spiral pattern created",
            waypoints=len(waypoints),
            turns=turns,
        )

        return waypoints

    def add_pattern(self, waypoints: List[Waypoint]):
        """
        Add pattern waypoints to mission.

        Args:
            waypoints: List of waypoints
        """
        for wp in waypoints:
            self._mission.add_waypoint(wp)

    def clear_mission(self):
        """Clear current mission."""
        self._mission.clear()
        logger.info("Mission cleared")

    def distance_to_waypoint(
        self,
        current_lat: float,
        current_lon: float,
        waypoint: Waypoint,
    ) -> float:
        """
        Calculate distance to waypoint.

        Args:
            current_lat: Current latitude
            current_lon: Current longitude
            waypoint: Target waypoint

        Returns:
            Distance in meters
        """
        return self._haversine_distance(
            current_lat, current_lon,
            waypoint.latitude, waypoint.longitude,
        )

    def bearing_to_waypoint(
        self,
        current_lat: float,
        current_lon: float,
        waypoint: Waypoint,
    ) -> float:
        """
        Calculate bearing to waypoint.

        Args:
            current_lat: Current latitude
            current_lon: Current longitude
            waypoint: Target waypoint

        Returns:
            Bearing in degrees
        """
        lat1 = math.radians(current_lat)
        lat2 = math.radians(waypoint.latitude)
        dlon = math.radians(waypoint.longitude - current_lon)

        x = math.sin(dlon) * math.cos(lat2)
        y = (math.cos(lat1) * math.sin(lat2) -
             math.sin(lat1) * math.cos(lat2) * math.cos(dlon))

        bearing = math.degrees(math.atan2(x, y))
        return (bearing + 360) % 360

    def total_mission_distance(self) -> float:
        """
        Calculate total mission distance.

        Returns:
            Total distance in meters
        """
        total = 0.0
        waypoints = self._mission.waypoints

        # Distance from home to first waypoint
        if waypoints:
            total += self._haversine_distance(
                self._mission.home_lat, self._mission.home_lon,
                waypoints[0].latitude, waypoints[0].longitude,
            )

        # Distance between waypoints
        for i in range(len(waypoints) - 1):
            total += self._haversine_distance(
                waypoints[i].latitude, waypoints[i].longitude,
                waypoints[i + 1].latitude, waypoints[i + 1].longitude,
            )

        return total

    def _offset_to_latlon(
        self,
        lat: float,
        lon: float,
        x_m: float,
        y_m: float,
    ) -> Tuple[float, float]:
        """Convert meter offset to lat/lon."""
        # Earth radius
        R = 6371000

        # Latitude offset
        dlat = y_m / R
        new_lat = lat + math.degrees(dlat)

        # Longitude offset (accounts for latitude)
        dlon = x_m / (R * math.cos(math.radians(lat)))
        new_lon = lon + math.degrees(dlon)

        return new_lat, new_lon

    def _haversine_distance(
        self,
        lat1: float,
        lon1: float,
        lat2: float,
        lon2: float,
    ) -> float:
        """Calculate great-circle distance."""
        R = 6371000  # Earth radius

        phi1 = math.radians(lat1)
        phi2 = math.radians(lat2)
        dphi = math.radians(lat2 - lat1)
        dlambda = math.radians(lon2 - lon1)

        a = (math.sin(dphi / 2) ** 2 +
             math.cos(phi1) * math.cos(phi2) *
             math.sin(dlambda / 2) ** 2)
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

        return R * c
