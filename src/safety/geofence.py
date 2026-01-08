"""
Geofence management for drone safety.

Defines virtual boundaries and actions when breached.
"""

import math
from dataclasses import dataclass, field
from enum import Enum
from typing import List, Optional, Tuple

import structlog

logger = structlog.get_logger()


class GeofenceAction(Enum):
    """Action when geofence is breached."""
    NONE = 0  # Report only
    WARN = 1  # Warning only
    LOITER = 2  # Hold position
    RTL = 3  # Return to launch
    LAND = 4  # Land immediately


class GeofenceType(Enum):
    """Type of geofence."""
    CIRCULAR = "circular"  # Radius around center
    POLYGON = "polygon"  # Arbitrary polygon
    ALTITUDE = "altitude"  # Altitude limits only


@dataclass
class CircularGeofence:
    """Circular geofence definition."""
    center_lat: float
    center_lon: float
    radius_m: float
    min_altitude_m: float = 0
    max_altitude_m: float = 120
    action: GeofenceAction = GeofenceAction.RTL


@dataclass
class PolygonGeofence:
    """Polygon geofence definition."""
    vertices: List[Tuple[float, float]] = field(default_factory=list)  # (lat, lon) pairs
    min_altitude_m: float = 0
    max_altitude_m: float = 120
    action: GeofenceAction = GeofenceAction.RTL
    is_inclusion: bool = True  # True = stay inside, False = stay outside


@dataclass
class GeofenceStatus:
    """Current geofence status."""
    is_inside: bool = True
    distance_to_boundary_m: float = 0.0
    breach_type: Optional[str] = None
    recommended_action: GeofenceAction = GeofenceAction.NONE


class Geofence:
    """
    Geofence manager.

    Monitors position relative to defined boundaries.
    """

    def __init__(self):
        """Initialize geofence manager."""
        self._circular_fences: List[CircularGeofence] = []
        self._polygon_fences: List[PolygonGeofence] = []
        self._enabled = True

        # Default home fence
        self._home_fence: Optional[CircularGeofence] = None

    @property
    def enabled(self) -> bool:
        """Check if geofencing is enabled."""
        return self._enabled

    def enable(self):
        """Enable geofencing."""
        self._enabled = True
        logger.info("Geofence enabled")

    def disable(self):
        """Disable geofencing."""
        self._enabled = False
        logger.warning("Geofence disabled")

    def set_home_fence(
        self,
        lat: float,
        lon: float,
        radius_m: float,
        max_altitude_m: float = 120,
        action: GeofenceAction = GeofenceAction.RTL,
    ):
        """
        Set home-centered circular fence.

        Args:
            lat: Home latitude
            lon: Home longitude
            radius_m: Maximum distance from home
            max_altitude_m: Maximum altitude
            action: Action on breach
        """
        self._home_fence = CircularGeofence(
            center_lat=lat,
            center_lon=lon,
            radius_m=radius_m,
            max_altitude_m=max_altitude_m,
            action=action,
        )
        logger.info(
            "Home fence set",
            radius=radius_m,
            max_alt=max_altitude_m,
        )

    def add_circular_fence(self, fence: CircularGeofence):
        """Add circular geofence."""
        self._circular_fences.append(fence)
        logger.info(
            "Circular fence added",
            lat=fence.center_lat,
            lon=fence.center_lon,
            radius=fence.radius_m,
        )

    def add_polygon_fence(self, fence: PolygonGeofence):
        """Add polygon geofence."""
        self._polygon_fences.append(fence)
        logger.info(
            "Polygon fence added",
            vertices=len(fence.vertices),
            inclusion=fence.is_inclusion,
        )

    def add_no_fly_zone(
        self,
        vertices: List[Tuple[float, float]],
        max_altitude_m: float = 120,
        action: GeofenceAction = GeofenceAction.RTL,
    ):
        """
        Add no-fly zone (exclusion polygon).

        Args:
            vertices: Polygon vertices (lat, lon)
            max_altitude_m: Maximum altitude
            action: Action if entered
        """
        fence = PolygonGeofence(
            vertices=vertices,
            max_altitude_m=max_altitude_m,
            action=action,
            is_inclusion=False,
        )
        self.add_polygon_fence(fence)

    def check(
        self,
        lat: float,
        lon: float,
        altitude_m: float,
    ) -> GeofenceStatus:
        """
        Check position against all geofences.

        Args:
            lat: Current latitude
            lon: Current longitude
            altitude_m: Current altitude (AGL)

        Returns:
            Geofence status
        """
        if not self._enabled:
            return GeofenceStatus(is_inside=True)

        status = GeofenceStatus(is_inside=True)
        worst_action = GeofenceAction.NONE

        # Check home fence first
        if self._home_fence:
            result = self._check_circular(lat, lon, altitude_m, self._home_fence)
            if not result.is_inside:
                status = result
                worst_action = max(worst_action, result.recommended_action, key=lambda x: x.value)

        # Check other circular fences
        for fence in self._circular_fences:
            result = self._check_circular(lat, lon, altitude_m, fence)
            if not result.is_inside:
                status.is_inside = False
                status.breach_type = result.breach_type
                worst_action = max(worst_action, result.recommended_action, key=lambda x: x.value)

        # Check polygon fences
        for fence in self._polygon_fences:
            result = self._check_polygon(lat, lon, altitude_m, fence)
            if not result.is_inside:
                status.is_inside = False
                status.breach_type = result.breach_type
                worst_action = max(worst_action, result.recommended_action, key=lambda x: x.value)

        status.recommended_action = worst_action

        if not status.is_inside:
            logger.warning(
                "Geofence breach",
                breach_type=status.breach_type,
                action=status.recommended_action.name,
            )

        return status

    def distance_to_boundary(
        self,
        lat: float,
        lon: float,
    ) -> float:
        """
        Calculate distance to nearest fence boundary.

        Args:
            lat: Current latitude
            lon: Current longitude

        Returns:
            Distance in meters (negative = outside)
        """
        min_distance = float('inf')

        if self._home_fence:
            dist = self._distance_to_circle(lat, lon, self._home_fence)
            min_distance = min(min_distance, dist)

        for fence in self._circular_fences:
            dist = self._distance_to_circle(lat, lon, fence)
            min_distance = min(min_distance, dist)

        return min_distance

    def _check_circular(
        self,
        lat: float,
        lon: float,
        alt: float,
        fence: CircularGeofence,
    ) -> GeofenceStatus:
        """Check position against circular fence."""
        distance = self._haversine_distance(
            lat, lon,
            fence.center_lat, fence.center_lon,
        )

        status = GeofenceStatus()

        # Check radius
        if distance > fence.radius_m:
            status.is_inside = False
            status.breach_type = "radius"
            status.distance_to_boundary_m = distance - fence.radius_m
            status.recommended_action = fence.action
            return status

        # Check altitude
        if alt > fence.max_altitude_m:
            status.is_inside = False
            status.breach_type = "altitude_max"
            status.distance_to_boundary_m = alt - fence.max_altitude_m
            status.recommended_action = fence.action
            return status

        if alt < fence.min_altitude_m:
            status.is_inside = False
            status.breach_type = "altitude_min"
            status.distance_to_boundary_m = fence.min_altitude_m - alt
            status.recommended_action = fence.action
            return status

        # Inside fence
        status.is_inside = True
        status.distance_to_boundary_m = fence.radius_m - distance
        return status

    def _check_polygon(
        self,
        lat: float,
        lon: float,
        alt: float,
        fence: PolygonGeofence,
    ) -> GeofenceStatus:
        """Check position against polygon fence."""
        status = GeofenceStatus()

        # Check altitude first
        if alt > fence.max_altitude_m or alt < fence.min_altitude_m:
            status.is_inside = False
            status.breach_type = "altitude"
            status.recommended_action = fence.action
            return status

        # Point in polygon test
        inside_polygon = self._point_in_polygon(lat, lon, fence.vertices)

        if fence.is_inclusion:
            # Must be inside inclusion zone
            status.is_inside = inside_polygon
            if not inside_polygon:
                status.breach_type = "outside_inclusion_zone"
        else:
            # Must be outside exclusion zone
            status.is_inside = not inside_polygon
            if inside_polygon:
                status.breach_type = "inside_exclusion_zone"

        if not status.is_inside:
            status.recommended_action = fence.action

        return status

    def _distance_to_circle(
        self,
        lat: float,
        lon: float,
        fence: CircularGeofence,
    ) -> float:
        """Distance to circular fence boundary (positive = inside)."""
        distance = self._haversine_distance(
            lat, lon,
            fence.center_lat, fence.center_lon,
        )
        return fence.radius_m - distance

    def _point_in_polygon(
        self,
        lat: float,
        lon: float,
        vertices: List[Tuple[float, float]],
    ) -> bool:
        """Test if point is inside polygon (ray casting)."""
        n = len(vertices)
        if n < 3:
            return False

        inside = False

        j = n - 1
        for i in range(n):
            if ((vertices[i][1] > lon) != (vertices[j][1] > lon) and
                lat < (vertices[j][0] - vertices[i][0]) *
                (lon - vertices[i][1]) / (vertices[j][1] - vertices[i][1]) +
                vertices[i][0]):
                inside = not inside
            j = i

        return inside

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

    def clear_fences(self):
        """Clear all geofences."""
        self._circular_fences.clear()
        self._polygon_fences.clear()
        self._home_fence = None
        logger.info("All fences cleared")

    def get_status_dict(self) -> dict:
        """Get status as dictionary."""
        return {
            "enabled": self._enabled,
            "home_fence_set": self._home_fence is not None,
            "circular_fences": len(self._circular_fences),
            "polygon_fences": len(self._polygon_fences),
        }
