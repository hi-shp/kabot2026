"""Generate GPS points around a buoy selected on the map.

The GUI can call buoy_circle_points(latitude, longitude). Coordinates are in
(latitude, longitude) order. The points start at north and proceed clockwise.
To draw a closed line, connect the last point back to the first.
"""

import argparse
import json
import math


EARTH_RADIUS_M = 6_371_008.8
SEARCH_RADIUS_M = 3.0
DEFAULT_POINT_COUNT = 12


def buoy_circle_points(latitude: float, longitude: float,
                       point_count: int = DEFAULT_POINT_COUNT, *,
                       radius_m: float = SEARCH_RADIUS_M) -> list[tuple[float, float]]:
    """Return evenly spaced points around a center using a spherical Earth model."""
    if not -90.0 <= latitude <= 90.0 or not -180.0 <= longitude <= 180.0:
        raise ValueError("Latitude must be within ±90° and longitude within ±180°.")
    if type(point_count) is not int or point_count < 3:
        raise ValueError("Point count must be an integer of at least 3.")
    if not math.isfinite(radius_m) or radius_m <= 0:
        raise ValueError("Radius must be positive and finite.")

    lat = math.radians(latitude)
    lon = math.radians(longitude)
    distance = radius_m / EARTH_RADIUS_M
    sin_lat, cos_lat = math.sin(lat), math.cos(lat)
    sin_distance, cos_distance = math.sin(distance), math.cos(distance)
    center = (cos_lat * math.cos(lon), cos_lat * math.sin(lon), sin_lat)
    north = (-sin_lat * math.cos(lon), -sin_lat * math.sin(lon), cos_lat)
    east = (-math.sin(lon), math.cos(lon), 0.0)

    points = []
    for index in range(point_count):
        bearing = math.tau * index / point_count
        north_offset = sin_distance * math.cos(bearing)
        east_offset = sin_distance * math.sin(bearing)
        x, y, z = (
            origin * cos_distance + north_axis * north_offset + east_axis * east_offset
            for origin, north_axis, east_axis in zip(center, north, east)
        )
        points.append((math.degrees(math.atan2(z, math.hypot(x, y))),
                       math.degrees(math.atan2(y, x))))
    return points


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("latitude", type=float)
    parser.add_argument("longitude", type=float)
    parser.add_argument("--points", type=int, default=DEFAULT_POINT_COUNT)
    args = parser.parse_args()
    try:
        points = buoy_circle_points(args.latitude, args.longitude, args.points)
    except ValueError as error:
        parser.error(str(error))
    print(json.dumps({
        "center": [args.latitude, args.longitude],
        "radius_m": SEARCH_RADIUS_M,
        "points": points,
    }, indent=2))


if __name__ == "__main__":
    main()
