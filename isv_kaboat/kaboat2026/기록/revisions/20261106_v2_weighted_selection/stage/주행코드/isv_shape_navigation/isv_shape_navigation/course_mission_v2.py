"""ROS entry point for the V2 waypoint mission."""
from .avoidance_v2 import RouteMissionV2
from .follow_node import main


if __name__ == "__main__":
    main(mode="course", route_mission_class=RouteMissionV2)
