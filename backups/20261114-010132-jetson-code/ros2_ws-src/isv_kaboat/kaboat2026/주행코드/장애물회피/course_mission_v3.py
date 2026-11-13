"""ROS entry point for the V3 waypoint mission."""
from .avoidance_v3 import RouteMissionV3
from isv_shape_navigation.follow_node import main as _ros_main


def main(args=None):
    _ros_main(args, mode="course", route_mission_class=RouteMissionV3)


if __name__ == "__main__":
    main()
