"""Capture ordered indoor mission waypoints from the confirmed fixed-map pose."""
import argparse
import json
import math
import os
from pathlib import Path
import tempfile
import time


def mission_points(route):
    """Return the registered ordered route, or the legacy A/B route."""
    registered = route.get('mission_waypoints')
    if registered is None:
        points = [(name, route[key]) for name, key in (('A', 'a'), ('B', 'b')) if isinstance(route.get(key),dict)]
        if not points:raise ValueError('Register at least one mission waypoint')
    else:
        entries = registered.get('points')
        if not isinstance(entries, list) or len(entries) > 2:
            raise ValueError('Mission waypoints must contain WP1 and/or WP2')
        names = ('WP1', 'WP2')[:len(entries)]
        points = [(name, point) for name, point in zip(names, entries) if isinstance(point, dict)]
        if not points:
            raise ValueError('Register at least one mission waypoint')
    result = []
    for name, point in points:
        xy = tuple(float(point[k]) for k in ('x', 'y'))
        if not all(math.isfinite(v) for v in xy):
            raise ValueError(f'{name} must have finite map coordinates')
        result.append((name, xy))
    return result


def store_waypoint(route_path, map_id, index, pose):
    path = Path(route_path).expanduser()
    route = json.loads(path.read_text(encoding='utf-8'))
    if route.get('frame_id') != 'tank_map':
        raise ValueError('Route must use tank_map coordinates')
    points = route.get('mission_waypoints', {}).get('points', [None, None])
    if not isinstance(points, list) or len(points) != 2:
        points = [None, None]
    points[index - 1] = {'x': float(pose[0]), 'y': float(pose[1])}
    route['session_id'] = map_id
    route['mission_waypoints'] = {'map_id': map_id, 'frame_id': 'tank_map', 'points': points}
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, temp_name = tempfile.mkstemp(prefix=path.name + '.', dir=path.parent)
    try:
        with os.fdopen(fd, 'w', encoding='utf-8') as stream:
            json.dump(route, stream, ensure_ascii=False, indent=2)
            stream.write('\n')
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temp_name, path)
    finally:
        if os.path.exists(temp_name):
            os.unlink(temp_name)
    return points


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('index', type=int, choices=(1, 2), help='WP1 or WP2')
    parser.add_argument('--route', default='~/water_trials/tank/route.json')
    parser.add_argument('--wait', type=float, default=0., help='optional timeout in seconds; default waits until localization is ready')
    args = parser.parse_args(argv)
    if args.wait < 0 or not math.isfinite(args.wait):
        parser.error('--wait must be non-negative')

    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from std_msgs.msg import String

    rclpy.init()

    class Capture(Node):
        def __init__(self):
            super().__init__('tank_waypoint_capture')
            self.pose = None
            self.session = None
            self.pose_at = 0.
            self.create_subscription(PoseWithCovarianceStamped, '/tank/pose', self.on_pose, 10)
            self.create_subscription(String, '/tank/session', self.on_session, 10)

        def on_pose(self, msg):
            if msg.header.frame_id != 'tank_map':
                return
            p = msg.pose.pose
            q = p.orientation
            heading = math.atan2(2 * (q.w*q.z + q.x*q.y), 1 - 2 * (q.y*q.y + q.z*q.z))
            self.pose = (p.position.x, p.position.y, heading,
                         msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9)
            self.pose_at = time.monotonic()

        def on_session(self, msg):
            try:
                self.session = json.loads(msg.data)
            except (ValueError, TypeError):
                self.session = None

    node = Capture()
    deadline = time.monotonic() + args.wait if args.wait else None
    try:
        while rclpy.ok() and (deadline is None or time.monotonic() < deadline):
            rclpy.spin_once(node, timeout_sec=.1)
            s, p = node.session, node.pose
            if (s and s.get('fixed_map') and s.get('localization_ready')
                    and s.get('session_id') and p and time.monotonic()-node.pose_at < .5
                    and all(math.isfinite(v) for v in p)):
                points = store_waypoint(args.route, s['session_id'], args.index, p[:2])
                print(f"WP{args.index} saved: x={p[0]:.3f} y={p[1]:.3f} yaw={math.degrees(p[2]):.1f} deg map={s['session_id']}")
                print('Both WP1 and WP2 are ready.' if all(points) else 'Register the other waypoint before starting the trial.')
                return
        raise RuntimeError('No fresh pose from a confirmed saved-map localization')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
