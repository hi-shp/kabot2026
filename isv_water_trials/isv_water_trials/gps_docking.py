"""Freeze saved-map A/B in Earth ENU once; track with GPS and IMU only."""
import math


def rotate(point, angle):
    co, si = math.cos(angle), math.sin(angle)
    return (co*point[0]-si*point[1], si*point[0]+co*point[1])


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class GpsDocking:
    def __init__(self, route, map_pose, gps_pose, origin, lidar_arm=(0., 0.), lidar_yaw_deg=0.):
        self.map_id = route['session_id']
        self.origin = tuple(origin)
        self.map_anchor = (float(map_pose['x']), float(map_pose['y']))
        self.map_yaw = math.radians(float(map_pose['yaw_deg']))
        self.lidar_arm = tuple(float(v) for v in lidar_arm)
        self.lidar_yaw = math.radians(float(lidar_yaw_deg))
        self.rotation = wrap(float(gps_pose['yaw'])+self.lidar_yaw-self.map_yaw)
        self.anchor = self.position(gps_pose)
        self.names, self.map_points, self.targets = [], [], []
        for name in ('a', 'b'):
            point = route.get(name)
            if point is None:
                continue
            xy = (float(point['x']), float(point['y']))
            delta = rotate((xy[0]-self.map_anchor[0], xy[1]-self.map_anchor[1]), self.rotation)
            self.names.append(name.upper());self.map_points.append(xy)
            self.targets.append((self.anchor[0]+delta[0], self.anchor[1]+delta[1]))
        if not self.targets or not all(math.isfinite(v) for v in
            (*self.origin, *self.map_anchor, self.map_yaw, *self.lidar_arm, self.rotation,
             *self.anchor, *(v for point in self.targets for v in point))):
            raise ValueError('Invalid GPS docking transform')
        self.index = 0
        self.legs = 0
        self.complete = False
        self.matched_stamp = float(map_pose['stamp_s'])
        self.gps_stamp = gps_pose['position_stamp_s']
        self.imu_stamp = gps_pose['heading_stamp_s']

    def position(self, gps_pose):
        offset = rotate(self.lidar_arm, gps_pose['yaw'])
        return (gps_pose['position'][0]+offset[0], gps_pose['position'][1]+offset[1])

    def control_state(self, gps_pose):
        position=gps_pose['position'] if getattr(self,'registered_a',None) is not None else self.position(gps_pose)
        return dict(gps_pose, position=list(position),
                    position_source='gps', heading_source='imu')

    def registered_return(self, registered_a):
        """Keep the operator's antenna GPS A; freeze the existing map B once."""
        if 'B' not in self.names:raise ValueError('선택한 도킹 경로에 B 후진 지점이 없습니다')
        self.registered_a=tuple(float(v) for v in registered_a)
        if not all(math.isfinite(v) for v in self.registered_a):raise ValueError('Invalid registered GPS A')
        if 'A' not in self.names:raise ValueError('선택한 도킹 경로에 A 지점이 없습니다')
        a=self.map_points[self.names.index('A')]
        self.targets=[]
        for point in self.map_points:
            delta=rotate((point[0]-a[0],point[1]-a[1]),self.rotation)
            self.targets.append((self.registered_a[0]+delta[0],self.registered_a[1]+delta[1]))
        self.index=self.names.index('B');self.legs=1

    def map_pose(self, gps_pose):
        position = self.position(gps_pose)
        delta = rotate((position[0]-self.anchor[0], position[1]-self.anchor[1]), -self.rotation)
        return dict(x=self.map_anchor[0]+delta[0], y=self.map_anchor[1]+delta[1],
            yaw_deg=math.degrees(wrap(gps_pose['yaw']+self.lidar_yaw-self.rotation)),
            stamp_s=max(gps_pose['position_stamp_s'], gps_pose['heading_stamp_s']),
            position_stamp_s=gps_pose['position_stamp_s'], heading_stamp_s=gps_pose['heading_stamp_s'],
            held=not gps_pose.get('pose_fresh', True), position_source='gps', heading_source='imu')

    def advance(self, position, radius):
        if self.complete or math.dist(position, self.targets[self.index]) > radius:
            return False
        self.legs += 1
        if self.index+1 < len(self.targets):
            self.index += 1
        else:
            self.complete = True
        return True

    def snapshot(self):
        return dict(map_id=self.map_id, origin=list(self.origin),
            map_anchor_m=list(self.map_anchor), enu_anchor_m=list(self.anchor),
            map_to_enu_yaw_deg=math.degrees(self.rotation),
            matched_stamp_s=self.matched_stamp, gps_stamp_s=self.gps_stamp, imu_stamp_s=self.imu_stamp,
            names=list(self.names), targets_enu_m=[list(p) for p in self.targets],
            position_source='gps', heading_source='imu', conversion_frozen=True,
            registered_a_enu_m=list(self.registered_a) if getattr(self,'registered_a',None) is not None else None,
            target_source='registered_a_plus_map_ab_vector' if getattr(self,'registered_a',None) is not None else 'frozen_slam_transform')
