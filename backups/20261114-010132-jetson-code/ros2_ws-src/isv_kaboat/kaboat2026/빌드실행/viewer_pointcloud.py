"""Bounded, display-only XYZ snapshots; no navigation or actuator dependencies."""
import math
import threading
import time

import numpy as np


def xyz_preview(message, max_points=6000, max_range=12.0, range_mode="radial", front_only=False):
    """Read organized/unorganized PointCloud2, including padded rows/endian data."""
    width, height = int(message.width), int(message.height)
    step, row = int(message.point_step), int(message.row_step)
    if width < 0 or height < 0 or step <= 0 or row < width * step:
        raise ValueError('Invalid PointCloud2 layout')
    if max_points < 1 or not math.isfinite(max_range) or max_range <= 0:
        raise ValueError('Invalid display bounds')
    fields = {f.name: f for f in message.fields}
    endian = '>' if message.is_bigendian else '<'
    formats = {7: 'f4', 8: 'f8'}
    columns = []
    needed = (height - 1) * row + width * step if height and width else 0
    if len(message.data) < needed:
        raise ValueError('Truncated PointCloud2 data')
    for name in ('x', 'y', 'z'):
        field = fields.get(name)
        if field is None or field.datatype not in formats or field.count != 1:
            raise ValueError('XYZ float fields required')
        dtype = np.dtype(endian + formats[field.datatype])
        if field.offset < 0 or field.offset + dtype.itemsize > step:
            raise ValueError('Invalid XYZ offset')
        if not width or not height:
            columns.append(np.empty(0))
        else:
            columns.append(np.ndarray((height, width), dtype=dtype,
                buffer=message.data, offset=field.offset, strides=(row, step)).ravel())
    xyz = np.column_stack(columns)
    if range_mode not in ('radial','horizontal'):
        raise ValueError('Invalid display range mode')
    ranged=xyz[:,:2] if range_mode=='horizontal' else xyz
    distance = np.einsum('ij,ij->i', ranged, ranged)
    keep = np.isfinite(xyz).all(axis=1) & (distance >= .01) & (distance <= max_range**2)
    if front_only:
        keep &= xyz[:,0]>=0.
    xyz = xyz[keep]
    count = len(xyz)
    if count > max_points:
        # Spread samples across the whole scan rather than truncating one azimuth.
        xyz = xyz[np.linspace(0, count - 1, max_points, dtype=np.int64)]
    return dict(points=np.round(xyz.astype(np.float64), 3).tolist(), input_count=width * height,
        range_count=count, display_count=len(xyz), sampled=count > max_points,
        max_range_m=max_range,range_mode=range_mode,front_only=front_only)


class PointCloudBuffer:
    def __init__(self, clock=time.monotonic, *, max_range=12.0, range_mode="radial", front_only=False):
        self.front_only=front_only
        self.max_range=max_range
        self.range_mode=range_mode
        self.clock = clock
        self.lock = threading.Lock()
        self.requested = -math.inf
        self.processed = -math.inf
        self.received = None
        self.alignment = None
        self.aligned_at = -math.inf
        self.value = dict(points=[], frame_id=None, source_topic='/velodyne_points',
            stamp=None, display_count=0, input_count=0, range_count=0,
            max_range_m=max_range, range_mode=range_mode, front_only=front_only, sampled=False, error=None)

    def update(self, message, source_topic='/velodyne_points'):
        now = self.clock()
        with self.lock:
            if now - self.requested > 3 or now - self.processed < .45:
                return
            self.processed = now
        try:
            value = xyz_preview(message,max_range=self.max_range,range_mode=self.range_mode,front_only=self.front_only)
            value.update(frame_id=message.header.frame_id,
                stamp=message.header.stamp.sec + message.header.stamp.nanosec / 1e9,
                source_topic=source_topic, error=None)
        except (ValueError, TypeError, BufferError, OverflowError) as error:
            with self.lock:
                self.value = dict(self.value, error=str(error))
            return
        with self.lock:
            self.value, self.received = value, now

    def snapshot(self):
        now = self.clock()
        with self.lock:
            self.requested = now
            age = None if self.received is None else max(0., now - self.received)
            alignment_age = now - self.aligned_at
            alignment = dict(self.alignment or {}, valid=self.alignment is not None and alignment_age < 6,
                age_s=alignment_age if math.isfinite(alignment_age) else None)
            return dict(self.value, age_s=age, stale=age is None or age > 1.5,
                body_from_cloud=alignment)

    def set_alignment(self, frame_id, yaw_deg, x_m, y_m):
        if not isinstance(frame_id, str) or not frame_id or not all(
                math.isfinite(v) for v in (yaw_deg, x_m, y_m)):
            raise ValueError('Invalid navigation display transform')
        with self.lock:
            self.alignment = dict(frame_id=frame_id, yaw_deg=yaw_deg, x_m=x_m, y_m=y_m)
            self.aligned_at = self.clock()


def attach_navigation_alignment(node, buffer):
    """Read the running controller's planar scan transform; never change parameters."""
    from rcl_interfaces.srv import GetParameters
    client = node.create_client(GetParameters, '/course_mission/get_parameters')
    pending = [None, -math.inf]
    names = ['scan_frame', 'scan_yaw_deg', 'scan_x', 'scan_y']

    def complete(future):
        if pending[0] is not future:
            return
        pending[0] = None
        try:
            result = future.result()
            if result is None or len(result.values) != 4:
                return
            values = result.values
            if values[0].type != 4 or any(v.type not in (2, 3) for v in values[1:]):
                return
            numbers = [v.double_value if v.type == 3 else float(v.integer_value) for v in values[1:]]
            buffer.set_alignment(values[0].string_value, *numbers)
        except (ValueError, TypeError, RuntimeError):
            pass

    def refresh():
        if pending[0] is not None:
            if time.monotonic() - pending[1] < 2:
                return
            old = pending[0]
            pending[0] = None
            old.cancel()
        if not client.service_is_ready():
            return
        request = GetParameters.Request(names=names)
        future = client.call_async(request)
        pending[:] = [future, time.monotonic()]
        future.add_done_callback(complete)

    return client, node.create_timer(2., refresh)
