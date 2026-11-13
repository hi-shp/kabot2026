"""Bounded, display-only XYZ snapshots; no navigation or actuator dependencies."""
import math
import threading
import time

import numpy as np


def xyz_preview(message, max_points=6000, max_range=12.0):
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
    distance = np.einsum('ij,ij->i', xyz, xyz)
    keep = np.isfinite(xyz).all(axis=1) & (distance >= .01) & (distance <= max_range**2)
    xyz = xyz[keep]
    count = len(xyz)
    if count > max_points:
        # Spread samples across the whole scan rather than truncating one azimuth.
        xyz = xyz[np.linspace(0, count - 1, max_points, dtype=np.int64)]
    return dict(points=np.round(xyz.astype(np.float64), 3).tolist(), input_count=width * height,
        range_count=count, display_count=len(xyz), sampled=count > max_points,
        max_range_m=max_range)


class PointCloudBuffer:
    def __init__(self, clock=time.monotonic):
        self.clock = clock
        self.lock = threading.Lock()
        self.requested = -math.inf
        self.processed = -math.inf
        self.received = None
        self.value = dict(points=[], frame_id=None, source_topic='/velodyne_points',
            stamp=None, display_count=0, input_count=0, range_count=0,
            max_range_m=12., sampled=False, error=None)

    def update(self, message):
        now = self.clock()
        with self.lock:
            if now - self.requested > 3 or now - self.processed < .45:
                return
            self.processed = now
        try:
            value = xyz_preview(message)
            value.update(frame_id=message.header.frame_id,
                stamp=message.header.stamp.sec + message.header.stamp.nanosec / 1e9,
                source_topic='/velodyne_points', error=None)
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
            return dict(self.value, age_s=age, stale=age is None or age > 1.5)
