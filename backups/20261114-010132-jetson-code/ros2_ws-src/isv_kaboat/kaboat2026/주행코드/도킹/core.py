"""Target identity and camera-ray bearing; no navigation or actuator output."""
from dataclasses import dataclass
import math
from typing import Optional

import cv2
import numpy as np

MODEL_CLASSES = (
    'blue circle', 'blue cross', 'blue triangle',
    'green circle', 'green cross', 'green triangle',
    'red circle', 'red cross', 'red triangle',
)


def validate_model_names(names):
    expected = dict(enumerate(MODEL_CLASSES))
    actual = dict(enumerate(names)) if isinstance(names, (list, tuple)) else dict(names)
    if actual != expected:
        raise ValueError(f'best3.pt class mismatch: expected={expected}; actual={actual}')


@dataclass(frozen=True)
class Detection:
    class_id: int
    confidence: float
    track_id: int
    box: tuple

    @property
    def center(self):
        x1, y1, x2, y2 = self.box
        return ((x1 + x2) / 2.0, (y1 + y2) / 2.0)

    @property
    def size(self):
        return (self.box[2] - self.box[0], self.box[3] - self.box[1])


class TargetSelector:
    """Keep identity through short gaps; ambiguous reacquisitions stay unselected.

    Once the reacquisition window expires, restart or change target_class_id to
    select a new physical marker. A reused tracker ID alone never bypasses gates.
    """
    def __init__(self, class_id=8, lost_timeout=0.8, reacquire_timeout=3.0,
                 center_gate_fraction=0.15, size_ratio_limit=2.0):
        if type(class_id) is not int or not 0 <= class_id < len(MODEL_CLASSES):
            raise ValueError('target_class_id must be an integer in 0..8')
        values = (lost_timeout, reacquire_timeout, center_gate_fraction, size_ratio_limit)
        if not all(math.isfinite(v) and v > 0 for v in values):
            raise ValueError('tracking settings must be finite and positive')
        if reacquire_timeout < lost_timeout or center_gate_fraction > 1 or size_ratio_limit < 1:
            raise ValueError('invalid tracking time/position/size gates')
        self.class_id = class_id
        self.lost_timeout = lost_timeout
        self.reacquire_timeout = reacquire_timeout
        self.center_gate_fraction = center_gate_fraction
        self.size_ratio_limit = size_ratio_limit
        self.last: Optional[Detection] = None
        self.last_seen = -math.inf

    def status(self, now):
        if self.last is None:
            return 'SEARCHING'
        return 'TRACKING' if now - self.last_seen <= self.lost_timeout else 'LOST'

    def update(self, detections, now, width, height):
        if not math.isfinite(now) or width <= 0 or height <= 0:
            raise ValueError('invalid frame size or observation time')
        candidates = [d for d in detections if d.class_id == self.class_id
                      and d.track_id >= 0 and 0 <= d.confidence <= 1
                      and all(math.isfinite(v) for v in (*d.box, d.confidence))
                      and min(d.size) > 0]
        selected = None
        if self.last is None:
            if candidates:
                selected = max(candidates, key=lambda d: (d.confidence, -d.track_id))
        elif 0 <= now - self.last_seen <= self.reacquire_timeout:
            prior = self.last
            gate = self.center_gate_fraction * math.hypot(width, height)
            eligible = []
            for d in candidates:
                ratios = [a / b for a, b in zip(d.size, prior.size)]
                if (math.dist(d.center, prior.center) <= gate
                        and all(1 / self.size_ratio_limit <= r <= self.size_ratio_limit for r in ratios)):
                    eligible.append(d)
            same_id = [d for d in eligible if d.track_id == prior.track_id]
            if len(same_id) == 1:
                selected = same_id[0]
            elif not same_id and len(eligible) == 1:
                selected = eligible[0]
        if selected is not None:
            self.last, self.last_seen = selected, now
        return selected


@dataclass(frozen=True)
class Calibration:
    width: int
    height: int
    k: tuple
    d: tuple
    distortion_model: str = 'plumb_bob'
    roi_active: bool = False
    binning_x: int = 0
    binning_y: int = 0

    def normalized_x(self, u, v, width, height):
        if (width, height) != (self.width, self.height):
            raise ValueError('CameraInfo dimensions do not match image')
        if self.roi_active or self.binning_x > 1 or self.binning_y > 1:
            raise ValueError('CameraInfo ROI/binning requires matching calibrated pixels')
        if len(self.k) != 9 or not all(math.isfinite(x) for x in (*self.k, *self.d)):
            raise ValueError('CameraInfo K/D is malformed')
        matrix = np.asarray(self.k, dtype=float).reshape(3, 3)
        if matrix[0, 0] <= 0 or matrix[1, 1] <= 0 or not np.allclose(matrix[2], [0, 0, 1]):
            raise ValueError('CameraInfo is uncalibrated or invalid')
        point = np.asarray([[[u, v]]], dtype=float)
        coefficients = np.asarray(self.d, dtype=float)
        if self.distortion_model == 'equidistant':
            if len(self.d) != 4:
                raise ValueError('equidistant CameraInfo requires four D coefficients')
            ray = cv2.fisheye.undistortPoints(point, matrix, coefficients)
        elif self.distortion_model in ('plumb_bob', 'rational_polynomial', ''):
            if len(self.d) not in (0, 4, 5, 8, 12, 14):
                raise ValueError('unsupported CameraInfo D length')
            ray = cv2.undistortPoints(point, matrix, coefficients if len(self.d) else None)
        else:
            raise ValueError('unsupported distortion model: ' + self.distortion_model)
        value = float(ray[0, 0, 0])
        if not math.isfinite(value):
            raise ValueError('invalid undistorted camera ray')
        return value


def bearing_deg(u, v, width, height, calibration=None, yaw_offset_deg=0.0,
                allow_fov_fallback=False, horizontal_fov_deg=0.0):
    """Bow 0, port positive, starboard negative; camera yaw offset port positive.

    This is the camera viewing ray expressed relative to bow orientation, not a
    parallax-corrected bearing from the bow's physical position.
    """
    if not all(math.isfinite(x) for x in (u, v, yaw_offset_deg)):
        raise ValueError('non-finite pixel/yaw')
    if width <= 0 or height <= 0 or not (0 <= u < width and 0 <= v < height):
        raise ValueError('target center outside image')
    try:
        if calibration is None:
            raise ValueError('valid CameraInfo required')
        x = calibration.normalized_x(u, v, width, height)
    except ValueError:
        if not allow_fov_fallback or not math.isfinite(horizontal_fov_deg) or not 0 < horizontal_fov_deg < 180:
            raise
        fx = width / (2 * math.tan(math.radians(horizontal_fov_deg) / 2))
        x = (u - width / 2.0) / fx
    angle = yaw_offset_deg - math.degrees(math.atan(x))
    return (angle + 180) % 360 - 180
