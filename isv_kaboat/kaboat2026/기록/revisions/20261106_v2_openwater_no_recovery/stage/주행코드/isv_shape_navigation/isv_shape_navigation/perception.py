"""Untrained colour/contour shape detector. Not a learned classifier."""
import math
import cv2
import numpy as np
from .core import Detection


class Camera:
    """Rectify raw images/pixels with CameraInfo, or explicit rectified intrinsics."""
    def __init__(self, fx=0., cx=0., width=0, height=0):
        self.fx, self.cx, self.width, self.height = fx, cx, width, height
        self.k, self.d = None, None
        self.map1, self.map2 = None, None

    def configure(self, width, height, k, d, model='plumb_bob'):
        k = np.asarray(k, dtype=float).reshape(3, 3)
        d = np.asarray(d, dtype=float)
        if (model not in ('plumb_bob', 'rational_polynomial', '') or width <= 0 or height <= 0
                or not np.isfinite(k).all() or not np.isfinite(d).all()
                or k[0, 0] <= 0 or k[1, 1] <= 0 or k[2, 2] != 1):
            raise ValueError('Valid calibrated pinhole CameraInfo is required')
        if len(d) not in (0, 4, 5, 8, 12, 14):
            raise ValueError('Unsupported distortion coefficient count')
        self.k, self.d = k, d if len(d) else np.zeros(5)
        self.fx, self.cx, self.width, self.height = float(k[0, 0]), float(k[0, 2]), width, height
        self.map1, self.map2 = cv2.initUndistortRectifyMap(k, self.d, None, k, (width, height), cv2.CV_32FC1)

    def valid(self, width, height):
        return (math.isfinite(self.fx) and self.fx > 0 and math.isfinite(self.cx)
                and self.width == width and self.height == height and 0 <= self.cx < width)

    def rectify(self, frame):
        if not self.valid(frame.shape[1], frame.shape[0]):
            raise ValueError('Missing calibration or calibrated image size differs')
        if self.map1 is None:
            return frame
        return cv2.remap(frame, self.map1, self.map2, cv2.INTER_LINEAR)

    def rectify_detection(self, detection):
        if self.k is None:
            return detection
        d = detection
        pixels = np.array([[[d.cx-d.width/2, d.cy-d.height/2]],
                           [[d.cx+d.width/2, d.cy+d.height/2]], [[d.cx, d.cy]]], dtype=float)
        rectified = cv2.undistortPoints(pixels, self.k, self.d, P=self.k)[:, 0]
        return Detection(d.label, d.score, float(rectified[2, 0]), float(rectified[2, 1]),
                         abs(float(rectified[1, 0]-rectified[0, 0])),
                         abs(float(rectified[1, 1]-rectified[0, 1])), d.stamp)


def detect_shapes(frame, stamp, color='red', min_area=250., max_area_fraction=.45):
    """Return geometric labels in rectified image pixels, with heuristic scores."""
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    masks = {
        'red': [((0, 85, 65), (12, 255, 255)), ((168, 85, 65), (179, 255, 255))],
        'green': [((35, 70, 55), (90, 255, 255))],
        'blue': [((92, 80, 55), (135, 255, 255))],
        'yellow': [((18, 80, 70), (35, 255, 255))],
        'white': [((0, 0, 150), (179, 65, 255))],
        'black': [((0, 0, 0), (179, 255, 65))],
    }
    if color not in masks:
        raise ValueError('Unsupported target_color: '+color)
    mask = np.zeros(frame.shape[:2], dtype=np.uint8)
    for low, high in masks[color]:
        mask |= cv2.inRange(hsv, np.array(low), np.array(high))
    kernel = np.ones((3, 3), dtype=np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    found = []
    h, w = frame.shape[:2]
    for contour in contours:
        area = cv2.contourArea(contour)
        if not min_area <= area <= w*h*max_area_fraction:
            continue
        x, y, bw, bh = cv2.boundingRect(contour)
        if x <= 1 or y <= 1 or x+bw >= w-1 or y+bh >= h-1 or min(bw, bh) < 8:
            continue
        perimeter = cv2.arcLength(contour, True)
        if perimeter <= 0:
            continue
        polygon = cv2.approxPolyDP(contour, .025*perimeter, True)
        hull_area = cv2.contourArea(cv2.convexHull(contour))
        solidity = area/max(1., hull_area)
        circularity = 4*math.pi*area/(perimeter*perimeter)
        vertices = len(polygon)
        convex = cv2.isContourConvex(polygon)
        label = None
        if convex and vertices == 3 and solidity > .9:
            label = 'triangle'
        elif convex and vertices == 4 and solidity > .9:
            # Rotation-invariant aspect ratio, measured on the minimum rectangle.
            rw, rh = cv2.minAreaRect(contour)[1]
            ratio = max(rw, rh)/max(1., min(rw, rh))
            rectangularity = area/max(1., rw*rh)
            if rectangularity > .82:
                label = 'square' if ratio < 1.18 else 'rectangle'
        elif convex and vertices == 5 and solidity > .9:
            label = 'pentagon'
        elif vertices >= 7 and circularity > .78 and solidity > .93:
            label = 'circle'
        elif 10 <= vertices <= 14 and .45 < solidity < .82 and .65 < bw/bh < 1.55:
            label = 'cross'
        if label:
            found.append(Detection(label, float(min(.99, .8+.19*solidity)),
                                   x+bw/2, y+bh/2, float(bw), float(bh), stamp))
    return sorted(found, key=lambda d: d.width*d.height, reverse=True)


def draw_detections(frame, detections, target):
    result = frame.copy()
    for d in detections:
        color = (0, 220, 0) if d.label == target else (160, 160, 160)
        x, y = round(d.cx-d.width/2), round(d.cy-d.height/2)
        cv2.rectangle(result, (x, y), (round(x+d.width), round(y+d.height)), color, 2)
        cv2.putText(result, d.label, (x, max(18, y-6)), cv2.FONT_HERSHEY_SIMPLEX, .6, color, 2)
    return result
