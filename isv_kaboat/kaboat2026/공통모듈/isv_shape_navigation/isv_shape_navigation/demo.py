"""Deterministic offline camera -> scan -> SLAM map -> mission demonstration."""
import argparse
import csv
import json
import math
from pathlib import Path
import cv2
import numpy as np
from .core import Mission, Pose, Scan, fuse_detection, thrust_to_pwm, wrap
from .perception import detect_shapes, draw_detections
from .planner import Grid


def scene(pose, stamp):
    """Synthetic sign on wall at x=8, y=5. All data are generated, not field data."""
    dx, dy = 8.-pose.x, 5.-pose.y
    forward = math.cos(pose.yaw)*dx+math.sin(pose.yaw)*dy
    left = -math.sin(pose.yaw)*dx+math.cos(pose.yaw)*dy
    frame = np.full((480, 640, 3), 35, dtype=np.uint8)
    if forward > .3:
        u = 320-500*left/forward
        half = min(100, 500*.35/forward)
        polygon = np.rint([[u, 240-half], [u-half, 240+half], [u+half, 240+half]]).astype(np.int32)
        cv2.fillPoly(frame, [polygon], (0, 0, 255))
    angles = np.linspace(-math.pi, math.pi, 1441)
    global_angles = angles+pose.yaw
    # Intersections with the four outer walls; no privileged target distance.
    ranges = np.full(angles.shape, math.inf)
    for coordinate, direction, lo, hi in (
            (pose.x, np.cos(global_angles), 0., 8.),
            (pose.y, np.sin(global_angles), 0., 10.)):
        for boundary in (lo, hi):
            with np.errstate(divide='ignore', invalid='ignore'):
                distance = (boundary-coordinate)/direction
            ranges = np.minimum(ranges, np.where(distance > 0, distance, math.inf))
    scan = Scan(ranges, float(angles[0]), float(angles[1]-angles[0]), .2, 25., stamp)
    return frame, scan


def run(output):
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    cells = np.zeros((100, 80), dtype=np.int8)
    cells[[0, -1], :] = 100; cells[:, [0, -1]] = 100
    mission = Mission()
    grid = Grid(cells, .1)
    x, y, yaw = 2., 5., 0.
    dt, rows, states = .1, [], []
    first_frame = None
    for tick in range(1600):
        now = tick*dt+1.
        pose = Pose(x, y, yaw, now)
        frame, scan = scene(pose, now)
        detections = [d for d in detect_shapes(frame, now) if d.label == 'triangle']
        if first_frame is None:
            first_frame = draw_detections(frame, detections, 'triangle')
        if len(detections) == 1:
            point = fuse_detection(detections[0], scan, pose, 500., 320.)
            if point is not None:
                mission.observe(point, pose, now)
        state, (forward, turn), path = mission.step(now, pose, scan, grid)
        if not states or states[-1] != state:
            states.append(state)
        left, right = thrust_to_pwm(forward, turn)
        rows.append(dict(time=round(now, 2), x=x, y=y, yaw=yaw, state=state,
                         left_pwm=left, right_pwm=right))
        if mission.completed:
            break
        # Toy dynamics solely to exercise the pipeline; no boat calibration claim.
        yaw = wrap(yaw+turn*2.5*dt)
        x += forward*2*math.cos(yaw)*dt
        y += forward*2*math.sin(yaw)*dt
    if not mission.completed:
        raise RuntimeError('Offline mission failed to reach region: '+states[-1])
    with (output/'trajectory.csv').open('w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader(); writer.writerows(rows)
    cv2.imwrite(str(output/'detected_shape.png'), first_frame)
    canvas = np.full((600, 480, 3), 245, dtype=np.uint8)
    def pixel(point):
        return round(point[0]*60), 600-round(point[1]*60)
    cv2.rectangle(canvas, (3, 3), (477, 597), (50, 50, 50), 4)
    cv2.polylines(canvas, [np.array([pixel((r['x'], r['y'])) for r in rows], dtype=np.int32)], False, (200, 90, 20), 3)
    for point, label, color in [((2, 5), 'START', (160, 90, 0)),
                                ((5.5, 5), 'APPROACH', (20, 150, 20)),
                                ((6.8, 5), 'REGION', (20, 100, 220))]:
        p = pixel(point); cv2.circle(canvas, p, 7, color, -1)
        cv2.putText(canvas, label, (p[0]-60, p[1]-20), cv2.FONT_HERSHEY_SIMPLEX, .45, color, 1)
    cv2.putText(canvas, 'SYNTHETIC OFFLINE DEMO', (65, 40), cv2.FONT_HERSHEY_SIMPLEX, .6, (50, 50, 50), 1)
    cv2.imwrite(str(output/'route.png'), canvas)
    summary = dict(synthetic=True, reached=mission.completed, steps=len(rows), states=states,
                   final_pose=[x, y, yaw], target=mission.target.tolist())
    (output/'summary.json').write_text(json.dumps(summary, indent=2), encoding='utf-8')
    return summary


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', default='demo_output')
    args = parser.parse_args(argv)
    print(json.dumps(run(args.output), indent=2))


if __name__ == '__main__':
    main()
