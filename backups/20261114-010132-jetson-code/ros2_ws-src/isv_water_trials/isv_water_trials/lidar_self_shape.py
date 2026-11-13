"""Body-fixed near-field voxel mask; capture needs motion, never a radius cutout."""
from collections import deque
import json
import math
from pathlib import Path
import re
import numpy as np


CELL = .04
LIMIT = np.array([1.0, .65, 1.2])  # LiDAR frame, metres. No far-field masking.


def near(points):
    p = np.asarray(points)
    return np.isfinite(p).all(axis=1) & (np.abs(p) <= LIMIT).all(axis=1)


def keys(points):
    return np.floor(np.asarray(points)/CELL).astype(np.int32)


class ShapeCapture:
    def __init__(self):
        self.frames = deque(maxlen=240); self.candidates = set(); self.stable = 0
        self.frame_id = None; self.last_time = -math.inf; self.started = None
        self.ready = False; self.yaw_span = 0.; self.error = ''

    def update(self, points, frame_id, stamp, yaw):
        if stamp-self.last_time < .19: return
        if self.frame_id is not None and frame_id != self.frame_id:
            raise ValueError('라이다 좌표계가 바뀌었습니다. 형상 기록을 다시 시작하세요')
        self.frame_id = frame_id; self.last_time = stamp
        if self.started is None: self.started = stamp
        if yaw is None or not math.isfinite(yaw):
            self.ready = False; self.error = 'IMU 방향 수신 대기'; return
        self.error = ''
        points = np.asarray(points); current = {tuple(k) for k in keys(points[near(points)])}
        self.frames.append((current, float(yaw)))
        # Look at six heading groups, preventing a long static dwell from
        # outvoting a few moving scans. Retain only repeatedly seen exact cells.
        unwrapped = np.unwrap([f[1] for f in self.frames])
        self.yaw_span = math.degrees(float(np.ptp(unwrapped)))
        groups = {}
        for (cells, _), angle in zip(self.frames, unwrapped):
            group = groups.setdefault(int(math.floor(angle/math.radians(20))), [])
            group.append(cells)
        persistent = []
        for group in groups.values():
            if len(group) < 4: continue
            counts = {}
            for cells in group:
                for k in cells: counts[k] = counts.get(k, 0)+1
            persistent.append({k for k, n in counts.items() if n >= .65*len(group)})
        candidate = set.intersection(*persistent) if persistent else set()
        overlap = len(candidate & self.candidates)/max(1, len(candidate | self.candidates))
        self.stable = self.stable+1 if candidate and overlap >= .9 else 0
        self.candidates = candidate
        self.ready = bool(stamp-self.started >= 30 and self.yaw_span >= 100 and
                          len(persistent) >= 5 and self.stable >= 15 and candidate)

    def preview(self):
        return (np.array(sorted(self.candidates), dtype=float).reshape(-1,3)+.5)*CELL

    def status(self):
        return dict(ready=self.ready, frames=len(self.frames), points=len(self.candidates),
                    yaw_span_deg=round(self.yaw_span,1), stable_frames=self.stable,
                    elapsed_s=max(0,self.last_time-self.started) if self.started is not None else 0,
                    error=self.error)


class ShapeMask:
    def __init__(self, data):
        if data.get('schema') != 'lidar-self-shape-v1' or data.get('cell_m') != CELL:
            raise ValueError('배 형상 파일 형식 오류')
        points = np.asarray(data['points'], dtype=float).reshape(-1,3)
        if not 1 <= len(points) <= 10000 or not near(points).all():
            raise ValueError('배 형상은 라이다 주변의 작은 고정 영역만 저장할 수 있습니다')
        self.points=points; self.frame_id=data['frame_id']; self.name=data['name']; self.data=data
        # Exact voxels only: no full bearing or broad range suppression.
        self.cells = {tuple(k) for k in keys(points)}

    def keep(self, points, frame_id):
        if frame_id != self.frame_id: raise ValueError('저장 형상과 라이다 좌표계가 다릅니다')
        result=np.ones(len(points),dtype=bool); ids=np.flatnonzero(near(points))
        result[ids]=[tuple(k) not in self.cells for k in keys(np.asarray(points)[ids])]
        return result


def save_shape(root, name, capture):
    if not capture.ready: raise ValueError('회전 관측과 형상 수렴을 기다려 주세요')
    if not isinstance(name,str) or not re.fullmatch(r'[\w가-힣 -]{1,48}',name):
        raise ValueError('형상 이름은 글자·숫자·공백·밑줄·하이픈으로 입력하세요')
    root=Path(root);root.mkdir(parents=True,exist_ok=True)
    data=dict(schema='lidar-self-shape-v1',name=name,frame_id=capture.frame_id,cell_m=CELL,
              points=capture.preview().tolist(),capture=capture.status())
    mask=ShapeMask(data)
    for path, value in ((root/(name+'.json'),data),(root/'active.json',{'file':name+'.json'})):
        temp=path.with_suffix('.incoming');temp.write_text(json.dumps(value,ensure_ascii=False),encoding='utf-8');temp.replace(path)
    return mask


def cloud_arrays(msg):
    """Preserve all fields, ring, intensity, time, endian and row padding."""
    fields={f.name:f for f in msg.fields}; endian='>' if msg.is_bigendian else '<'
    formats={7:'f4',8:'f8'}
    if not all(k in fields and fields[k].datatype in formats and fields[k].count==1 for k in ('x','y','z')):
        raise ValueError('라이다 XYZ 형식 오류')
    if msg.height*msg.width > 2_000_000 or msg.row_step < msg.width*msg.point_step or len(msg.data)<msg.height*msg.row_step:
        raise ValueError('라이다 데이터 크기 오류')
    dtype=np.dtype(dict(names=['x','y','z'],formats=[endian+formats[fields[k].datatype] for k in ('x','y','z')],
                        offsets=[fields[k].offset for k in ('x','y','z')],itemsize=msg.point_step))
    view=np.ndarray((msg.height,msg.width),dtype=dtype,buffer=memoryview(msg.data),strides=(msg.row_step,msg.point_step))
    xyz=np.column_stack([view[k].reshape(-1) for k in ('x','y','z')])
    records=np.ndarray((msg.height,msg.width,msg.point_step),dtype=np.uint8,buffer=memoryview(msg.data),
                       strides=(msg.row_step,msg.point_step,1)).reshape(-1,msg.point_step)
    return xyz,records
