"""Edit existing GPS literals only, preserving all other configuration text."""
import ast
import hashlib
import math
import os
from pathlib import Path
import shutil
import tempfile
import threading
import time


def parse_slots(raw):
    tree = ast.parse(raw.decode('utf-8'))
    assignments = [n for n in tree.body if isinstance(n, ast.Assign)
                   and any(isinstance(t, ast.Name) and t.id == 'STATUS_CONFIG' for t in n.targets)]
    if len(assignments) != 1 or not isinstance(assignments[0].value, ast.Dict):
        raise ValueError('STATUS_CONFIG must be a literal dictionary')
    root = assignments[0].value
    stages = ast.literal_eval(root)
    slots, nodes = [], {}

    def fields(node):
        if not isinstance(node, ast.Dict):
            raise ValueError('Stage configuration must be literal')
        pairs = [(ast.literal_eval(k), v) for k, v in zip(node.keys, node.values)]
        if len(dict(pairs)) != len(pairs):
            raise ValueError('Duplicate configuration keys')
        return pairs

    def add(stage, path, label, node):
        pair = ast.literal_eval(node)
        if not isinstance(pair, (tuple, list)) or len(pair) != 2:
            raise ValueError('GPS must be a pair')
        ident = '/'.join(map(str, [stage, *path]))
        slots.append(dict(id=ident, label=label, gps=list(pair)))
        nodes[ident] = node

    for stage, node in fields(root):
        values = dict(fields(node))
        if stage in (0, 2, 3, 6, 7) and 'gps' in values:
            label = {0:'0 경로 시작점',2:'2 호핑 목표',3:'3 도킹 시작점',6:'6 회피 시작점',7:'7 회피 최종 목표'}[stage]
            add(stage, ['gps'], label, values['gps'])
        if stage in (1, 7):
            key = 'gps_points' if stage == 1 else 'intermediate_gps'
            seq = values.get(key)
            if seq is not None:
                if not isinstance(seq, (ast.List, ast.Tuple)):
                    raise ValueError('GPS sequence must be literal')
                for index, item in enumerate(seq.elts):
                    add(stage, [key, index], f'{stage} '+('경로' if stage == 1 else '회피 중간')+f' {index+1}', item)
        if stage in (4, 5):
            key = 'gps_by_shape' if stage == 4 else 'gps_by_buoy'
            for name, item in fields(values[key]):
                add(stage, [key, name], f'{stage} '+('도킹' if stage == 4 else '부표 중심')+' · '+name, item)
    return stages, slots, nodes


class GPSConfigEditor:
    def __init__(self, path):
        self.path = Path(path).resolve()
        self.lock = threading.RLock()

    def snapshot(self):
        with self.lock:
            raw = self.path.read_bytes()
            _, slots, _ = parse_slots(raw)
            return dict(slots=slots, revision=hashlib.sha256(raw).hexdigest(), applies='next_mission_start')

    def save(self, request):
        if not isinstance(request, dict) or set(request) != {'id', 'gps', 'revision'}:
            raise ValueError('Expected id, gps, revision')
        gps = request['gps']
        if (not isinstance(gps, list) or len(gps) != 2
                or not all(type(v) in (int, float) and math.isfinite(v) for v in gps)
                or abs(gps[0]) > 85.05112878 or abs(gps[1]) > 180):
            raise ValueError('Invalid GPS coordinates')
        with self.lock:
            raw = self.path.read_bytes()
            if request['revision'] != hashlib.sha256(raw).hexdigest():
                raise FileExistsError('Configuration changed')
            _, _, nodes = parse_slots(raw)
            if not isinstance(request['id'], str) or request['id'] not in nodes:
                raise ValueError('Unknown mission GPS slot')
            node = nodes[request['id']]
            lines = raw.splitlines(keepends=True)
            start = sum(map(len, lines[:node.lineno-1])) + node.col_offset
            end = sum(map(len, lines[:node.end_lineno-1])) + node.end_col_offset
            # repr(float) round-trips the click coordinate without silently rounding it.
            replacement = repr(tuple(float(v) for v in gps)).encode('ascii')
            edited = raw[:start] + replacement + raw[end:]
            stages, _, _ = parse_slots(edited)
            backup_dir = self.path.parent/'backups'/'gui_gps'
            backup_dir.mkdir(parents=True, exist_ok=True)
            backup = backup_dir/f'{self.path.stem}_{time.time_ns()}.py'
            with backup.open('xb') as f:
                f.write(raw)
            temp = None
            try:
                with tempfile.NamedTemporaryFile(dir=self.path.parent, delete=False) as f:
                    temp = f.name
                    f.write(edited)
                    f.flush()
                    os.fsync(f.fileno())
                shutil.copymode(self.path, temp)
                if self.path.read_bytes() != raw:
                    raise FileExistsError('Configuration changed')
                os.replace(temp, self.path)
                temp = None
            finally:
                if temp is not None:
                    os.unlink(temp)
            return dict(ok=True, stages=stages, revision=hashlib.sha256(edited).hexdigest(),
                        applies='next_mission_start')
