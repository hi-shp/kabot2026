"""Durable segmented logs and atomic checkpoints; no automatic data deletion."""
import json
import os
from pathlib import Path
import time


def atomic_json(path, value):
    path = Path(path); path.parent.mkdir(parents=True,exist_ok=True)
    temporary = path.with_suffix(path.suffix+'.incoming')
    with temporary.open('w',encoding='utf-8',newline='\n') as stream:
        json.dump(value,stream,ensure_ascii=False,allow_nan=False,indent=2)
        stream.flush(); os.fsync(stream.fileno())
    os.replace(temporary,path)
    if os.name=='posix':
        fd = os.open(str(path.parent),os.O_RDONLY)
        try: os.fsync(fd)
        finally: os.close(fd)


class Journal:
    def __init__(self, root, sync_seconds=2., segment_seconds=60.):
        self.root = Path(root); self.root.mkdir(parents=True,exist_ok=True)
        self.sync_seconds = sync_seconds; self.segment_seconds = segment_seconds
        self.stream = None; self.index = 0; self.last_sync = 0.; self.opened = 0.; self.count = 0

    def write(self, row):
        now = time.monotonic()
        if self.stream is None or now-self.opened>=self.segment_seconds:
            self.close(); self.index += 1
            self.stream = (self.root/f'samples-{self.index:06d}.jsonl').open('x',encoding='utf-8',newline='\n')
            self.opened = now
        self.stream.write(json.dumps(row,ensure_ascii=False,allow_nan=False)+'\n'); self.count += 1
        if now-self.last_sync>=self.sync_seconds:
            self.stream.flush(); os.fsync(self.stream.fileno()); self.last_sync = now
            atomic_json(self.root/'checkpoint.json', {'rows':self.count,'segment':self.index,
                'last_t':row.get('t'),'goal':row.get('goal'),'legs':row.get('legs'),
                'state':row.get('state'),'saved_unix':time.time()})

    def close(self):
        if self.stream:
            try: self.stream.flush(); os.fsync(self.stream.fileno())
            finally: self.stream.close(); self.stream = None


def read_rows(root):
    """Ignore only a power-cut partial LAST line; surface other corruption."""
    root = Path(root)
    files = sorted(root.glob('samples-*.jsonl'))
    if (root/'samples.jsonl').exists(): files.insert(0,root/'samples.jsonl')
    for path in files:
        with path.open('rb') as stream:
            while True:
                line = stream.readline()
                if not line: break
                if not line.endswith(b'\n'): break
                yield json.loads(line)
