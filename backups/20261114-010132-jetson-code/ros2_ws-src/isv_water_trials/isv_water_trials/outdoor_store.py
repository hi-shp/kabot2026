"""Durable RC history across pause, sensor restart and Jetson reboot."""
import json
from pathlib import Path
import sqlite3
import time
import uuid
import random
from datetime import datetime, timezone, timedelta
from .outdoor_model import fingerprint
from .outdoor_config import compatible_signatures, learning_identity
from .tank_storage import atomic_json


class OutdoorStore:
    def __init__(self, root, config):
        self.root = Path(root); self.root.mkdir(parents=True, exist_ok=True)
        self.signature = fingerprint(config); self.segment = uuid.uuid4().hex
        self.signatures=compatible_signatures(self.root,config)
        self.config=config
        self._catalog_at=-float('inf');self._catalog=[];self._model_meta={}
        self._summary_at=-float('inf');self._summary={};self._display_names={}
        self.db = sqlite3.connect(self.root/'observations.sqlite3')
        self.db.execute('PRAGMA journal_mode=WAL'); self.db.execute('PRAGMA synchronous=FULL')
        self.db.execute('CREATE TABLE IF NOT EXISTS frames (id INTEGER PRIMARY KEY, signature TEXT, segment TEXT, stamp REAL, payload TEXT)')
        self.db.execute('CREATE INDEX IF NOT EXISTS frames_signature ON frames(signature,id)')
        if 'branch_id' not in {r[1] for r in self.db.execute('PRAGMA table_info(frames)')}:
            self.db.execute('ALTER TABLE frames ADD COLUMN branch_id TEXT')
        self.db.execute('CREATE INDEX IF NOT EXISTS frames_branch ON frames(branch_id,id)')
        self.db.execute('CREATE TABLE IF NOT EXISTS events (stamp REAL, payload TEXT)')
        self.db.execute('CREATE TABLE IF NOT EXISTS positions (id INTEGER PRIMARY KEY, signature TEXT, segment TEXT, stamp REAL, lat REAL, lon REAL)')
        self.last_commit = time.monotonic()
        atomic_json(self.root/'configs'/(self.signature+'.json'), config)

    def event(self, action, **kwargs):
        self.db.execute('INSERT INTO events VALUES (?,?)', (time.time(), json.dumps(dict(action=action, **kwargs), allow_nan=False)))
        self.commit()

    def new_segment(self, reason):
        self.segment = uuid.uuid4().hex; self.event('segment', reason=reason, segment=self.segment)

    def add(self, row):
        if row.get('synthetic'):
            raise ValueError('가상 기록은 실선 학습 저장소에 넣을 수 없습니다')
        row = dict(row, segment=self.segment)
        self.db.execute('INSERT INTO frames(signature,segment,stamp,payload,branch_id) VALUES (?,?,?,?,?)',
                        (self.signature, self.segment, row['t'], json.dumps(row, allow_nan=False),row.get('branch_id')))
        if time.monotonic()-self.last_commit >= 1: self.commit()
        return row

    def rows(self, limit=30000):
        marks=','.join('?' for _ in self.signatures)
        values = self.db.execute(f'SELECT payload FROM frames WHERE signature IN ({marks}) ORDER BY id DESC LIMIT ?', (*self.signatures, limit)).fetchall()
        return [json.loads(row[0]) for row in reversed(values)]

    def recording_summary(self):
        if time.monotonic()-self._summary_at>=3.:
            marks=','.join('?' for _ in self.signatures)
            samples,segments=self.db.execute(f'SELECT COUNT(*),COUNT(DISTINCT segment) FROM frames WHERE signature IN ({marks})',self.signatures).fetchone()
            self._summary={'samples':samples,'segments':segments}
            self._summary_at=time.monotonic()
        return dict(self._summary)

    def model_display(self, model):
        """Resolve old percentage labels without rewriting saved models."""
        if model is None:return None
        identity=model.get('id');name=self._display_names.get(identity)
        if name is None:
            ancestor=model;seen=set()
            for _ in range(100):
                name=ancestor.get('candidate_name') or ancestor.get('variant')
                if name and name not in ('기준 보존','기존 모델'):break
                parent=ancestor.get('parent_model_id')
                if not parent or parent in seen:name='기존 모델';break
                seen.add(parent)
                try:ancestor=json.loads(self.model_path(parent).read_text(encoding='utf-8'))
                except (ValueError,OSError):name='기존 모델';break
            if not name or name=='기준 보존':name='기존 모델'
            self._display_names[identity]=name
        return dict(model,display_name=name)

    def training_rows(self, lineage=None):
        if not lineage:
            # Keep recent manoeuvres dense and revisit bounded continuous windows
            # from the archive. The original database is never trimmed.
            marks=','.join('?' for _ in self.signatures)
            recent=self.db.execute(f'SELECT id,payload FROM frames WHERE signature IN ({marks}) ORDER BY id DESC LIMIT 15000',self.signatures).fetchall()
            if len(recent)<15000:return [json.loads(r[1]) for r in reversed(recent)]
            first=self.db.execute(f'SELECT MIN(id) FROM frames WHERE signature IN ({marks})',self.signatures).fetchone()[0]
            boundary=recent[-1][0]; selected=dict(recent)
            rng=random.SystemRandom()
            for i in range(50):
                low=first+(boundary-first)*i//50
                high=first+(boundary-first)*(i+1)//50
                anchor=first if i==0 else rng.randrange(low,max(low+1,high))
                block=self.db.execute(f'SELECT id,payload FROM frames WHERE signature IN ({marks}) AND id>=? AND id<? ORDER BY id LIMIT 300',(*self.signatures,anchor,boundary)).fetchall()
                selected.update(block)
            return [json.loads(selected[k]) for k in sorted(selected)]
        marks=','.join('?' for _ in self.signatures)
        old=self.db.execute(f'SELECT payload FROM frames WHERE signature IN ({marks}) AND stamp<=? ORDER BY id DESC LIMIT 15000',
                            (*self.signatures,lineage['data_cutoff_t'])).fetchall()
        new=self.db.execute(f'SELECT payload FROM frames WHERE signature IN ({marks}) AND branch_id=? AND stamp>? ORDER BY id DESC LIMIT 15000',
                            (*self.signatures,lineage['id'],lineage['started_t'])).fetchall()
        return [json.loads(r[0]) for r in list(reversed(old))+list(reversed(new))]

    def live_selection(self):
        try:saved=json.loads((self.root/'selection.json').read_text(encoding='utf-8'))
        except FileNotFoundError:
            catalog=[m for m in self.model_catalog() if m['compatible']]
            current=self.load_compatible_model(catalog[0]['id']) if catalog else None
            chosen=next((m for m in catalog if m['finalized']),None)
            validation=self.load_compatible_model(chosen['id']) if chosen else None
            return current,validation,False
        current_id=saved.get('current_model_id',saved.get('model_id'))
        validation_id=saved.get('validation_model_id')
        if saved.get('schema')!='live-models-v2' and saved.get('frozen'):validation_id=current_id
        current=self.load_compatible_model(current_id) if current_id else None
        validation=self.load_compatible_model(validation_id) if validation_id else None
        # Every bringup waits for an explicit learning start; models still restore.
        return current,validation,False

    def save_live_selection(self,current,validation,learning):
        atomic_json(self.root/'selection.json',dict(schema='live-models-v2',
            current_model_id=current['id'] if current else None,
            validation_model_id=validation['id'] if validation else None,learning=learning))

    def fork_learning(self, model, stamp):
        self.save_model(model)
        branch={'id':uuid.uuid4().hex, 'parent_model_id':model['id'],
                'started_t':stamp,'data_cutoff_t':float(model.get('training_end_t',stamp)),
                'created_unix':time.time()}
        atomic_json(self.root/'branches'/(branch['id']+'.json'),branch)
        self.event('learning_branch',**branch)
        return branch

    def load_compatible_model(self, identity):
        model=json.loads(self.model_path(identity).read_text(encoding='utf-8'))
        if model.get('synthetic'):
            raise ValueError('가상 시험 모델은 실선 주행에 사용할 수 없습니다')
        if model.get('signature') not in self.signatures:
            raise ValueError('현재 센서·선체 학습 설정과 다른 모델입니다')
        if model.get('learning_settings') is not None and model['learning_settings']!=learning_identity(self.config):
            raise ValueError('모델의 학습 설정이 일치하지 않습니다')
        return model

    def position(self, stamp, lat, lon):
        self.db.execute('INSERT INTO positions(signature,segment,stamp,lat,lon) VALUES (?,?,?,?,?)',
                        (self.signature, self.segment, stamp, lat, lon))
        if time.monotonic()-self.last_commit >= 1: self.commit()

    def trail(self, after=0., limit=20000):
        marks=','.join('?' for _ in self.signatures)
        rows = self.db.execute(f'SELECT lat,lon,segment,stamp FROM positions WHERE signature IN ({marks}) AND stamp>=? ORDER BY id DESC LIMIT ?',
                               (*self.signatures, after, limit)).fetchall()
        return list(reversed(rows))

    def save_model(self, model):
        self._catalog_at=-float('inf')
        existing = list((self.root/'models').rglob(model['id']+'.json'))
        day = datetime.now(timezone(timedelta(hours=9))).strftime('%Y-%m-%d')
        path = existing[0] if existing else self.root/'models'/day/(model['id']+'.json')
        if path.exists():
            if json.loads(path.read_text(encoding='utf-8')) != model: raise ValueError('저장된 모델 ID 충돌')
        else: atomic_json(path, model)
        self.event('model_saved', id=model['id']); return path

    def finalize_model(self, model):
        path = self.save_model(model)
        atomic_json(self.root/'finalized'/(model['id']+'.json'),
                    {'model': str(path.relative_to(self.root)), 'finalized_unix': time.time()})
        self.event('model_finalized', id=model['id'])

    def model_path(self, identity):
        files = list((self.root/'models').rglob(identity+'.json'))
        if len(files) != 1: raise ValueError('저장 모델을 찾을 수 없거나 ID가 중복되었습니다')
        return files[0]

    def model_catalog(self):
        if time.monotonic()-self._catalog_at<2.: return self._catalog
        result = []
        for path in (self.root/'models').rglob('*.json'):
            try:
                stat=path.stat();cached=self._model_meta.get(path)
                if cached and cached[0]==stat.st_mtime_ns:model=cached[1]
                else:
                    model=json.loads(path.read_text(encoding='utf-8'))
                    self._model_meta[path]=(stat.st_mtime_ns,model)
            except (ValueError,OSError): continue
            stamp = stat.st_mtime
            label = self.model_display(model)['display_name']
            finalized = (self.root/'finalized'/(path.stem+'.json')).exists()
            branch=(model.get('lineage') or {}).get('id')
            result.append({'id': path.stem, 'label': label, 'saved_unix': stamp,
                           'name':label,
                           'parent_model_id':model.get('parent_model_id'),'branch_id':branch,
                           'compatible':model.get('signature') in self.signatures,
                           'finalized': finalized, 'path': str(path.relative_to(self.root))})
        self._catalog=sorted(result, key=lambda r:r['saved_unix'], reverse=True)
        self._catalog_at=time.monotonic()
        return self._catalog

    def commit(self):
        self.db.commit(); self.last_commit = time.monotonic()

    def close(self):
        self.commit(); self.db.close()
