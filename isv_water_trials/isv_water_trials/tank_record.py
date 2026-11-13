"""Jetson-owned compressed sensor recording, independent of SSH and learning."""
import argparse
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import time
import uuid
from .tank_storage import atomic_json
from .shutdown import keep_terminal_loss

ROOT=Path.home()/'water_trials/bags'
TOPICS=['/scan','/gps/fix','/gps/fix_velocity','/vectornav/imu','/vectornav/raw/common',
        '/tank/odom','/tank/pose','/tank/session','/actuators/status','/outdoor/command',
        '/water_trial/command','/tf','/tf_static']
TOPICS+=['/tank/localization_candidates','/tank/status','/outdoor/docking_observation']


def recording_status(root=None):
    root=Path(root or ROOT)
    try:state=json.loads((root/'recording_state.json').read_text())
    except (OSError,ValueError):state={}
    if time.time()-state.get('updated_unix_s',0)>10:state=dict(state,state='stopped')
    return state


def recordings(root=None,limit=20):
    root=Path(root or ROOT);state=recording_status(root)
    sessions=[]
    if root.exists():
        for folder in sorted((p for p in root.glob('tank_*') if p.is_dir()),reverse=True)[:limit]:
            files=[p for p in folder.iterdir() if p.is_file()]
            try:size=sum(p.stat().st_size for p in files)
            except FileNotFoundError:continue
            sessions.append(dict(name=folder.name,path=str(folder),bytes=size,files=len(files)))
    return dict(state=state,sessions=sessions,root=str(root),index=str(root/'sessions.jsonl'))


def main(argv=None):
    import fcntl
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root',type=Path,default=ROOT)
    parser.add_argument('--reserve-mb',type=int,default=256)
    parser.add_argument('--include-cloud',action='store_true')
    args=parser.parse_args(argv);root=args.root.expanduser();root.mkdir(parents=True,exist_ok=True)
    keep_terminal_loss();stopped=False;child=None;log=None;output=None
    def stop(*_):
        nonlocal stopped
        stopped=True
    signal.signal(signal.SIGINT,stop);signal.signal(signal.SIGTERM,stop)
    lock=open('/tmp/kabot_sensor_record_'+os.environ.get('ROS_DOMAIN_ID','0')+'.lock','w')
    acquired=False
    while not stopped:
        try:fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB);acquired=True;break
        except BlockingIOError:time.sleep(.5)
    if not acquired:lock.close();return
    topics=TOPICS+(['/velodyne_points'] if args.include_cloud else [])
    reserve=max(0,args.reserve_mb)*1024**2
    state=dict(topics=topics,domain=os.environ.get('ROS_DOMAIN_ID','0'),owner_pid=os.getpid(),root=str(root))
    def event(kind,**extra):
        with (root/'sessions.jsonl').open('a',encoding='utf-8') as stream:
            stream.write(json.dumps(dict(event=kind,unix_s=time.time(),path=str(output) if output else None,**extra))+'\n')
            stream.flush();os.fsync(stream.fileno())
    def halt():
        nonlocal child,log
        if child is not None and child.poll() is None:
            os.killpg(child.pid,signal.SIGINT)
            try:child.wait(timeout=25)
            except subprocess.TimeoutExpired:
                os.killpg(child.pid,signal.SIGTERM);child.wait(timeout=5)
        if child is not None:event('closed',returncode=child.returncode)
        child=None
        if log is not None:log.close();log=None
    try:
        while not stopped:
            free=shutil.disk_usage(root).free
            if child is not None and child.poll() is not None:halt()
            if child is not None and free<reserve:halt()
            if child is None and free>reserve+64*1024**2:
                name='tank_'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')+'_'+uuid.uuid4().hex[:8]
                output=root/name
                log=(root/(name+'.log')).open('a',encoding='utf-8')
                command=['ros2','bag','record','-o',str(output),'--max-bag-duration','60',
                    '--max-bag-size','67108864','--storage-preset-profile','resilient',
                    '--compression-mode','file','--compression-format','zstd',*topics]
                child=subprocess.Popen(command,stdin=subprocess.DEVNULL,stdout=log,stderr=subprocess.STDOUT,
                                       start_new_session=True,close_fds=True)
                event('started',pid=child.pid,topics=topics)
            state.update(state='recording' if child is not None else 'disk_wait',
                         updated_unix_s=time.time(),free_bytes=free,current_bag=str(output) if output else None)
            try:atomic_json(root/'recording_state.json',state)
            except OSError:pass
            time.sleep(1.)
    finally:
        try:
            halt();state.update(state='stopped',updated_unix_s=time.time())
            try:atomic_json(root/'recording_state.json',state)
            except OSError:pass
        finally:lock.close()


if __name__=='__main__':main()
