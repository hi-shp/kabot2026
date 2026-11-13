"""Jetson-owned sessions: survive SSH disconnect, stop explicitly with Ctrl+C."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import uuid
from .tank_storage import atomic_json

ROOT = Path.home()/'water_trials/runtime'
NAMES = ('trial','record','mapping','outdoor','sensors')


def wake_dashboard():
    atomic_json(ROOT/'dashboard_wake.json',{'id':uuid.uuid4().hex})


def identity(pid):
    try: return Path(f'/proc/{pid}/stat').read_text().rsplit(')',1)[1].split()[19]
    except (FileNotFoundError,ProcessLookupError): return None


def running(name):
    try: value = json.loads((ROOT/(name+'.json')).read_text())
    except (OSError,ValueError): return None
    return value if identity(value['pid'])==value['start_ticks'] else None


def command_for(name, extra):
    if name=='record': return [sys.executable,'-m','isv_water_trials.tank_record',*extra]
    if name=='outdoor': return [sys.executable,'-m','isv_water_trials.outdoor',*extra]
    if name=='sensors': return ['ros2','launch','mechaship_bringup','all_sensors.launch.py',*extra]
    if name=='mapping': return ['ros2','launch','isv_water_trials','tank_mapping.launch.py',*extra]
    return [sys.executable,'-m','isv_water_trials.indoor_trial',*extra]


def worker(name, extra):
    import fcntl
    lock = (ROOT/(name+'.lock')).open('w')
    try: fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    except BlockingIOError: lock.close(); return
    stopped = False; child = None
    sensor_owner = running('sensors') if name=='outdoor' else None
    if name=='outdoor' and not sensor_owner:
        lock.close(); return
    def stop(*_):
        nonlocal stopped
        stopped = True
        if child and child.poll() is None:
            try: os.killpg(child.pid,signal.SIGINT)
            except ProcessLookupError: pass
    signal.signal(signal.SIGHUP,signal.SIG_IGN)
    signal.signal(signal.SIGINT,stop); signal.signal(signal.SIGTERM,stop)
    atomic_json(ROOT/(name+'.json'),{'pid':os.getpid(),'start_ticks':identity(os.getpid()),'args':extra})
    command = command_for(name,extra)
    try:
        if name=='mapping':main(['start','record'],_internal=True)
        while not stopped:
            child = subprocess.Popen(command,stdin=subprocess.DEVNULL,start_new_session=True)
            while child.poll() is None:
                # A supplemental navigation worker belongs to this sensor session.
                if name=='outdoor' and running('sensors')!=sensor_owner:stop()
                time.sleep(.1)
            code = child.returncode
            # An orderly exit, bad CLI, missing route or duplicate launch needs
            # operator correction. Only an unexpected runtime exit is retried.
            if stopped or code in (0,2): break
            print(f'{name}: exit {code}; retry in 5s',flush=True)
            until = time.monotonic()+5
            while not stopped and time.monotonic()<until: time.sleep(.1)
    finally:
        try:
            if child and child.poll() is None:
                os.killpg(child.pid,signal.SIGINT)
                child.wait(timeout=30 if name=='record' else 15)
        finally:
            marker = ROOT/(name+'.json')
            if running(name) and running(name)['pid']==os.getpid(): marker.unlink(missing_ok=True)
            lock.close()


def main(argv=None,*,_internal=False):
    argv=list(sys.argv[1:] if argv is None else argv)
    if argv and argv[0]=='freeze' and (len(argv)==1 or argv[1]!='mapping'):
        argv.insert(1,'mapping')
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action',choices=('start','status','attach','stop','freeze','_worker'))
    choices=NAMES if _internal or argv[:1]==['_worker'] else tuple(n for n in NAMES if n not in ('record','outdoor'))
    parser.add_argument('name',choices=(*choices,'all'),nargs='?',default='all')
    args,extra = parser.parse_known_args(argv)
    if extra and extra[0]=='--': extra = extra[1:]
    ROOT.mkdir(parents=True,exist_ok=True)
    if args.action=='_worker': return worker(args.name,extra)
    names = NAMES if args.name=='all' else (args.name,)
    if args.action=='stop' and args.name=='all':names=tuple(n for n in names if n!='record')
    if args.action=='freeze':
        freeze_parser=argparse.ArgumentParser()
        freeze_parser.add_argument('--name',default='~/water_trials/tank_maps/pool.tankmap')
        freeze_parser.add_argument('--crop',nargs=4,type=float)
        config=freeze_parser.parse_args(extra)
        if running('trial'):main(['stop','trial'])
        if running('trial'):raise RuntimeError('왕복 종료를 먼저 확인하세요.')
        save_command=[sys.executable,'-m','isv_water_trials.tank_save','--freeze','--name',config.name]
        if config.crop:save_command+=['--crop',*map(str,config.crop)]
        result=subprocess.run(save_command)
        if result.returncode:raise SystemExit(result.returncode)
        main(['stop','mapping'])
        if running('mapping'):raise RuntimeError('지도 프로세스 종료 대기 중입니다.')
        main(['start','mapping'])
    elif args.action=='start':
        if args.name=='all': parser.error('Choose sensors, mapping or trial')
        if running(args.name):
            if args.name=='mapping' and not running('record'):main(['start','record'],_internal=True)
            if args.name=='sensors':wake_dashboard()
            print(args.name+' already running'); return
        with (ROOT/(args.name+'.log')).open('a',encoding='utf-8') as stream:
            subprocess.Popen([sys.executable,'-m','isv_water_trials.tank_session','_worker',args.name,'--',*extra],
                start_new_session=True,stdin=subprocess.DEVNULL,stdout=stream,stderr=subprocess.STDOUT,close_fds=True)
        time.sleep(.5)
        if args.name=='sensors' and running('sensors'):wake_dashboard()
        print(args.name+(' started' if running(args.name) else ' not started; inspect '+str(ROOT/(args.name+'.log'))))
    elif args.action=='status':
        for name in names: print(name+': '+('running' if running(name) else 'stopped'))
    elif args.action=='stop':
        if args.name=='sensors':main(['stop','outdoor'],_internal=True)
        for name in names:
            value = running(name)
            if value:
                try: os.kill(value['pid'],signal.SIGINT)
                except ProcessLookupError: pass
                until = time.monotonic()+({'mapping':50,'record':35}.get(name,15))
                while running(name) and time.monotonic()<until: time.sleep(.1)
            if not _internal:print(name+': '+('STOP_PENDING: inspect session log' if running(name) else 'stopped'))
    elif args.action=='attach':
        if args.name=='all': parser.error('Choose one session to attach')
        print('Ctrl+C: stop this session. SSH loss: keep it running.',flush=True)
        # HUP terminates this reader without forwarding any signal to the worker.
        try:
            with (ROOT/(args.name+'.log')).open(encoding='utf-8',errors='replace') as stream:
                stream.seek(max(0,os.path.getsize(stream.name)-3000))
                stream.readline()
                while running(args.name):
                    line = stream.readline()
                    if line: print(line,end='',flush=True)
                    else: time.sleep(.2)
        except KeyboardInterrupt:
            main(['stop',args.name])


if __name__=='__main__': main()
