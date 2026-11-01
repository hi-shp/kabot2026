"""Jetson-owned sessions: survive SSH disconnect, stop explicitly with Ctrl+C."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
from .tank_storage import atomic_json

ROOT = Path.home()/'water_trials/runtime'
NAMES = ('trial','record','mapping','sensors')


def identity(pid):
    try: return Path(f'/proc/{pid}/stat').read_text().rsplit(')',1)[1].split()[19]
    except (FileNotFoundError,ProcessLookupError): return None


def running(name):
    try: value = json.loads((ROOT/(name+'.json')).read_text())
    except (OSError,ValueError): return None
    return value if identity(value['pid'])==value['start_ticks'] else None


def command_for(name, extra):
    if name=='record': return [sys.executable,'-m','isv_water_trials.tank_record',*extra]
    if name=='sensors': return ['ros2','launch','mechaship_bringup','all_sensors.launch.py',*extra]
    if name=='mapping': return ['ros2','launch','isv_water_trials','tank_mapping.launch.py',*extra]
    return [sys.executable,'-m','isv_water_trials.indoor_trial',*extra]


def worker(name, extra):
    import fcntl
    lock = (ROOT/(name+'.lock')).open('w')
    try: fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    except BlockingIOError: return
    stopped = False; child = None
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
        while not stopped:
            child = subprocess.Popen(command,stdin=subprocess.DEVNULL,start_new_session=True)
            while child.poll() is None: time.sleep(.1)
            code = child.returncode
            # An orderly exit, bad CLI, missing route or duplicate launch needs
            # operator correction. Only an unexpected runtime exit is retried.
            if stopped or code in (0,2): break
            print(f'{name}: exit {code}; retry in 5s',flush=True)
            until = time.monotonic()+5
            while not stopped and time.monotonic()<until: time.sleep(.1)
    finally:
        if child and child.poll() is None:
            os.killpg(child.pid,signal.SIGINT)
            child.wait(timeout=15)
        marker = ROOT/(name+'.json')
        if running(name) and running(name)['pid']==os.getpid(): marker.unlink(missing_ok=True)


def main(argv=None):
    argv=list(sys.argv[1:] if argv is None else argv)
    if argv and argv[0]=='freeze' and (len(argv)==1 or argv[1]!='mapping'):
        argv.insert(1,'mapping')
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action',choices=('start','status','attach','stop','freeze','_worker'))
    parser.add_argument('name',choices=(*NAMES,'all'),nargs='?',default='all')
    args,extra = parser.parse_known_args(argv)
    if extra and extra[0]=='--': extra = extra[1:]
    ROOT.mkdir(parents=True,exist_ok=True)
    if args.action=='_worker': return worker(args.name,extra)
    names = NAMES if args.name=='all' else (args.name,)
    if args.action=='freeze':
        freeze_parser=argparse.ArgumentParser()
        freeze_parser.add_argument('--name',default='~/water_trials/tank_maps/pool.tankmap')
        config=freeze_parser.parse_args(extra)
        if running('trial'):main(['stop','trial'])
        if running('trial'):raise RuntimeError('왕복 종료를 먼저 확인하세요.')
        result=subprocess.run([sys.executable,'-m','isv_water_trials.tank_save','--freeze','--name',config.name])
        if result.returncode:raise SystemExit(result.returncode)
        main(['stop','mapping'])
        if running('mapping'):raise RuntimeError('지도 프로세스 종료 대기 중입니다.')
        main(['start','mapping'])
    elif args.action=='start':
        if args.name=='all': parser.error('Start sensors, mapping, record and trial individually')
        if running(args.name): print(args.name+' already running'); return
        if args.name=='trial' and '--execute' in extra:
            main(['start','record'])
        with (ROOT/(args.name+'.log')).open('a',encoding='utf-8') as stream:
            subprocess.Popen([sys.executable,'-m','isv_water_trials.tank_session','_worker',args.name,'--',*extra],
                start_new_session=True,stdin=subprocess.DEVNULL,stdout=stream,stderr=subprocess.STDOUT,close_fds=True)
        time.sleep(.5)
        print(args.name+(' started' if running(args.name) else ' not started; inspect '+str(ROOT/(args.name+'.log'))))
    elif args.action=='status':
        for name in names: print(name+': '+('running' if running(name) else 'stopped'))
    elif args.action=='stop':
        for name in names:
            value = running(name)
            if value:
                try: os.kill(value['pid'],signal.SIGINT)
                except ProcessLookupError: pass
                until = time.monotonic()+15
                while running(name) and time.monotonic()<until: time.sleep(.1)
            print(name+': '+('STOP_PENDING: inspect session log' if running(name) else 'stopped'))
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
