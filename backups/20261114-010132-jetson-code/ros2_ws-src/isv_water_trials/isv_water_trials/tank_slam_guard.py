"""Restart only the SLAM engine after a crash, retaining complete checkpoints."""
import argparse
import os
import signal
import subprocess
import time
from .tank_checkpoint import latest_snapshot


def resumed_command(command,mode,checkpoint):
    result=list(command)
    if mode=='mapping' and checkpoint:
        result+=['--ros-args','-p','map_file_name:='+checkpoint['prefix'],
                 '-p','map_start_at_dock:=false']
    return result


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--mode',choices=('mapping','localization'),required=True)
    args,command=parser.parse_known_args(argv)
    if command[:1]==['--']:command=command[1:]
    if not command:parser.error('Missing SLAM executable')
    stopped=False;child=None
    def stop(*_):
        nonlocal stopped
        stopped=True
        if child and child.poll() is None:
            try:os.killpg(child.pid,signal.SIGINT)
            except ProcessLookupError:pass
    signal.signal(signal.SIGHUP,signal.SIG_IGN);signal.signal(signal.SIGINT,stop);signal.signal(signal.SIGTERM,stop)
    try:
        while not stopped:
            checkpoint=latest_snapshot() if args.mode=='mapping' else None
            print('SLAM engine starting'+(' from '+checkpoint['prefix'] if checkpoint else ''),flush=True)
            child=subprocess.Popen(resumed_command(command,args.mode,checkpoint),start_new_session=True)
            while child.poll() is None:time.sleep(.1)
            if stopped or child.returncode in (0,2):break
            print('SLAM engine exited '+str(child.returncode)+'; retrying in 3 seconds',flush=True)
            until=time.monotonic()+3.
            while not stopped and time.monotonic()<until:time.sleep(.1)
    finally:
        if child and child.poll() is None:
            os.killpg(child.pid,signal.SIGINT)
            try:child.wait(timeout=15)
            except subprocess.TimeoutExpired:child.terminate();child.wait(timeout=5)


if __name__=='__main__':main()
