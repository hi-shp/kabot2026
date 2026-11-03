"""Periodic complete SLAM snapshots, published only after files are synced."""
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import time
from .tank_storage import atomic_json


def main():
    import argparse
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--interval',type=float,default=60.)
    parser.add_argument('--output',default='~/water_trials/tank_maps/autosave')
    from rclpy.utilities import remove_ros_args
    args = parser.parse_args(remove_ros_args()[1:])
    if args.interval<30: parser.error('interval must be >=30s')
    root = Path(args.output).expanduser(); root.mkdir(parents=True,exist_ok=True)
    stopped = False; child = None
    def stop(*_):
        nonlocal stopped
        stopped = True
        if child and child.poll() is None: child.send_signal(signal.SIGINT)
    signal.signal(signal.SIGINT,stop); signal.signal(signal.SIGTERM,stop)
    try:
        while not stopped:
            deadline = time.monotonic()+args.interval
            if shutil.disk_usage(root).free>500*1024**2:
                directory = root/time.strftime('%Y%m%dT%H%M%S')
                directory.mkdir(exist_ok=True); prefix = directory/'tank'
                with (directory/'save.log').open('w',encoding='utf-8') as stream:
                    child = subprocess.Popen([sys.executable,'-m','isv_water_trials.tank_save','--name',str(prefix)],
                        stdout=stream,stderr=subprocess.STDOUT)
                    try: result = child.wait(timeout=90.)
                    except subprocess.TimeoutExpired:
                        child.kill(); child.wait(); result = 1
                if result==0:
                    for path in directory.iterdir():
                        if path.is_file():
                            with path.open('rb') as stream: os.fsync(stream.fileno())
                    atomic_json(directory/'complete.json',{'map_file':str(prefix),'saved_unix':time.time()})
                    atomic_json(root/'latest.json',{'map_file':str(prefix),'saved_unix':time.time()})
                    print('MAP_SAVED '+str(prefix),flush=True)
                else: print('MAP_SAVE_WAIT '+str(directory/'save.log'),flush=True)
            else: print('MAP_SAVE_WAIT: less than 500 MiB free',flush=True)
            while not stopped and time.monotonic()<deadline: time.sleep(.25)
    finally:
        if child and child.poll() is None: child.terminate(); child.wait(timeout=10)


if __name__=='__main__': main()
