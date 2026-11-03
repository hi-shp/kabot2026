"""Split sensor bags; reserve disk space for control logs and cleanly close bags."""
import os
from pathlib import Path
import shutil
import signal
import subprocess
import time

TOPICS = ['/scan','/velodyne_points','/vectornav/imu','/vectornav/raw/common',
          '/tank/odom','/tank/pose','/tank/session','/tank/status','/tank/prediction',
          '/actuators/status','/water_trial/command','/tf','/tf_static','/map','/gps/fix']


def main():
    root = Path.home()/'water_trials/bags'; root.mkdir(parents=True,exist_ok=True)
    stopped = False; child = None
    def stop(*_):
        nonlocal stopped
        stopped = True
    signal.signal(signal.SIGINT,stop); signal.signal(signal.SIGTERM,stop)
    signal.signal(signal.SIGHUP,signal.SIG_IGN)
    try:
        while not stopped:
            free = shutil.disk_usage(root).free
            if child and child.poll() is not None: child = None
            if child is None and free>2*1024**3:
                output = root/time.strftime('tank_%Y%m%dT%H%M%S')
                child = subprocess.Popen(['ros2','bag','record','-o',str(output),
                    '--max-bag-duration','60','--max-bag-size','268435456',
                    '--storage-preset-profile','resilient',*TOPICS],start_new_session=True)
            if child and free<1024**3:
                os.killpg(child.pid,signal.SIGINT); child.wait(timeout=20); child = None
                print('RAW_RECORD_WAIT: reserved 1 GiB for control logs',flush=True)
            time.sleep(1)
    finally:
        if child and child.poll() is None:
            os.killpg(child.pid,signal.SIGINT); child.wait(timeout=20)


if __name__=='__main__': main()
