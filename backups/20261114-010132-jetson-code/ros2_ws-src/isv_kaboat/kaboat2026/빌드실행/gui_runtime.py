"""Bootstrap once; refreshing the page never restarts stopped programs."""
from pathlib import Path
import fcntl
from viewer_bringup import BringupManager
from viewer_runtime import RuntimeManager,processes
ROOT=Path(__file__).resolve().parent.parent

def ensure_runtime():
    logs=ROOT/'.runtime/logs';logs.mkdir(parents=True,exist_ok=True)
    with (logs/'gui_autostart.lock').open('a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX)
        BringupManager(ROOT).command('start')
        manager=RuntimeManager(ROOT)
        if not processes('mapper'):manager.command('mapper_start')
        if not processes('mission'):manager.command('v3_preview')

if __name__=='__main__':ensure_runtime()
