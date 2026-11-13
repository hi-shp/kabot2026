"""Stop other vessel ROS processes; retain water_trial and its dashboard."""
import json
import os
from pathlib import Path
import signal
import time


def snapshot():
    result={}
    for p in Path('/proc').glob('[0-9]*'):
        try:
            if p.stat().st_uid!=os.getuid():continue
            fields=(p/'stat').read_text().rsplit(')',1)[1].split()
            if fields[0]=='Z':continue
            args=(p/'cmdline').read_bytes().split(b'\0')
            result[int(p.name)]=(int(fields[1]),fields[19],args)
        except (OSError,IndexError):pass
    return result


def operational(args):
    if b'isv_water_trials.stop_nodes' in args:return False
    home=os.fsencode(Path.home()/'ros2_ws')+b'/'
    if any(a.startswith(home+b'install/') for a in args):return True
    if any(a.startswith(home+b'src/isv_kaboat/kaboat2026/') and a.endswith((b'.py',b'.launch.py',b'.sh')) for a in args):return True
    if b'-m' in args:
        i=args.index(b'-m')+1
        if i<len(args) and args[i].startswith((b'isv_water_trials.',b'isv_shape_navigation.',b'kaboat_avoidance.')):return True
    if any(a.endswith(b'/ros2') or a==b'ros2' for a in args):
        return b'launch' in args or (b'bag' in args and b'record' in args)
    return False


def water_trial(args):
    """Identify a water_trial entry point, including its shared sensor launch."""
    home=os.fsencode(Path.home()/'ros2_ws')+b'/'
    if any(a.startswith((home+b'install/isv_water_trials/',home+b'src/isv_water_trials/')) for a in args):
        return True
    if b'-m' in args:
        index=args.index(b'-m')+1
        if index<len(args) and args[index].startswith(b'isv_water_trials.'):
            return True
    if b'launch' in args and (b'isv_water_trials' in args or
                              (b'mechaship_bringup' in args and b'all_sensors.launch.py' in args)):
        return True
    return False


def targets_to_stop(processes):
    """Preserve every descendant of a water_trial root, even shared ROS drivers."""
    roots={pid for pid,(_,_,args) in processes.items() if operational(args)}

    def owner(pid):
        seen=set()
        while pid in roots and pid not in seen:
            seen.add(pid)
            parent=processes[pid][0]
            if parent not in roots:return pid
            pid=parent
        return pid

    protected_roots={pid for pid in roots if owner(pid)==pid and water_trial(processes[pid][2])}
    protected=set(protected_roots)
    while True:
        expanded=protected|{pid for pid,(parent,_,_) in processes.items() if parent in protected}
        if expanded==protected:break
        protected=expanded
    foreign_roots=roots-protected
    targets=set(foreign_roots)
    while True:
        expanded=targets|{pid for pid,(parent,_,_) in processes.items() if parent in targets}
        if expanded==targets:break
        targets=expanded
    return foreign_roots,targets-protected


def disarm():
    node=None;owned=False
    try:
        import rclpy
        from std_srvs.srv import Trigger
        if not rclpy.ok():rclpy.init(args=['--ros-args','--disable-rosout-logs']);owned=True
        node=rclpy.create_node('dashboard_stop_nodes',enable_rosout=False,start_parameter_services=False)
        client=node.create_client(Trigger,'/actuators/stop')
        if not client.wait_for_service(timeout_sec=1.):return False
        future=client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(node,future,timeout_sec=2.)
        return bool(future.done() and future.result() and future.result().success)
    except Exception:return False
    finally:
        if node is not None:node.destroy_node()
        if owned:
            import rclpy
            if rclpy.ok():rclpy.shutdown()


def stop(*,disarm_first=True):
    processes=snapshot();roots,targets=targets_to_stop(processes)
    acknowledged=disarm() if targets and disarm_first else False
    targets.discard(os.getpid())
    identities={pid:processes[pid][1] for pid in targets}
    def remaining():
        current=snapshot()
        return {pid for pid,start in identities.items() if pid in current and current[pid][1]==start}
    for sig,seconds in ((signal.SIGINT,20),(signal.SIGTERM,3),(signal.SIGKILL,1)):
        active=remaining()
        if not active:break
        # Stop respawn supervisors before their descendants. Controllers get
        # their normal SIGINT cleanup; bag writers have time to flush.
        supervisors={pid for pid in roots if b'launch' in processes[pid][2] or b'_worker' in processes[pid][2]}
        for pid in sorted(active,key=lambda p:p not in supervisors):
            try:os.kill(pid,sig)
            except ProcessLookupError:pass
        until=time.monotonic()+seconds
        while remaining() and time.monotonic()<until:time.sleep(.1)
    pending=remaining()
    # Include a newly respawned operational process instead of claiming success.
    _,new_targets=targets_to_stop(snapshot())
    pending.update(pid for pid in new_targets if pid!=os.getpid())
    return dict(ok=not pending,stopped=len(targets)-len(remaining()),remaining=sorted(pending),disarm_acknowledged=acknowledged)


def stop_watertrial():
    """Stop WaterTrial-owned ROS process trees while preserving unrelated work."""
    processes=snapshot()
    roots={pid for pid,(_,_,args) in processes.items() if operational(args) and water_trial(args)}
    targets=set(roots)
    while True:
        expanded=targets|{pid for pid,(parent,_,_) in processes.items() if parent in targets}
        if expanded==targets:break
        targets=expanded
    targets.discard(os.getpid())
    identities={pid:processes[pid][1] for pid in targets}
    def remaining():
        current=snapshot()
        return {pid for pid,start in identities.items() if pid in current and current[pid][1]==start}
    for sig,seconds in ((signal.SIGINT,20),(signal.SIGTERM,3),(signal.SIGKILL,1)):
        active=remaining()
        if not active:break
        supervisors={pid for pid in roots if b'launch' in processes[pid][2] or b'_worker' in processes[pid][2]}
        for pid in sorted(active,key=lambda p:p not in supervisors):
            try:os.kill(pid,sig)
            except ProcessLookupError:pass
        until=time.monotonic()+seconds
        while remaining() and time.monotonic()<until:time.sleep(.1)
    pending=remaining()
    current=snapshot()
    pending.update(pid for pid,(_,_,args) in current.items()
                   if pid!=os.getpid() and operational(args) and water_trial(args))
    return dict(ok=not pending,stopped=len(targets)-len(remaining()),remaining=sorted(pending))


if __name__=='__main__':
    result=stop();print(json.dumps(result),flush=True)
    raise SystemExit(0 if result['ok'] else 1)
