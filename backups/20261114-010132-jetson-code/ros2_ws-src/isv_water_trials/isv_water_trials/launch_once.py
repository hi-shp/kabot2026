"""One local ROS resource per user/domain, including launches from other terminals."""
import fcntl
import json
import os
from pathlib import Path
import re


GPS_EXECUTABLES = ('ublox_gps_node','smc_2000','smc_2000_node','smc_gps_topics.py')
_leases = {}


def existing_processes(executables, domain=None):
    domain = str(os.environ.get('ROS_DOMAIN_ID','0') if domain is None else domain)
    matches = []
    for process in Path('/proc').iterdir():
        if not process.name.isdigit() or int(process.name)==os.getpid():continue
        try:
            if process.stat().st_uid!=os.getuid():continue
            stat=(process/'stat').read_text().rsplit(')',1)[1].split()
            if stat[0] in ('Z','X'):continue
            args=(process/'cmdline').read_bytes().split(bytes([0]))
            # ros2 run wrappers are not a second node; match actual executables.
            if not any(Path(os.fsdecode(arg)).name in executables for arg in args[:2] if arg):continue
            env=dict(item.split(b'=',1) for item in (process/'environ').read_bytes().split(bytes([0])) if b'=' in item)
            if os.fsdecode(env.get(b'ROS_DOMAIN_ID',b'0'))==domain:matches.append(int(process.name))
        except (OSError,ValueError,IndexError):continue
    return sorted(matches)


def claim(resource, executables):
    domain=os.environ.get('ROS_DOMAIN_ID','0')
    key=(domain,resource)
    if key in _leases:return False,[]
    root=Path('/tmp')/f'isv_ros_once_{os.getuid()}'
    root.mkdir(mode=0o700,exist_ok=True)
    filename=re.sub(r'[^a-zA-Z0-9_.-]','_',domain+'_'+resource)+'.lock'
    stream=(root/filename).open('a+')
    try:fcntl.flock(stream,fcntl.LOCK_EX|fcntl.LOCK_NB)
    except BlockingIOError:
        stream.close();return False,[]
    existing=existing_processes(executables,domain)
    if existing:
        stream.close();return False,existing
    stream.seek(0);stream.truncate()
    json.dump({'pid':os.getpid(),'domain':domain,'resource':resource},stream);stream.flush()
    # Keep the inode and descriptor until the owning launch exits. Never unlink locks.
    _leases[key]=stream
    return True,[]


def once(resource, executables, actions, condition=None):
    from launch.actions import OpaqueFunction,LogInfo
    def start(context):
        owned,pids=claim(resource,executables)
        if owned:return actions
        suffix=' (PID '+','.join(map(str,pids))+')' if pids else ''
        return [LogInfo(msg='Already running / starting: '+resource+suffix+'; reusing existing nodes.')]
    return OpaqueFunction(function=start,condition=condition)


def node(**kwargs):
    from launch_ros.actions import Node
    executable=kwargs['executable']
    aliases=GPS_EXECUTABLES if executable in GPS_EXECUTABLES else (executable,)
    resource='gps_receiver' if executable in GPS_EXECUTABLES else 'node_'+executable
    return once(resource,aliases,[Node(**kwargs)],condition=kwargs.get('condition'))
