"""Identify vessel links by interface hardware, independent of DHCP addresses."""
import getpass
import json
import os
from pathlib import Path
import subprocess
import time

# Dedicated onboard USB Ethernet adapter connected to the Rocket bridge.
ROCKET_MACS={'80:69:1a:e8:5b:26'}
_cached={}
_updated=-float('inf')


def classify_interface(name,mac,sys_root=Path('/sys/class/net')):
    if (sys_root/name/'wireless').is_dir() or (sys_root/name/'phy80211').exists():return 'hotspot'
    if mac.lower() in ROCKET_MACS:return 'rocket'
    return None


def snapshot():
    global _cached,_updated
    if time.monotonic()-_updated<4:return _cached
    connection=os.environ.get('SSH_CONNECTION','').split()
    server_ip=connection[2] if len(connection)==4 else None
    result={'connection_path':None,'server_ip':server_ip,'remote_user':getpass.getuser(),
            'endpoints':{},'evidence':'실제 접속 인터페이스 확인 중'}
    try:
        rows=json.loads(subprocess.run(['ip','-j','-4','address','show'],capture_output=True,text=True,check=True,timeout=1).stdout)
        for row in rows:
            name=row['ifname']
            try:mac=(Path('/sys/class/net')/name/'address').read_text().strip().lower()
            except OSError:mac=row.get('address','').lower()
            kind=classify_interface(name,mac)
            addresses=[entry['local'] for entry in row.get('addr_info',[]) if entry.get('family')=='inet' and entry.get('scope')=='global']
            if kind:
                endpoint={'interface':name,'mac':mac,'addresses':addresses,'link_up':'LOWER_UP' in row.get('flags',[])}
                result['endpoints'][kind]=endpoint
                if server_ip in addresses:
                    result['connection_path']=kind
                    result['evidence']=f'SSH 수신 인터페이스 {name} · MAC {mac}'
    except (OSError,ValueError,KeyError,subprocess.SubprocessError) as exc:
        result['evidence']='네트워크 정보 수신 대기: '+str(exc)
    _cached=result;_updated=time.monotonic()
    return result
