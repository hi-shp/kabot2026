"""Run the user's fixed Rocket launcher once and observe its management link."""
import ctypes
from ctypes import wintypes
import ipaddress
import json
import os
from pathlib import Path
import socket
import subprocess
import threading
import time

from ssh_discovery import adapter_networks

ROCKET_MAC='58:D6:1F:0E:CF:CD'
LAUNCHER=Path.home()/'\ub2e4\uc6b4\ub85c\ub4dc/outputs/Rocket-Connect.cmd'


def neighbor_matches(source,target,mac):
    """Read IPv4 interface/ARP tables without changing network settings."""
    if os.name!='nt':return False
    class Address(ctypes.Structure):
        _fields_=[(name,wintypes.DWORD) for name in ('ip','index','mask','broadcast','reassembly')]+[('unused',wintypes.WORD),('kind',wintypes.WORD)]
    class Neighbor(ctypes.Structure):
        _fields_=[('index',wintypes.DWORD),('length',wintypes.DWORD),('mac',ctypes.c_ubyte*8),('ip',wintypes.DWORD),('kind',wintypes.DWORD)]
    api=ctypes.WinDLL('iphlpapi')
    def rows(name,kind):
        size=wintypes.ULONG(0);function=getattr(api,name)
        if function(None,ctypes.byref(size),False)!=122:return []
        buffer=ctypes.create_string_buffer(size.value)
        if function(buffer,ctypes.byref(size),False):return []
        count=ctypes.cast(buffer,ctypes.POINTER(wintypes.DWORD)).contents.value
        return [kind.from_buffer_copy(buffer,4+i*ctypes.sizeof(kind)) for i in range(count)]
    source_number=int.from_bytes(socket.inet_aton(source),'little')
    target_number=int.from_bytes(socket.inet_aton(target),'little')
    indexes={row.index for row in rows('GetIpAddrTable',Address) if row.ip==source_number}
    return any(row.index in indexes and row.ip==target_number and row.length==6 and
               ':'.join(f'{b:02X}' for b in row.mac[:6])==mac for row in rows('GetIpNetTable',Neighbor))


def management_alive(result):
    if result.get('Success') is not True or result.get('RocketMAC','').replace('-',':').upper()!=ROCKET_MAC:return False
    source=str(ipaddress.IPv4Address(result['LaptopManagementIP']))
    target=str(ipaddress.IPv4Address(result['ManagementIP']))
    if source not in {ip for ip,_ in adapter_networks('rocket')}:return False
    try:
        with socket.socket() as connection:
            connection.settimeout(.5);connection.bind((source,0));connection.connect((target,443))
        return neighbor_matches(source,target,ROCKET_MAC)
    except OSError:return False


class RocketPower:
    def __init__(self,launcher=LAUNCHER,probe=management_alive):
        self.launcher=Path(launcher);self.result_path=self.launcher.with_name('Rocket-Connect-result.json')
        self.probe=probe;self.lock=threading.RLock();self.stop=threading.Event();self.monitor=None;self.worker=None;self.last_launch=-float('inf')
        self.value=dict(state='unknown',busy=False,powered=False,error=None,management_ip=None,browser_opened=False)

    def start_monitor(self):
        with self.lock:
            if self.monitor and self.monitor.is_alive():return
            self.monitor=threading.Thread(target=self.observe,daemon=True);self.monitor.start()

    def result(self):
        try:
            value=json.loads(self.result_path.read_text(encoding='utf-8-sig'))
            return value if isinstance(value,dict) else {}
        except (OSError,ValueError):return {}

    def refresh(self):
        result=self.result()
        try:powered=self.probe(result)
        except (OSError,ValueError,KeyError,TypeError,AttributeError):powered=False
        with self.lock:
            self.value.update(powered=bool(powered),management_ip=result.get('ManagementIP') if powered else None)
            if not self.value['busy']:
                if powered:self.value.update(state='on',error=None)
                elif self.value['state']!='error':self.value['state']='off'
        return self.snapshot()

    def observe(self):
        while not self.stop.is_set():
            self.refresh();self.stop.wait(3)

    def snapshot(self):
        with self.lock:return dict(self.value)

    def request(self):
        with self.lock:
            if self.value['busy'] or self.value['powered'] or time.monotonic()-self.last_launch<5:return self.snapshot()
            if not self.launcher.is_file():raise OSError('Rocket-Connect.cmd 파일을 찾지 못했습니다: '+str(self.launcher))
            self.value.update(state='starting',busy=True,error=None,browser_opened=False)
            self.last_launch=time.monotonic()
            self.worker=threading.Thread(target=self.run,daemon=True);self.worker.start()
            return self.snapshot()

    def run(self):
        try:
            previous=self.result_path.stat().st_mtime_ns if self.result_path.exists() else None
            env=dict(os.environ,ROCKET_CONNECT_NO_PAUSE='1');env.pop('ROCKET_CONNECT_NO_BROWSER',None)
            logs=Path(os.environ.get('LOCALAPPDATA',str(Path.home()/'.cache')))/'KabotSensorViewer'
            logs.mkdir(parents=True,exist_ok=True)
            command=subprocess.list2cmdline([os.environ.get('COMSPEC','C:/Windows/System32/cmd.exe')])+' /d /s /c ""'+str(self.launcher)+'""'
            with (logs/'rocket-connect.log').open('wb') as log:
                process=subprocess.Popen(command,cwd=str(self.launcher.parent),env=env,stdin=subprocess.DEVNULL,
                    stdout=log,stderr=subprocess.STDOUT,creationflags=getattr(subprocess,'CREATE_NO_WINDOW',0))
                code=process.wait()
            result=self.result()
            fresh=self.result_path.is_file() and self.result_path.stat().st_mtime_ns!=previous
            if code!=0 or not fresh or result.get('Success') is not True:
                raise OSError(result.get('Error') if fresh and result.get('Error') else '로켓 연결이 완료되지 않았습니다. 케이블·PoE 전원과 Windows 관리자 승인창을 확인하세요.')
            powered=self.probe(result)
            with self.lock:
                self.value.update(state='on' if powered else 'off',busy=False,powered=bool(powered),error=None,
                    management_ip=result.get('ManagementIP') if powered else None,browser_opened=bool(result.get('BrowserOpened')))
        except (OSError,ValueError,KeyError,TypeError,AttributeError) as exc:
            with self.lock:self.value.update(state='error',busy=False,powered=False,error=str(exc))

    def close(self):
        # Do not interrupt the user's network setup or its administrator prompt.
        self.stop.set()
