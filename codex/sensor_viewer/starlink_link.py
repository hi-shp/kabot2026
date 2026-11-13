"""Internet overlay discovery for the dashboard's Starlink connection button."""
from copy import deepcopy
import ipaddress
import json
import os
from pathlib import Path
import shutil
import socket
import subprocess
import threading
import time
from urllib.parse import urlsplit
import webbrowser

OVERLAY = ipaddress.ip_network('100.64.0.0/10')
JETSON_NAME = 'kaboat-jetson'


def overlay_address(value):
    try:
        address = ipaddress.ip_address(value)
        return address.version == 4 and address in OVERLAY
    except ValueError:
        return False


def executable():
    found = shutil.which('tailscale')
    if found:
        return found
    path = Path(os.environ.get('ProgramFiles', 'C:/Program Files'))/'Tailscale/tailscale.exe'
    return str(path) if path.is_file() else None


def parse_status(data):
    backend = data.get('BackendState', '')
    self_info = data.get('Self') or {}
    candidates = []
    for peer in (data.get('Peer') or {}).values():
        name = str(peer.get('HostName', ''))
        dns_name = str(peer.get('DNSName', ''))
        expected = name.casefold() == JETSON_NAME or dns_name.split('.')[0].casefold() == JETSON_NAME
        if not peer.get('Online') or not expected or peer.get('OS') != 'linux':
            continue
        for address in peer.get('TailscaleIPs') or []:
            if overlay_address(address):
                candidates.append(dict(address=address, name=name, preferred=expected))
    candidates.sort(key=lambda p: (not p['preferred'], p['name'], p['address']))
    candidates = candidates[:16]
    local_ips = [ip for ip in (data.get('TailscaleIPs') or []) if overlay_address(ip)]
    if backend == 'NeedsLogin':
        state, message = 'needs_login', '스타링크 최초 로그인 필요 · 노트북과 젯슨을 같은 Tailscale 계정에 등록하세요'
    elif backend == 'Stopped':
        state, message = 'stopped', '스타링크 VPN이 꺼져 있습니다 · 연결 버튼으로 시작하세요'
    elif backend != 'Running' or self_info.get('Online') is False:
        state, message = 'network_wait', '노트북 인터넷·Tailscale 연결 대기'
    elif not candidates:
        state, message = 'peer_offline', '배의 젯슨 연결 대기 · Kk 인터넷과 젯슨 Tailscale 등록을 확인하세요'
    else:
        state, message = 'ready', '스타링크 VPN 준비됨 · 젯슨 SSH 확인 중'
    return dict(state=state, message=message, ready=state == 'ready', installed=True,
                backend=backend, local_ips=local_ips, candidates=candidates,
                boat_wifi='Kk', laptop_wifi='welcome_CU', transport='tailscale')


class StarlinkLink:
    def __init__(self, runner=None, clock=None, opener=None):
        self.runner = runner or subprocess.run
        self.clock = clock or time.monotonic
        self.opener = opener or webbrowser.open
        self.lock = threading.RLock()
        self.cached = None
        self.updated = -float('inf')
        self.login_process = None
        self.login_requested = False
        self.login_at = -float('inf')
        self.opened_login = None

    def status(self, refresh=False):
        with self.lock:
            if self.cached is not None and not refresh and self.clock()-self.updated < 2:
                return deepcopy(self.cached)
            cli = executable()
            if not cli:
                value = dict(state='not_installed', ready=False, installed=False,
                    message='스타링크 초기 설정 필요 · 노트북과 젯슨에 Tailscale을 설치·등록하세요',
                    candidates=[], local_ips=[], boat_wifi='Kk', laptop_wifi='welcome_CU', transport='tailscale')
            else:
                try:
                    result = self.runner([cli, 'status', '--json'], capture_output=True, text=True,
                        encoding='utf-8', timeout=2, creationflags=getattr(subprocess, 'CREATE_NO_WINDOW', 0))
                    data = json.loads(result.stdout)
                    if not isinstance(data, dict):
                        raise ValueError('Invalid Tailscale status')
                    value = parse_status(data)
                    login_url = data.get('AuthURL', '')
                    url = urlsplit(login_url)
                    if (self.login_requested and login_url != self.opened_login and
                            url.scheme == 'https' and url.hostname in ('login.tailscale.com', 'controlplane.tailscale.com')):
                        self.opened_login = login_url
                        self.opener(login_url)
                except (OSError, ValueError, subprocess.SubprocessError) as exc:
                    value = dict(state='service_wait', ready=False, installed=True, candidates=[], local_ips=[],
                        message='스타링크 서비스 연결 대기 · Tailscale 실행 상태를 확인하세요', error=type(exc).__name__)
            self.cached, self.updated = value, self.clock()
            return deepcopy(value)

    def prepare_login(self):
        """Only an explicit connection click may start VPN login or open its URL."""
        value = self.status(refresh=True)
        if value['state'] not in ('needs_login', 'stopped') and value.get('backend') != 'NoState':
            return value
        with self.lock:
            if self.login_process is not None and self.login_process.poll() is None:
                return value
            if self.clock()-self.login_at < 30:
                return value
            cli = executable()
            if not cli:
                return value
            command = [cli, 'up', '--timeout=60s']
            if os.name == 'nt':command.append('--unattended=true')
            self.login_process = subprocess.Popen(command,
                stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                creationflags=getattr(subprocess, 'CREATE_NO_WINDOW', 0))
            self.login_at = self.clock()
            self.login_requested = True
            self.updated = -float('inf')
        return value

    def candidates(self):
        value = self.status()
        return [peer['address'] for peer in value['candidates']] if value.get('ready') else []

    def reachable(self, address, port=22):
        if not overlay_address(address) or address not in self.candidates():
            return False
        try:
            with socket.create_connection((address, port), timeout=1):
                return True
        except OSError:
            return False

    def close(self):
        with self.lock:
            if self.login_process is not None and self.login_process.poll() is None:
                self.login_process.terminate()
