"""PC-side IPv4 discovery for the selected Jetson connection path."""
from concurrent.futures import ThreadPoolExecutor
import ctypes
from ctypes import wintypes
from datetime import datetime
import ipaddress
from pathlib import Path
import re
import shutil
import socket
import sys


def adapter_networks(route):
    if sys.platform != 'win32':
        return []

    class IpString(ctypes.Structure):
        pass

    IpString._fields_ = [('next', ctypes.POINTER(IpString)), ('address', ctypes.c_char*16),
                         ('mask', ctypes.c_char*16), ('context', wintypes.DWORD)]

    class Adapter(ctypes.Structure):
        pass

    Adapter._fields_ = [('next', ctypes.POINTER(Adapter)), ('combo', wintypes.DWORD),
                        ('name', ctypes.c_char*260), ('description', ctypes.c_char*132),
                        ('mac_length', wintypes.UINT), ('mac', ctypes.c_ubyte*8),
                        ('index', wintypes.DWORD), ('kind', wintypes.UINT),
                        ('dhcp', wintypes.UINT), ('current', ctypes.POINTER(IpString)),
                        ('ips', IpString)]
    class InterfaceRow(ctypes.Structure):
        _fields_ = [('name', wintypes.WCHAR*256), ('index', wintypes.DWORD),
                    ('kind', wintypes.DWORD), ('mtu', wintypes.DWORD), ('speed', wintypes.DWORD),
                    ('mac_length', wintypes.DWORD), ('mac', ctypes.c_ubyte*8),
                    ('admin', wintypes.DWORD), ('oper', wintypes.DWORD),
                    ('remaining_counters_and_description', ctypes.c_ubyte*512)]

    size = wintypes.ULONG(0)
    library = ctypes.WinDLL('iphlpapi')
    api = library.GetAdaptersInfo
    api.argtypes = [ctypes.c_void_p, ctypes.POINTER(wintypes.ULONG)]
    api.restype = wintypes.ULONG
    if api(None, ctypes.byref(size)) != 111:
        return []
    buffer = ctypes.create_string_buffer(size.value)
    if api(buffer, ctypes.byref(size)):
        return []
    result = []
    item = ctypes.cast(buffer, ctypes.POINTER(Adapter))
    wanted = 71 if route == 'hotspot' else 6
    while item:
        adapter = item.contents
        row = InterfaceRow()
        row.index = adapter.index
        connected = library.GetIfEntry(ctypes.byref(row)) == 0 and row.oper in (4,5)
        if adapter.kind == wanted and connected:
            address = ctypes.pointer(adapter.ips)
            while address:
                value = address.contents
                try:
                    local = ipaddress.IPv4Address(value.address.decode())
                    network = ipaddress.IPv4Network((str(local), value.mask.decode()), strict=False)
                    if not local.is_unspecified and not local.is_link_local and not local.is_loopback:
                        # Search the local /24 around the adapter on broad networks.
                        if network.num_addresses > 256:
                            network = ipaddress.IPv4Network(str(local)+'/24', strict=False)
                        result.append((str(local), network))
                except (ValueError, UnicodeError):
                    pass
                address = value.next
        item = adapter.next
    return result


def discover_candidates(route, port=22):
    targets = set()
    for local, network in adapter_networks(route):
        targets.update((local, str(ip)) for ip in network.hosts() if str(ip) != local)

    def probe(pair):
        local, remote = pair
        with socket.socket() as sock:
            sock.settimeout(.3)
            try:
                sock.bind((local, 0))
                sock.connect((remote, port))
                return remote
            except OSError:
                return None

    with ThreadPoolExecutor(max_workers=48) as pool:
        return sorted({ip for ip in pool.map(probe, sorted(targets)) if ip})


def tcp_ready(route,address,port=22,timeout=.3):
    """Try the known address first, bound to the selected physical network."""
    try:target=ipaddress.IPv4Address(address)
    except ValueError:return False
    for local,network in adapter_networks(route):
        if target not in network or str(target)==local:continue
        try:
            with socket.socket() as connection:
                connection.settimeout(timeout);connection.bind((local,0));connection.connect((str(target),port))
            return True
        except OSError:pass
    return False


def update_verified_alias(host, address, path=None):
    if host not in ('jetson-rocket', 'jetson-hotspot', 'jetson-starlink'):
        raise ValueError('Unsupported connection alias')
    ipaddress.IPv4Address(address)
    path = Path(path) if path else Path.home()/'.ssh/config'
    original = path.read_bytes()
    text = original.decode('utf-8-sig')
    pattern = re.compile(r'(?m)^\s*Host[ \t]+'+re.escape(host)+r'[ \t]*\r?$', re.IGNORECASE)
    start = pattern.search(text)
    if start is None and host=='jetson-starlink':
        source=re.search(r'(?ms)^[ \t]*Host[ \t]+jetson-hotspot[ \t]*\r?$\n(.*?)(?=^[ \t]*Host[ \t]+|\Z)',text)
        if source is None:raise ValueError('Verified Jetson hotspot SSH identity is missing')
        values={}
        for line in source[1].splitlines():
            fields=line.strip().split(None,1)
            if len(fields)==2:values[fields[0].lower()]=fields[1]
        if not values.get('hostkeyalias'):raise ValueError('Pinned Jetson HostKeyAlias is missing')
        added='\nHost jetson-starlink\n    HostName '+address+'\n'
        for name in ('User','IdentityFile','HostKeyAlias'):
            if name.lower() in values:added+='    '+name+' '+values[name.lower()]+'\n'
        added+='    ServerAliveInterval 3\n    ServerAliveCountMax 2\n    ConnectTimeout 4\n    ConnectionAttempts 1\n    TCPKeepAlive yes\n'
        updated_text=text.rstrip()+'\n'+added
        if path.read_bytes()!=original:raise RuntimeError('SSH config changed during discovery')
        backup=path.with_name(path.name+'.auto-'+datetime.now().strftime('%Y%m%d-%H%M%S-%f'))
        shutil.copy2(path,backup)
        temporary=path.with_name(path.name+'.incoming');temporary.write_bytes(updated_text.encode('utf-8'));temporary.replace(path)
        return
    if start is None:
        raise ValueError('SSH alias block is missing: '+host)
    following = re.search(r'(?m)^[ \t]*Host[ \t]+', text[start.end():], re.IGNORECASE)
    end = start.end()+following.start() if following else len(text)
    block = text[start.start():end]
    updated, count = re.subn(r'(?im)^([ \t]*HostName[ \t]+)\S+',
                             lambda match: match[1]+address, block, count=1)
    if count != 1:
        raise ValueError('SSH HostName is missing: '+host)
    if updated == block:
        return
    result = text[:start.start()]+updated+text[end:]
    if path.read_bytes() != original:
        raise RuntimeError('SSH config changed during discovery')
    backup = path.with_name(path.name+'.auto-'+datetime.now().strftime('%Y%m%d-%H%M%S-%f'))
    shutil.copy2(path, backup)
    temporary = path.with_name(path.name+'.incoming')
    temporary.write_bytes(result.encode('utf-8'))
    temporary.replace(path)
