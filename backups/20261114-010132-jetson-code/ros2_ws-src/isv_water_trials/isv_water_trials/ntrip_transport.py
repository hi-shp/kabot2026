"""NTRIP wire helpers. Credentials and server response bodies never enter logs."""
import base64
from urllib.parse import quote


def request(config):
    mount = quote(str(config['mountpoint']).strip('/'), safe='')
    token = base64.b64encode((config['username'] + ':' + config['password']).encode()).decode()
    return (f'GET /{mount} HTTP/1.0\r\nHost: {config["host"]}:{config.get("port", 2101)}\r\n'
            'User-Agent: NTRIP KABOAT/1.0\r\nAccept: */*\r\n'
            f'Authorization: Basic {token}\r\nConnection: close\r\n\r\n').encode('ascii')


def valid_gga(sentence):
    s = sentence.strip()
    try:
        body, checksum = s[1:].split('*')
        value = 0
        for c in body:
            value ^= ord(c)
        fields = body.split(',')
        if not s.startswith('$') or value != int(checksum, 16):
            return None
        if fields[0] not in ('GPGGA', 'GNGGA') or int(fields[6]) <= 0:
            return None
        if not fields[2] or not fields[4]:
            return None
        return s, int(fields[6])
    except (ValueError, IndexError):
        return None


def crc24q(data):
    crc = 0
    for value in data:
        crc ^= value << 16
        for _ in range(8):
            crc <<= 1
            if crc & 0x1000000:
                crc ^= 0x1864CFB
    return crc & 0xffffff


class RTCMFrames:
    def __init__(self):
        self.buffer = bytearray()

    def feed(self, data):
        self.buffer.extend(data)
        frames = []
        while self.buffer:
            start = self.buffer.find(b'\xd3')
            if start < 0:
                self.buffer.clear()
                break
            del self.buffer[:start]
            if len(self.buffer) < 3:
                break
            if self.buffer[1] & 0xfc:
                del self.buffer[0]
                continue
            length = ((self.buffer[1] & 3) << 8 | self.buffer[2]) + 6
            if len(self.buffer) < length:
                break
            frame = bytes(self.buffer[:length])
            if crc24q(frame[:-3]) != int.from_bytes(frame[-3:], 'big'):
                del self.buffer[0]
                continue
            del self.buffer[:length]
            frames.append(frame)
        return frames


class CasterResponse:
    """Incremental HTTP/ICY header and optional HTTP chunk decoder."""
    def __init__(self):
        self.buffer = bytearray()
        self.ready = False
        self.chunked = False
        self.remaining = None

    def feed(self, data):
        self.buffer.extend(data)
        if not self.ready:
            if self.buffer.startswith(b'ICY 200 OK\r\n'):
                end = len(b'ICY 200 OK\r\n')
            else:
                end = self.buffer.find(b'\r\n\r\n')
                if end < 0:
                    if len(self.buffer) > 16384:
                        raise ValueError('invalid_caster_response')
                    return b''
                end += 4
                first = bytes(self.buffer).split(b'\r\n', 1)[0].split()
                if len(first) < 2 or first[0] not in (b'HTTP/1.0', b'HTTP/1.1') or first[1] != b'200':
                    raise ValueError('caster_rejected_request')
                header = bytes(self.buffer[:end]).lower()
                if b'gnss/sourcetable' in header:
                    raise ValueError('mountpoint_not_found')
                self.chunked = b'transfer-encoding: chunked' in header
            del self.buffer[:end]
            self.ready = True
        if not self.chunked:
            result = bytes(self.buffer)
            self.buffer.clear()
            return result
        result = bytearray()
        while True:
            if self.remaining is None:
                end = self.buffer.find(b'\r\n')
                if end < 0:
                    break
                self.remaining = int(bytes(self.buffer[:end]).split(b';')[0], 16)
                del self.buffer[:end + 2]
                if self.remaining <= 0 or self.remaining > 1048576:
                    raise ValueError('caster_stream_ended')
            if len(self.buffer) < self.remaining + 2:
                break
            result.extend(self.buffer[:self.remaining])
            if self.buffer[self.remaining:self.remaining+2] != b'\r\n':
                raise ValueError('invalid_chunk')
            del self.buffer[:self.remaining + 2]
            self.remaining = None
        return bytes(result)
