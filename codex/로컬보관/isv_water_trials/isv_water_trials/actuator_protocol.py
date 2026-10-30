"""Versioned CRC-protected atomic PWM protocol. No ROS/hardware imports."""
from dataclasses import dataclass
import re


def crc16(data):
    crc = 0xFFFF
    for value in data:
        crc ^= value << 8
        for _ in range(8):
            crc = ((crc << 1) ^ (0x1021 if crc & 0x8000 else 0)) & 0xFFFF
    return crc


def packet(payload):
    data = payload.encode('ascii')
    return data + f'*{crc16(data):04X}\n'.encode('ascii')


def command(sequence, operation, left=1500, right=1500):
    if type(sequence) is not int or not 0 <= sequence <= 65535 or operation not in ('A', 'C', 'D', 'S', 'R', 'I'):
        raise ValueError('Invalid sequence/operation')
    if any(type(p) is not int or not 1100 <= p <= 1900 for p in (left, right)):
        raise ValueError('PWM must be integer microseconds in 1100..1900')
    if operation != 'C' and (left, right) != (1500, 1500):
        raise ValueError('Arm/disarm must be neutral')
    return packet(f'K1,{sequence},{operation},{left},{right}')


@dataclass(frozen=True)
class Status:
    sequence: int
    mode: int
    armed: bool
    left: int
    right: int
    rc_valid: bool
    estop_known: bool
    estop_active: bool
    stop_latched: bool = False
    protocol_version: int = 1


def status(line):
    if len(line) > 64 or not line.endswith(b'\n'):
        raise ValueError('Invalid line length/terminator')
    match = re.fullmatch(rb'(S[13],[0-9,]+)\*([0-9A-F]{4})\r?\n', line)
    if not match or crc16(match[1]) != int(match[2], 16):
        raise ValueError('Invalid status format/checksum')
    fields=match[1].decode().split(',');version=int(fields[0][1])
    if len(fields)!=(10 if version==3 else 9) or any(not p or len(p)>5 for p in fields[1:]):
        raise ValueError('Invalid status fields')
    values=list(map(int,fields[1:]));seq,mode,armed,left,right,rc,known,active=values[:8]
    stopped=values[8] if version==3 else 0
    if seq > 65535 or mode not in (0,1,2) or any(v not in (0,1) for v in (armed,rc,known,active,stopped)) or not 1100 <= left <= 1900 or not 1100 <= right <= 1900:
        raise ValueError('Invalid status range')
    if stopped and (mode!=0 or armed or (left,right)!=(1500,1500)):
        raise ValueError('Invalid stopped state')
    return Status(seq, mode, bool(armed), left, right, bool(rc), bool(known), bool(active),bool(stopped),version)


def raw_status(line):
    match = re.fullmatch(rb'(R1,([0-9]{1,4}),([0-9]{1,4}),([0-9]{1,4}),([0-9]{1,4}),([0-9]{1,4}))\*([0-9A-F]{4})\n', line)
    if not match or crc16(match[1]) != int(match[7], 16):
        raise ValueError('Invalid raw status')
    values = [int(match[i]) for i in range(2, 7)]
    if any(value != 0 and not 800 <= value <= 2200 for value in values):
        raise ValueError('Invalid pulse observation')
    return dict(zip(('rc_left_us', 'rc_right_us', 'rc_mode_us', 'observed_left_us', 'observed_right_us'), values))
