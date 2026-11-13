"""Forward authenticated NTRIP RTCM into the existing u-blox ROS port owner."""
import json
from pathlib import Path
import queue
import socket
import ssl
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nmea_msgs.msg import Sentence
from rtcm_msgs.msg import Message
from std_msgs.msg import String

from .ntrip_transport import CasterResponse, RTCMFrames, request, valid_gga


class NtripBridge(Node):
    def __init__(self):
        super().__init__('ntrip_bridge')
        filename = self.declare_parameter('config_file', str(Path.home()/'.config/kabot/ntrip.json')).value
        self.config = json.loads(Path(filename).expanduser().read_text())
        self.gga = ('', 0.0, 0)
        self.stopping = threading.Event()
        self.frames = queue.Queue(maxsize=512)
        self.state = 'waiting_gga'
        self.last_rtcm = 0.0
        self.frame_count = 0
        self.byte_count = 0
        self.types = {}
        self.rtcm = self.create_publisher(Message, '/rtcm', 10)
        self.status = self.create_publisher(String, '/gps/corrections', 10)
        self.create_subscription(Sentence, '/nmea', self.on_nmea, qos_profile_sensor_data)
        self.create_timer(.05, self.forward)
        self.create_timer(1., self.report)
        self.worker = threading.Thread(target=self.receive, daemon=True)
        self.worker.start()

    def on_nmea(self, msg):
        parsed = valid_gga(msg.sentence)
        if parsed:
            self.gga = (parsed[0], time.monotonic(), parsed[1])

    def receive(self):
        delay = 2
        while not self.stopping.is_set():
            gga, stamp, _ = self.gga
            if not gga or time.monotonic() - stamp > 10:
                self.state = 'waiting_gga'
                self.stopping.wait(1)
                continue
            try:
                self.state = 'connecting'
                with socket.create_connection((self.config['host'], int(self.config.get('port', 2101))), timeout=8) as raw:
                    sock = ssl.create_default_context().wrap_socket(raw, server_hostname=self.config['host']) if self.config.get('ssl') else raw
                    with sock:
                        sock.settimeout(1)
                        sock.sendall(request(self.config))
                        sock.sendall((gga + '\r\n').encode('ascii'))
                        sent = last_frame = time.monotonic()
                        response, parser = CasterResponse(), RTCMFrames()
                        while not self.stopping.is_set():
                            now = time.monotonic()
                            gga, stamp, _ = self.gga
                            if now - stamp > 10:
                                raise TimeoutError('gga_stale')
                            if now - sent >= 5:
                                sock.sendall((gga + '\r\n').encode('ascii'))
                                sent = now
                            if now - last_frame > 20:
                                raise TimeoutError('corrections_stale')
                            try:
                                chunk = sock.recv(8192)
                            except socket.timeout:
                                continue
                            if not chunk:
                                raise ConnectionError()
                            for frame in parser.feed(response.feed(chunk)):
                                last_frame = time.monotonic()
                                # Never deliver a stale backlog after a blocked ROS executor.
                                if self.frames.full():
                                    try:
                                        self.frames.get_nowait()
                                    except queue.Empty:
                                        pass
                                self.frames.put_nowait((last_frame, frame))
                                self.state = 'receiving'
                                delay = 2
            except (OSError, ValueError) as exc:
                # Exception strings can include remote content; expose only safe classes.
                self.state = 'retry_' + type(exc).__name__
            self.stopping.wait(delay)
            delay = min(delay * 2, 30)

    def forward(self):
        for _ in range(100):
            try:
                stamp, frame = self.frames.get_nowait()
            except queue.Empty:
                break
            if time.monotonic() - stamp > 2:
                continue
            msg = Message()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.message = list(frame)
            self.rtcm.publish(msg)
            self.last_rtcm = stamp
            self.frame_count += 1
            self.byte_count += len(frame)
            if len(frame) >= 8:
                kind = (frame[3] << 4) | (frame[4] >> 4)
                self.types[str(kind)] = self.types.get(str(kind), 0) + 1

    def report(self):
        now = time.monotonic()
        _, stamp, quality = self.gga
        status = dict(state=self.state, frames=self.frame_count, bytes=self.byte_count,
                      rtcm_age_s=round(now-self.last_rtcm, 2) if self.last_rtcm else None,
                      gga_age_s=round(now-stamp, 2) if stamp else None,
                      fix_quality=quality if now-stamp < 10 else 0,
                      rtcm_types=self.types, receiver_subscribers=self.rtcm.get_subscription_count())
        self.status.publish(String(data=json.dumps(status)))

    def close(self):
        self.stopping.set()
        self.worker.join(timeout=10)


def main(args=None):
    rclpy.init(args=args)
    node = NtripBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
