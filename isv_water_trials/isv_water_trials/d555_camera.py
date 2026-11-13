#!/usr/bin/env python3
"""Forward D555 native Ethernet images to the boat ROS domain."""
import argparse
import signal
import time

import cv2
import numpy as np
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import QoSProfile,ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import CompressedImage, Image


def color_bgr(message):
    channels = {'yuv422_yuy2': 2, 'yuyv': 2, 'rgb8': 3, 'bgr8': 3, 'mono8': 1}
    count = channels.get(message.encoding.lower())
    if count is None:
        raise ValueError('Unsupported D555 color encoding: ' + message.encoding)
    if message.step < message.width * count or len(message.data) < message.height * message.step:
        raise ValueError('Invalid D555 color frame length or row stride')
    rows = np.frombuffer(message.data, np.uint8).reshape(message.height, message.step)
    pixels = rows[:, :message.width * count].reshape(message.height, message.width, count)
    if count == 2:
        return cv2.cvtColor(pixels, cv2.COLOR_YUV2BGR_YUY2)
    if count == 1:
        return cv2.cvtColor(pixels[:, :, 0], cv2.COLOR_GRAY2BGR)
    return cv2.cvtColor(pixels, cv2.COLOR_RGB2BGR) if message.encoding.lower() == 'rgb8' else pixels


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--serial', default='262422303049')
    parser.add_argument('--source-domain', type=int, default=0)
    parser.add_argument('--topic-prefix',default='/camera/depth/color')
    parser.add_argument('--max-fps',type=float,default=5.)
    parser.add_argument('--max-width',type=int,default=640)
    parser.add_argument('--max-height',type=int,default=480)
    parser.add_argument('--jpeg-quality',type=int,default=60)
    parser.add_argument('--publish-depth',action='store_true')
    parser.add_argument('--publish-raw',action='store_true')
    args = parser.parse_args()
    if not 1<=args.max_fps<=30 or min(args.max_width,args.max_height)<1 or not 1<=args.jpeg_quality<=95:
        parser.error('Invalid camera FPS, dimensions or JPEG quality')
    source_context = Context()
    output_context = Context()
    rclpy.init(context=source_context, domain_id=args.source_domain,
               signal_handler_options=SignalHandlerOptions.NO)
    rclpy.init(context=output_context, signal_handler_options=SignalHandlerOptions.NO)
    receiver = Node('d555_ethernet_receiver', context=source_context)
    output = Node('d555_camera_bridge', context=output_context)
    executor = SingleThreadedExecutor(context=source_context)
    executor.add_node(receiver)
    raw_pub = output.create_publisher(Image, args.topic_prefix+'/image_raw', qos_profile_sensor_data) if args.publish_raw else None
    jpeg_pub = output.create_publisher(CompressedImage, args.topic_prefix+'/image_raw/compressed',
                                       qos_profile_sensor_data)
    depth_pub = output.create_publisher(Image, '/camera/depth/image_rect_raw',
                                        qos_profile_sensor_data) if args.publish_depth else None
    last_color = [time.monotonic()]
    last_error = [0.]
    seen_color = [False]
    stopped = [False]
    last_processed=[-float('inf')]

    def color(message):
        now=time.monotonic()
        if now-last_processed[0]<1/args.max_fps:return
        last_processed[0]=now
        try:
            bgr = color_bgr(message)
            height,width=bgr.shape[:2];scale=min(1.,args.max_width/width,args.max_height/height)
            if scale<1:bgr=cv2.resize(bgr,(max(1,round(width*scale)),max(1,round(height*scale))),interpolation=cv2.INTER_AREA)
            ok, encoded = cv2.imencode('.jpg', bgr, [cv2.IMWRITE_JPEG_QUALITY, args.jpeg_quality])
            if not ok:
                raise ValueError('JPEG encoding failed')
        except (ValueError, cv2.error) as error:
            now = time.monotonic()
            if now - last_error[0] >= 5:
                output.get_logger().error(str(error))
                last_error[0] = now
            return
        stamp = output.get_clock().now().to_msg()
        raw = Image()
        raw.header.stamp = stamp
        raw.header.frame_id = 'd555_color_optical_frame'
        raw.height, raw.width = bgr.shape[:2]
        raw.encoding = 'bgr8'
        raw.step = raw.width * 3
        if raw_pub:raw.data = bgr.tobytes()
        jpeg = CompressedImage()
        jpeg.header = raw.header
        jpeg.format = 'bgr8; jpeg compressed bgr8'
        jpeg.data = encoded.tobytes()
        if raw_pub:raw_pub.publish(raw)
        jpeg_pub.publish(jpeg)
        last_color[0] = time.monotonic()
        if not seen_color[0]:
            output.get_logger().info(f'D555 color receiving {raw.width}x{raw.height}')
            seen_color[0] = True

    def depth(message):
        message.header.stamp = output.get_clock().now().to_msg()
        message.header.frame_id = 'd555_depth_optical_frame'
        depth_pub.publish(message)

    prefix = '/realsense/D555_' + args.serial
    qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT)
    subscriptions = [receiver.create_subscription(Image, prefix + '_Color', color, qos)]
    if args.publish_depth:subscriptions.append(receiver.create_subscription(Image, prefix + '_Depth', depth, qos))
    # No D555 Motion subscription or IMU publisher. Heading remains on VectorNav.
    output.get_logger().info(
        f'D555 Ethernet domain {args.source_domain} -> image topics; VectorNav supplies IMU')
    signal.signal(signal.SIGINT, lambda *_: stopped.__setitem__(0, True))
    signal.signal(signal.SIGTERM, lambda *_: stopped.__setitem__(0, True))
    try:
        while not stopped[0] and source_context.ok() and output_context.ok():
            executor.spin_once(timeout_sec=.1)
            now = time.monotonic()
            if now - last_color[0] > 5 and now - last_error[0] >= 5:
                output.get_logger().warning('D555 color frames delayed; check Ethernet power and IP')
                last_error[0] = now
    finally:
        executor.shutdown()
        receiver.destroy_node()
        output.destroy_node()
        source_context.try_shutdown()
        output_context.try_shutdown()


if __name__ == '__main__':
    main()
