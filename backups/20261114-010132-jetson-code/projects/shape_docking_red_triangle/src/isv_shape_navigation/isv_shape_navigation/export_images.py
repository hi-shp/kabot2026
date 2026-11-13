"""Export original camera frames and timestamps from a ROS2 bag; no fake labels."""
import argparse
import json
from pathlib import Path


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag', type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--topic', default='/camera/image_raw')
    parser.add_argument('--every', type=float, default=.5, help='Minimum interval in seconds')
    args = parser.parse_args(argv)
    if args.every < 0:
        parser.error('--every must be >= 0')
    if args.output.exists():
        parser.error('Output directory must be new')
    import cv2
    import rosbag2_py
    from cv_bridge import CvBridge
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(args.bag.expanduser().resolve()), storage_id=''),
                rosbag2_py.ConverterOptions('', ''))
    types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    if args.topic not in types:
        parser.error('Camera topic absent from bag: '+args.topic)
    if types[args.topic] not in ('sensor_msgs/msg/Image', 'sensor_msgs/msg/CompressedImage'):
        parser.error('Topic is not an image stream')
    kind = get_message(types[args.topic]); bridge = CvBridge()
    args.output.mkdir(parents=True)
    images = args.output/'images'; images.mkdir()
    previous, count = None, 0
    with (args.output/'manifest.jsonl').open('w', encoding='utf-8') as manifest:
        while reader.has_next():
            topic, data, bag_ns = reader.read_next()
            if topic != args.topic:
                continue
            msg = deserialize_message(data, kind)
            ns = msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
            if previous is not None and 0 <= ns-previous < args.every*1e9:
                continue
            frame = (bridge.compressed_imgmsg_to_cv2(msg, 'bgr8') if types[topic].endswith('CompressedImage')
                     else bridge.imgmsg_to_cv2(msg, 'bgr8'))
            name = f'{count:07d}_{ns}.jpg'
            if not cv2.imwrite(str(images/name), frame):
                raise OSError('Image write failed: '+name)
            manifest.write(json.dumps(dict(image='images/'+name, stamp_ns=ns, bag_stamp_ns=bag_ns,
                                           frame_id=msg.header.frame_id, topic=topic, labeled=False))+'\n')
            previous, count = ns, count+1
    print(f'Exported {count} unlabeled images to {args.output}')


if __name__ == '__main__':
    main()
