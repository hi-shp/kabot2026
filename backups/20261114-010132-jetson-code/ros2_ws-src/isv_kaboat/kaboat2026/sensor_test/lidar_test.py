#!/usr/bin/env python3
"""Check Velodyne packets, point-cloud XYZ samples and scan ranges."""
import math
import struct
from _common import parser,run

def inspect_scan(msg):
    if msg is None: return dict(valid=False,issues=['No LaserScan received'])
    good=[float(v) for v in msg.ranges if math.isfinite(v) and msg.range_min<=v<=msg.range_max]
    valid=bool(good) and math.isfinite(msg.angle_increment) and msg.angle_increment>0
    return dict(valid=valid,bins=len(msg.ranges),valid_ranges=len(good),
        nearest_m=min(good) if good else None,farthest_m=max(good) if good else None,
        issues=[] if valid else ['No usable scan returns or invalid angle increment'])

def inspect_cloud(msg):
    if msg is None: return dict(valid=False,issues=['No point cloud received'])
    fields={f.name:f for f in msg.fields};count=int(msg.width)*int(msg.height)
    layout=count>0 and msg.point_step>0 and msg.row_step>=msg.width*msg.point_step and len(msg.data)>=msg.height*msg.row_step
    samples=[]
    if layout and all(k in fields and fields[k].datatype in (7,8) for k in ('x','y','z')):
        for i in range(0,count,max(1,count//256)):
            base=(i//msg.width)*msg.row_step+(i%msg.width)*msg.point_step
            try:
                xyz=[struct.unpack_from(('>' if msg.is_bigendian else '<')+('f' if fields[k].datatype==7 else 'd'),msg.data,base+fields[k].offset)[0] for k in ('x','y','z')]
            except (struct.error,ValueError): break
            if all(math.isfinite(v) for v in xyz): samples.append(xyz)
    return dict(valid=bool(layout and samples),points=count,data_bytes=len(msg.data),finite_xyz_samples=len(samples),
        xyz_example=samples[0] if samples else None,issues=[] if layout and samples else ['Empty/truncated cloud or no finite XYZ samples'])

def main():
    p=parser(__doc__);p.add_argument('--scan-topic',default='/scan');p.add_argument('--cloud-topic',default='/velodyne_points')
    p.add_argument('--packet-topic',default='/velodyne_packets');args=p.parse_args()
    topics={args.scan_topic:'sensor_msgs/msg/LaserScan',args.cloud_topic:'sensor_msgs/msg/PointCloud2',args.packet_topic:'velodyne_msgs/msg/VelodyneScan'}
    def analyze(latest):
        scan=inspect_scan(latest.get(args.scan_topic));cloud=inspect_cloud(latest.get(args.cloud_topic))
        packet=latest.get(args.packet_topic);packets=len(packet.packets) if packet is not None else 0
        return dict(valid=bool(scan['valid'] and cloud['valid'] and packets),scan=scan,cloud=cloud,packets_per_message=packets)
    return run('lidar',args,topics,analyze)

if __name__=='__main__': raise SystemExit(main())
