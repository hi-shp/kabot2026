"""Publish the immutable saved occupancy grid; localization updates pose only."""
import json
from array import array
from .tank_map_file import read_bundle


def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile,DurabilityPolicy
    from nav_msgs.msg import OccupancyGrid
    rclpy.init();node=Node('tank_fixed_map');node.declare_parameter('file','')
    _,_,files=read_bundle(node.get_parameter('file').value)
    info=json.loads(files['grid.json']);msg=OccupancyGrid();msg.header.frame_id='tank_map'
    msg.info.width=info['width'];msg.info.height=info['height'];msg.info.resolution=info['resolution']
    for k,v in zip(('x','y','z'),info['position']):setattr(msg.info.origin.position,k,v)
    for k,v in zip(('x','y','z','w'),info['orientation']):setattr(msg.info.origin.orientation,k,v)
    msg.data=array('b',files['grid.bin'])
    publisher=node.create_publisher(OccupancyGrid,'/map',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    def publish():
        msg.header.stamp=node.get_clock().now().to_msg();publisher.publish(msg)
    node.create_timer(1.,publish);publish()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
