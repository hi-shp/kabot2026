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
    navigation=OccupancyGrid();navigation.header.frame_id='tank_map'
    nav_info=json.loads(files.get('navigation_grid.json',files['grid.json']))
    navigation.info.width=nav_info['width'];navigation.info.height=nav_info['height'];navigation.info.resolution=nav_info['resolution']
    for k,v in zip(('x','y','z'),nav_info['position']):setattr(navigation.info.origin.position,k,v)
    for k,v in zip(('x','y','z','w'),nav_info['orientation']):setattr(navigation.info.origin.orientation,k,v)
    navigation.data=array('b',files.get('navigation_grid.bin',files['grid.bin']))
    nav_publisher=node.create_publisher(OccupancyGrid,'/tank/navigation_map',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    def publish():
        msg.header.stamp=node.get_clock().now().to_msg();publisher.publish(msg)
        navigation.header.stamp=msg.header.stamp;nav_publisher.publish(navigation)
    node.create_timer(1.,publish);publish()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
