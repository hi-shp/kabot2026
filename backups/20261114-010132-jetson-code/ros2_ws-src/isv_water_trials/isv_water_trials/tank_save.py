"""Save both occupancy image/YAML and the SLAM pose graph for localization."""
import argparse
import json
import tempfile
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--name', default='~/water_trials/tank_maps/tank')
    parser.add_argument('--freeze',action='store_true',help='Save one immutable .tankmap and select it as the default map')
    parser.add_argument('--from-snapshot',help='Import an existing saved map prefix into a fixed bundle')
    parser.add_argument('--crop',nargs=4,type=float,metavar=('X0','Y0','X1','Y1'))
    args = parser.parse_args()
    path = Path(args.name).expanduser().resolve(); path.parent.mkdir(parents=True, exist_ok=True)
    target=path if path.suffix=='.tankmap' else Path(str(path)+'.tankmap')
    if args.freeze and target.exists(): parser.error('기존 고정 지도입니다. 새 이름을 사용하세요: '+str(target))
    if args.from_snapshot:
        if not args.freeze:parser.error('--from-snapshot requires --freeze')
        from .tank_map_file import import_snapshot,activate
        result=import_snapshot(args.from_snapshot,target,args.crop);activate(result);print('FROZEN '+str(result));return
    temporary = tempfile.TemporaryDirectory(prefix='tank-freeze-') if args.freeze else None
    if temporary:path=Path(temporary.name)/'tank'
    import rclpy
    from rclpy.node import Node
    from slam_toolbox.srv import SaveMap, SerializePoseGraph
    rclpy.init(); node = Node('tank_save')
    saved={}
    if args.freeze:
        from nav_msgs.msg import OccupancyGrid
        from geometry_msgs.msg import PoseWithCovarianceStamped
        from rclpy.qos import QoSProfile,DurabilityPolicy
        node.create_subscription(OccupancyGrid,'/map',lambda m:saved.update(grid=m),QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        node.create_subscription(PoseWithCovarianceStamped,'/tank/pose',lambda m:saved.update(pose=m),10)
        node.create_subscription(PoseWithCovarianceStamped,'/pose',lambda m:saved.update(slam_pose=m),10)
    try:
        for srv, name, request in [
            (SaveMap, '/slam_toolbox/save_map', SaveMap.Request()),
            (SerializePoseGraph, '/slam_toolbox/serialize_map', SerializePoseGraph.Request())]:
            if isinstance(request, SaveMap.Request): request.name.data = str(path)
            else: request.filename = str(path)
            client = node.create_client(srv, name)
            if not client.wait_for_service(timeout_sec=8.): raise RuntimeError('Service missing: '+name)
            future = client.call_async(request)
            rclpy.spin_until_future_complete(node, future, timeout_sec=30.)
            if not future.done() or future.exception() or future.result().result != 0:
                raise RuntimeError('Map save failed: '+name)
            node.destroy_client(client)
        missing = [str(path)+suffix for suffix in ('.yaml', '.posegraph', '.data') if not Path(str(path)+suffix).is_file()]
        if missing: raise RuntimeError('Saved files missing: '+', '.join(missing))
        if args.freeze:
            import time
            until=time.monotonic()+5
            def fresh_pose():
                now=node.get_clock().now().nanoseconds
                candidates=[saved[k] for k in ('pose','slam_pose') if k in saved]
                candidates=[m for m in candidates if m.header.frame_id=='tank_map' and
                            -.1<=(now-(m.header.stamp.sec*10**9+m.header.stamp.nanosec))/1e9<2.]
                return max(candidates,key=lambda m:(m.header.stamp.sec,m.header.stamp.nanosec)) if candidates else None
            while time.monotonic()<until and ('grid' not in saved or fresh_pose() is None):rclpy.spin_once(node,timeout_sec=.1)
            if 'grid' not in saved:raise RuntimeError('현재 /map을 받지 못했습니다. 기존 파일은 보존됩니다.')
            pose=fresh_pose()
            if pose is None:raise RuntimeError('최근 지도 위치를 받지 못해 고정 저장을 중단했습니다. 지도 작성은 유지됩니다. 라이다·위치 수신을 확인하고 다시 저장하세요.')
            grid=saved['grid'];info=grid.info
            if grid.header.frame_id!='tank_map':raise RuntimeError('Expected tank_map coordinate frame')
            files={p.name:p.read_bytes() for p in path.parent.glob('tank.*') if p.is_file()}
            import re
            image_name=next((n for n in files if n.endswith(('.pgm','.png'))),None)
            if image_name and 'tank.yaml' in files:
                files['tank.yaml']=re.sub(r'(?m)^image:.*$', 'image: '+image_name,files['tank.yaml'].decode()).encode()
            files['grid.json']=json.dumps(dict(width=info.width,height=info.height,resolution=info.resolution,
                position=[getattr(info.origin.position,k) for k in ('x','y','z')],
                orientation=[getattr(info.origin.orientation,k) for k in ('x','y','z','w')])).encode()
            files['grid.bin']=grid.data.tobytes()
            from .tank_core import yaw
            p=pose.pose.pose;seed=[p.position.x,p.position.y,yaw(p.orientation)]
            from .tank_map_file import bundle,activate,crop_files
            if args.crop:files=crop_files(files,args.crop)
            result=bundle(target,files,seed);activate(result)
            print('FROZEN '+str(result))
        else: print('SAVED '+str(path)+' (.yaml / image / .posegraph / .data)')
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
        if temporary: temporary.cleanup()


if __name__ == '__main__': main()
