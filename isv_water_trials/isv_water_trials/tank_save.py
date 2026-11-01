"""Save both occupancy image/YAML and the SLAM pose graph for localization."""
import argparse
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--name', default='~/water_trials/tank_maps/tank')
    args = parser.parse_args()
    path = Path(args.name).expanduser().resolve(); path.parent.mkdir(parents=True, exist_ok=True)
    import rclpy
    from rclpy.node import Node
    from slam_toolbox.srv import SaveMap, SerializePoseGraph
    rclpy.init(); node = Node('tank_save')
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
        print('SAVED '+str(path)+' (.yaml / image / .posegraph / .data)')
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()


if __name__ == '__main__': main()
