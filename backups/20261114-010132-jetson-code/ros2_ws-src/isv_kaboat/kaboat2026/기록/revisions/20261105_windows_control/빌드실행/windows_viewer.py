"""Read-only Windows companion. Subscriptions + localhost HTTP only.

Never publishes ROS messages, calls actuator/start services or starts sensors.
The native mission GUI can continue running independently on the Jetson.
"""
import argparse
import json
from pathlib import Path
import time


def create_dashboard(port):
    from isv_shape_navigation.mission_dashboard import Dashboard
    dashboard=Dashboard(Path(__file__).resolve().parent/'windows_view_assets',port)
    def reject_commands(handler):
        handler.send(403,b'{"error":"Read-only viewer: no control commands are accepted"}')
    dashboard.server.RequestHandlerClass.do_POST=reject_commands
    return dashboard


def self_test():
    import urllib.request
    import urllib.error
    dashboard=create_dashboard(0)
    url='http://127.0.0.1:'+str(dashboard.port)
    try:
        assert dashboard.server.server_address[0]=='127.0.0.1'
        assert '읽기 전용' in urllib.request.urlopen(url).read().decode()
        assert b'viewerOnly' in urllib.request.urlopen(url+'/app.js').read()
        assert json.loads(urllib.request.urlopen(url+'/api/state').read())['state']=='WAIT_NODE'
        req=urllib.request.Request(url+'/api/command',data=b'{"action":"start"}',
            headers={'Content-Type':'application/json','Origin':url})
        try:urllib.request.urlopen(req);raise AssertionError('Control POST must be rejected')
        except urllib.error.HTTPError as exc:assert exc.code==403
        assert dashboard.commands.empty()
        assert urllib.request.urlopen(url+'/camera.jpg').status==204
        print('READ_ONLY_VIEWER_TESTS_OK: localhost only; GET works; POST rejected; no command queued; no ROS publishers/services created')
    finally:dashboard.close()


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--port',type=int,default=8771)
    parser.add_argument('--self-test',action='store_true')
    args=parser.parse_args()
    if args.self_test:self_test();return
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from std_msgs.msg import String
    from sensor_msgs.msg import Image
    from cv_bridge import CvBridge
    import cv2
    dashboard=create_dashboard(args.port)
    rclpy.init();node=rclpy.create_node('isv_windows_read_only_viewer')
    bridge=CvBridge();last_frame=-float('inf')
    def status(message):
        try:
            report=json.loads(message.data)
            report['viewer_only']=True
            dashboard.publish(report)
        except (ValueError,KeyError,TypeError):pass
    def camera(message):
        nonlocal last_frame
        now=time.monotonic()
        if now-last_frame<.35:return
        try:
            image=bridge.imgmsg_to_cv2(message,desired_encoding='bgr8')
            height,width=image.shape[:2]
            image=cv2.resize(image,(480,round(height*480/width)))
            ok,jpeg=cv2.imencode('.jpg',image,[cv2.IMWRITE_JPEG_QUALITY,70])
            if ok:dashboard.camera(jpeg.tobytes());last_frame=now
        except (ValueError,RuntimeError,cv2.error):pass
    subscriptions=[node.create_subscription(String,'/course/status',status,10),
                   node.create_subscription(Image,'/camera/image_raw',camera,qos_profile_sensor_data)]
    print('WINDOWS READ-ONLY VIEW READY: http://127.0.0.1:'+str(dashboard.port),flush=True)
    print('Use an SSH local-port tunnel. No propulsion/start commands are available here.',flush=True)
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        dashboard.close();node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
