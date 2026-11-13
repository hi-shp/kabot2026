#!/usr/bin/env python3
"""Check camera image payload/rate; optionally display or save a frame."""
import os
from pathlib import Path
from _common import parser,run

def inspect_image(msg):
    if msg is None: return dict(valid=False,issues=['No camera image received'])
    expected=int(msg.height)*int(msg.step)
    valid=msg.width>0 and msg.height>0 and msg.step>0 and bool(msg.encoding) and len(msg.data)>=expected
    return dict(valid=valid,width=int(msg.width),height=int(msg.height),encoding=msg.encoding,
        step_bytes=int(msg.step),data_bytes=len(msg.data),expected_bytes=expected,
        issues=[] if valid else ['Invalid dimensions, encoding, stride, or truncated image payload'])

def main():
    p=parser(__doc__);p.add_argument('--topic',default='/camera/image_raw')
    p.add_argument('--show',action='store_true',help='Open live image window; q exits')
    p.add_argument('--save-image',type=Path,help='Save the latest frame (requires cv_bridge/OpenCV)')
    args=p.parse_args();cv2=None;bridge=None;last=[None]
    if args.show or args.save_image:
        try:
            import cv2
            from cv_bridge import CvBridge
            bridge=CvBridge()
        except ImportError as exc: p.error('cv_bridge and OpenCV required for --show/--save-image: '+str(exc))
    if args.show and not (os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY')):
        p.error('--show requires a desktop display; omit it for SSH-only checks')
    def analyze(latest):
        msg=latest.get(args.topic);result=inspect_image(msg)
        if args.save_image and result['valid']:
            target=args.save_image.expanduser();target.parent.mkdir(parents=True,exist_ok=True)
            saved=cv2.imwrite(str(target),bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8'))
            result.update(image_saved=saved,image_path=str(target))
            if not saved: result['issues'].append('Failed to save image');result['valid']=False
        return result
    def display(latest):
        msg=latest.get(args.topic)
        if msg is not None and msg is not last[0] and inspect_image(msg)['valid']:
            cv2.imshow('camera_test (q to exit)',bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8'));last[0]=msg
        return cv2.waitKey(1)&255!=ord('q')
    try: return run('camera',args,{args.topic:'sensor_msgs/msg/Image'},analyze,display if args.show else None)
    finally:
        if args.show: cv2.destroyAllWindows()

if __name__=='__main__': raise SystemExit(main())
