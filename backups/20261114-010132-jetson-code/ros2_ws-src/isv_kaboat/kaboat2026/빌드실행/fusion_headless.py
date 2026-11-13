"""Run the installed fusion entrypoint with only its local OpenCV windows disabled.

No installed/source file is changed. Camera/LiDAR association and ROS outputs
still run in the original fusion_viewer implementation.
"""
from pathlib import Path
import runpy
import sys
import cv2
from ament_index_python.packages import get_package_prefix

entry=Path(get_package_prefix('fusion_viewer'))/'lib/fusion_viewer/viewer_node'
if not entry.is_file():raise RuntimeError('Installed fusion_viewer entrypoint missing')
if '--check' in sys.argv:
    print('HEADLESS_FUSION_ENTRY_OK:',entry)
else:
    cv2.imshow=lambda *_args,**_kwargs:None
    cv2.namedWindow=lambda *_args,**_kwargs:None
    cv2.waitKey=lambda *_args,**_kwargs:-1
    cv2.destroyAllWindows=lambda:None
    sys.argv[0]=str(entry)
    runpy.run_path(str(entry),run_name='__main__')
