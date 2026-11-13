"""Existing GUI with disposable synthetic map on localhost:8782 for visual QA.

No ROS initialization or device interfaces. All saved files are temporary.
"""
import json
from pathlib import Path
import sys
import tempfile
import threading
import time
from types import SimpleNamespace
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[3]/'빌드실행'))
from windows_viewer import create_dashboard
from docking_mapping.gui_bridge import MappingBridge
from docking_mapping.gui_commands import execute
from docking_mapping.preview import mapping_preview
from docking_mapping.reference import ReferenceMap
from docking_mapping.session import MappingSession
from test_reference import ReferenceTests


def main():
    with tempfile.TemporaryDirectory(prefix='gui_preview_hopping_') as directory:
        mapper,selection=ReferenceTests().fixture()
        ref=ReferenceMap.from_mapper(mapper,selection)
        session=MappingSession();session.reference_ready()
        folder=ref.save(directory)
        node=SimpleNamespace(map_lock=threading.RLock(),mapper=mapper,reference=ref,session=session,
            hopping=None,hopping_dirty=False,rc_ready=lambda:True,reference_folder=folder,config=mapper.config)
        report=dict(simulation=True,state='REFERENCE_LOADED',phase='REFERENCE_READY',initial_mapping_state='FROZEN',
                    mode='PAUSED',map_quality='CONFIRMED',reference_id=ref.reference_id,reference_version=1,
                    rc_ready=True,hopping_config=mapper.config['hopping'],last_gui_result=None)
        mapper.gps_pose=np.eye(4);mapper.corrected_pose=np.eye(4)
        mapper.path.extend([(time.time(),-1.,-1.,0.),(time.time(),0.,0.,0.)])
        def refresh():
            with node.map_lock:
                report.update(hopping_dirty=node.hopping_dirty,time_now_s=time.time())
                mapper.gps_pose_stamp=time.time();mapper.accepted_poses.clear();mapper.accepted_poses.append((time.time(),np.eye(4)))
                bridge.update(json.dumps(mapping_preview(mapper,ref,node.hopping,mapper.config,report)))
        def transport(payload):
            try:
                message=execute(node,payload)
                report['last_gui_result']=dict(request_id=payload['request_id'],success=True,message=message)
            except Exception as exc:
                report['last_gui_result']=dict(request_id=payload['request_id'],success=False,message=str(exc))
            refresh()
            return dict(success=True,message='queued fixture operation')
        bridge=MappingBridge(controls_enabled=True,transport=transport)
        refresh();dashboard=create_dashboard(8782,None,docking=bridge)
        dashboard.publish(dict(stamp=time.time(),state='SENSORS_ONLY',output=dict(enabled=False),
            intended_pwm=[1500,1500],position=dict(latitude=35.12,longitude=129.10,sigma=.1),
            gps_age=0.,waypoints=[],viewer_only=True,settings_available=False))
        print('GUI_FIXTURE_READY: http://127.0.0.1:8782 (synthetic map; no hardware)',flush=True)
        try:
            while True:refresh();time.sleep(.5)
        except KeyboardInterrupt:pass
        finally:dashboard.close()


if __name__=='__main__':main()
