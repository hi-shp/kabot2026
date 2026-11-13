"""No ROS or hardware: command freshness, ownership and competing sender checks."""
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from isv_shape_navigation import output_runtime as runtime


class OutputRuntimeTests(unittest.TestCase):
    def test_only_recent_completed_navigation_can_drive(self):
        lease=runtime.CommandLease()
        self.assertTrue(lease.read(10)[1])
        lease.publish(10,(1650,1660),True)
        self.assertEqual(lease.read(10.2),((1650,1660),''))
        self.assertEqual(lease.read(10.26),((1500,1500),'navigation command expired'))
        self.assertTrue(lease.read(9)[1])
        lease.publish(11,(1650,1660),False)
        self.assertEqual(lease.read(11),((1500,1500),'navigation not ready'))

    def test_project_senders_share_one_lock_and_release_on_close(self):
        with tempfile.TemporaryDirectory(dir=Path(__file__).parent) as directory:
            source=Path(directory)/'주행코드'/'pkg'/'pkg'/'output_runtime.py'
            with patch.object(runtime,'__file__',str(source)),patch.dict('os.environ',{'ROS_DOMAIN_ID':'50'}):
                first=runtime.acquire_output_lock()
                try:
                    with self.assertRaisesRegex(RuntimeError,'owns actuator output'):
                        runtime.acquire_output_lock()
                finally:first.close()
                with runtime.acquire_output_lock():pass

    def test_known_legacy_senders_and_duplicate_missions_are_detected(self):
        self.assertEqual(runtime.conflicting_output_nodes(['course_mission'],'course_mission'),[])
        self.assertEqual(runtime.conflicting_output_nodes(['course_mission','rc_auto_mode_test'],'course_mission'),['rc_auto_mode_test'])
        self.assertEqual(runtime.conflicting_output_nodes(['course_mission','course_mission'],'course_mission'),['course_mission'])
        self.assertEqual(runtime.conflicting_output_nodes(['course_mission','arduino_actuator_bridge'],'course_mission'),[])


if __name__=='__main__':unittest.main()
