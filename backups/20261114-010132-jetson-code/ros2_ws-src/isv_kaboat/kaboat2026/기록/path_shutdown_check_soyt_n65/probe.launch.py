from launch import LaunchDescription
from launch.actions import ExecuteProcess
def generate_launch_description():
    return LaunchDescription([ExecuteProcess(cmd=['/usr/bin/python3','-c','import time; print(12345,flush=True); time.sleep(30)'],output='screen')])
