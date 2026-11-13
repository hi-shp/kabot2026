"""Independent mapper; no sensor restart, mission node or actuator commands."""
from pathlib import Path
from launch import LaunchDescription
from launch.actions import ExecuteProcess


def generate_launch_description():
    root=Path(__file__).resolve().parents[1]
    return LaunchDescription([ExecuteProcess(cmd=['bash',str(root/'run_mapping.sh')],output='screen')])
