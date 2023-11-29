from launch import LaunchDescription
from launch.actions import ExecuteProcess
import os

def generate_launch_description():
    home_dir = os.path.expanduser("~")
    script_path = os.path.join(home_dir, 'ros2_ws/src/image2rtsp/python/rtsp.py')

    return LaunchDescription(
        [
            ExecuteProcess(
                cmd=['python3', script_path],
            ),
        ]
    )
