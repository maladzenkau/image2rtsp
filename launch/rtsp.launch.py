from launch import LaunchDescription
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('image2rtsp')
    script_path = os.path.join(pkg_share, '../../../../src/image2rtsp/python/rtsp.py')

    return LaunchDescription(
        [
            ExecuteProcess(
                cmd=['python3', script_path],
            ),
        ]
    )
