import os

from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    viewer = os.path.join(get_package_prefix('image2rtsp'), 'lib', 'image2rtsp', 'rtsp.py')

    return LaunchDescription([
        DeclareLaunchArgument('url', default_value='rtsp://127.0.0.1:8554/back',
                              description='RTSP stream to display'),
        ExecuteProcess(cmd=[viewer, LaunchConfiguration('url')], output='screen'),
    ])
