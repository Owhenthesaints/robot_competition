from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import subprocess
import time
import os


def generate_launch_description():
    child_launch_dir = os.path.join(os.path.abspath(os.path.dirname(__file__)), 'motor_control_launch.py')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(child_launch_dir)
        ),
        Node(
            package='joy_override_cpp',
            executable='control_override',
            namespace="",
            name='control_override_joy',
        ),
    ])


