from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import subprocess
import time
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory



def open_terminal_and_close():
    # Command to run in the terminal
    command = 'timeout 4s pio device monitor;exit;'
    
    # Open gnome-terminal and run the command
    subprocess.Popen(command, shell=True)



def generate_launch_description():
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='INFO',
        description='Log level for the nodes'
    )
    open_terminal_and_close()

    time.sleep(2)

    ekf_launch_file = os.path.join(os.path.abspath(os.path.dirname(__file__)), 'ekf.launch.py')


    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(ekf_launch_file)
        ),
        log_level_arg,
        Node(
            package='rx_dispatcher',
            executable='rx_dispatcher',
            namespace="",
            name = 'rx_dispatcher',
            shell =True,
            arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')]
        ),
        Node(
            package='imu_ready_py',
            executable='imu_publisher',
            namespace="",
            name = 'imu_publisher',
            shell=True,
        ),
    ])


