from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import subprocess
import time

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
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='robot2',  # Default namespace
        description='Namespace for the robot'
    )
    open_terminal_and_close()

    time.sleep(2)

    return LaunchDescription([
        log_level_arg,
        namespace_arg,
        Node(
            package='rx_dispatcher',
            executable='rx_dispatcher',
            namespace=LaunchConfiguration('namespace'),
            name = 'rx_dispatcher',
            shell =True,
            arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')]
        ),
        Node(
            package='vision_py',
            executable='camera_node',
            namespace=LaunchConfiguration('namespace'),
            name='camera_node',
            output='screen'
        ),
        Node(
            package='main_control_cpp',
            executable='main_control_cpp',
            namespace=LaunchConfiguration('namespace'),
            name='main_control_cpp',
            output='screen'
        ),
    ])