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
    open_terminal_and_close()

    time.sleep(2)

    return LaunchDescription([
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
            package='joy_override_cpp',
            executable='control_override',
            namespace="",
            name='control_override_joy',
        ),
        Node(
            package='joy',
            executable='joy_node',
            namespace="",
            name='joy',
        ),
    ])


