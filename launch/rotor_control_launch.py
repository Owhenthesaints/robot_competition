from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rotor_control',
            executable='rotor_control',
            namespace="",
            name = 'rotor_control',
            shell =True,
        ),
        Node(
            package='joy_override_cpp',
            executable='control_override',
            namespace="",
            name='control_override_joy'
        )
    ])


