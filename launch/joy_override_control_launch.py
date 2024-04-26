from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rotor_control',
            executable='rotor_control',
            namespace="",
            name = 'rotor_control',
            prefix = ["sudo -E env \"PYTHONPATH=$PYTHONPATH\" \"LD_LIBRARY_PATH=$LD_LIBRARY_PATH\" \"PATH=$PATH\" \"USER=$USER\" bash -c "],
            shell =True,
        ),
        Node(
            package = 'joy_override_cpp',
            executable='control_override',
            namespace = "",
            name = 'joy_override_cpp',
            prefix = ["sudo -E env \"PYTHONPATH=$PYTHONPATH\" \"LD_LIBRARY_PATH=$LD_LIBRARY_PATH\" \"PATH=$PATH\" \"USER=$USER\" bash -c "],
            shell =True,
        )
    ])


