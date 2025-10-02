from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='nero_drone',
            executable='hello_cpp',
            name='hello_cpp_node'
        ),
        Node(
            package='nero_drone',
            executable='hello_py.py',
            name='hello_py_node'
        )
    ])
