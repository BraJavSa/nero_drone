import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    # Configuración del nodo de Joy (Lectura del mando físico)
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
    )

    # Configuración de tu nodo de control (Mapeo a Twist)
    joy2cmd_node = Node(
        package='neroControl',
        executable='joy2cmd',
        name='joy2cmd_node',
        output='screen',
    )

    return LaunchDescription([
        joy_node,
        joy2cmd_node
    ])