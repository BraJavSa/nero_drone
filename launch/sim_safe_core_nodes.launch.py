from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('neroControl')

    urdf_file = os.path.join(pkg_share, 'urdf', 'bebop2.urdf')
    rviz_config = os.path.join(pkg_share, 'others', 'bebop2_sim.rviz')
    ekf_config = os.path.join(pkg_share, 'config', 'bebop_ekf.yaml')

    return LaunchDescription([

        Node(
            package='neroControl',
            executable='safety_watchdog',
            name='safety_watchdog',
            output='screen'
        ),

        Node(
            package='neroControl',
            executable='tf_odom_to_map',
            name='tf_odom_to_map',
            output='screen'
        ),
        
        Node(
            package='neroControl',
            executable='initial_frame.py',
            name='initial_frame',
            output='screen'
        ),

        Node(
            package='neroControl',
            executable='isfly',
            name='isfly',
            output='screen',
        ),

        Node(
            package='neroControl',
            executable='bebop_control_gui.py',
            name='bebop_control_gui',
            output='screen'
        ),
        
        Node(
            package='neroControl',
            executable='tf_camera',
            name='camera_tf_with_gimbal',
            output='screen'
        ),
    ])
