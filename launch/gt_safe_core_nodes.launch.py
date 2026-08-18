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
            executable='gt_mocap_odom.py',
            name='gt_mocap_odom',
            output='screen',
            parameters=[{
                'mocap_topic': '/vrpn_mocap/bebop/pose',
                'gt_odom_topic': '/bebop/gt_fullodom',
                'alias_odom_topic': '/bebop/gt_odom',
                'publish_rate': 15.0,
                'world_frame': 'world',
                'child_frame': 'bebop_gt',
                'filter_window': 1,
            }]
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
            executable='gt_tf_camera',
            name='camera_tf_with_gimbal',
            output='screen'
        ),
    ])
