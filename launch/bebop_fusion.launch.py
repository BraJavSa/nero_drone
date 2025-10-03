from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('nero_drone'),
        'config',
        'ekf.yaml'
    )

    return LaunchDescription([
        Node(
            package='ros2_bebop_driver',
            executable='bebop_driver',
            name='bebop_driver',
            output='screen'
        ),


        Node(
            package='rtabmap_ros',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            parameters=[{
                'frame_id': 'base_link',
                'subscribe_rgb': True,
                'subscribe_depth': False,
                'subscribe_odom_info': False,
            }],
            remappings=[
                ('rgb/image', '/bebop/camera/image_raw'),
                ('rgb/camera_info', '/bebop/camera/camera_info'),
                ('odom', '/bebop/odom')
            ]
        ),

        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[config_file]
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen'
        )
    ])
