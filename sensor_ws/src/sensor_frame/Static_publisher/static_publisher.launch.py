import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld=LaunchDescription

    static_tf_lidar1 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'ldlidar_base'],
        name='static_tf_lidar1',
        output='screen',
    )

    static_tf_lidar2 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'rpllidar'],
        name='static_tf_lidar2',
        output='screen',
    )

    static_tf_sum = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'base_link'],
        name='static_tf_sum',
        output='screen',
    )


    ld.add_action(static_tf_lidar1)
    ld.add_action(static_tf_lidar2)
    ld.add_action(static_tf_sum)


    return ld