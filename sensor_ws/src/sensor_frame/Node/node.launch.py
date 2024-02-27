import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import  IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    #Define LaunchDescription variable
    ld =LaunchDescription()
    ldlidar_launch=IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource([
            get_package_share_directory('ldlidar_node'),
            '/launch/ldlidar_with_mgr.launch.py'
        ])
    ),
    rpllidar_launch = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource([
            get_package_share_directory('sllidar_ros2'),
            '/launch/view_sllidar_s2_launch.py'
        ]),
       
    ),



    #Call nodes
    ld.add_action(ldlidar_launch)
    ld.add_action(rpllidar_launch)



    return ld 