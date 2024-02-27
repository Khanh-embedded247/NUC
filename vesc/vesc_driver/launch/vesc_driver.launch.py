#!/usr/bin/env python3
#CUng cấp các hàm giao tiếp hđh,đường dẫn,thư mục
import os
#hàm lấy đường dẫn đến thư mục chia sẻ của ros2
from ament_index_python.packages import get_package_share_directory
#lấy lớp mô tả file launch trong quá trình khởi chạy
from launch import LaunchDescription
#hàm định nghĩa các ptu đối số khi khởi chạy ,  mô tả quá trình khởi chạy
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
#chỉ định 1 nguồn mô tả python trong lúc chạy
from launch.launch_description_sources import PythonLaunchDescriptionSource 
#lớp đại diẹn giá trị cấu hình khi chạy 
from launch.substitutions import LaunchConfiguration
#định nghĩa node
from launch_ros.actions import Node

def generate_launch_description():
    




    ld = LaunchDescription()
    return ld 