
"""
<launch>

  <arg name="node_name" default="vesc_driver_nodelet" />
  <arg name="start_manager" default="true" />
  <arg name="manager" default="vesc_nodelet_manager" />

  <!-- Optionally launch manager in GDB, for debugging -->
  <arg name="debug" default="false" />
  <arg if="$(arg debug)" name="launch_prefix" value="xterm -e gdb --args" />
  <arg unless="$(arg debug)" name="launch_prefix" value="" />
  <!-- Worker threads -->
  <arg name="num_worker_threads" default="4" />

  <!-- VESC driver parameters -->
  <arg name="port" default="/dev/ttyUSB0" />
  
  <!-- Also globally disable bond heartbeat timeout in debug mode, so everything
       doesn't die when you hit a break point -->
  <param if="$(arg debug)" name="/bond_disable_heartbeat_timeout" value="true" />

  <!-- Nodelet manager -->
  <node if="$(arg start_manager)" pkg="nodelet" type="nodelet" name="$(arg manager)"
        args="manager" output="screen" launch-prefix="$(arg launch_prefix)">
     <param name="num_worker_threads" value="$(arg num_worker_threads)" />
  </node>

  <!-- VESC driver nodelet -->
  <node pkg="nodelet" type="nodelet" name="$(arg node_name)"
        args="load vesc_driver::VescDriverNodelet $(arg manager)" >
    <param name="port" value="$(arg port)" />
  </node>    

</launch>"""
# vesc_driver_launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='nodelet',
            executable='nodelet',
            name='vesc_nodelet_manager',
            arguments=['manager'],
            output='screen',
        ),
        Node(
            package='nodelet',
            executable='nodelet',
            name='vesc_driver_nodelet',
            arguments=['load', 'vesc_driver/VescDriverNodelet', 'vesc_nodelet_manager'],
            output='screen',
            parameters=[
                {'port': '/dev/ttyACM*'},
                {'num_worker_threads': 4},
            ],
        ),
    ])
