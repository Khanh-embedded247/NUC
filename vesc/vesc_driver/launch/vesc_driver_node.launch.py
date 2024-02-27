
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='vesc_driver',
            executable='vesc_driver_node',
            name='vesc_driver_node',
            output='screen',
            parameters=[
                {'port': '/dev/ttyACM*'},
                {'num_motor_pole_pairs': 10},
            ],
        ),
    ])
"""<launch>

  <arg name="node_name" default="vesc_driver_node" />
  <arg name="debug" default="false" />
  <arg if="$(arg debug)" name="launch_prefix" value="gdb -ex run --args" />
  <arg unless="$(arg debug)" name="launch_prefix" value="" />

  <!-- VESC driver parameters -->
  <arg name="port" default="/dev/ttyACM*" />
  <arg name="num_motor_pole_pairs" default="10" />

  <!-- VESC driver node -->
  <node pkg="vesc_driver" exec="vesc_driver_node" name="$(arg node_name)" output="screen" launch-prefix="$(arg launch_prefix)">
    <param name="port" value="$(arg port)" />
    <param name="num_motor_pole_pairs" value="$(arg num_motor_pole_pairs)" />
  </node>
</launch> 
"""

