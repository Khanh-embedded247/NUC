from launch import LaunchDescription
import launch_ros.actions
import os

def generate_launch_description():
    config=os.path.dirname(os.path.abspath(__file__))
    config =os.path.join(config,"turtlesim.yaml")
    print(config)

    return LaunchDescription(
        [
            launch_ros.actions.Node(
                namespace="turtlesim1",
                package="turtlesim",
                executable="turtlesim_node",
                output="screen",
            ),
            launch_ros.actions.Node(
                namespace="turtlesim2",
                package="turtlesim",
                executable="turtlesim_node",
                arguments=["--ross-args","--remap","__node:=my_turtle"],
                remappings=[("/turtlesim2/my_turtle/cmd_vel","turtlesim2/my_turtle/cmd_vel")],
                parameters=[{config}],
                output ="screen",
            ),
        ]
    )