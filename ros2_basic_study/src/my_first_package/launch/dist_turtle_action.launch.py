from launch import LaunchDescription
from launch_ros.actions import Node, ExecuteProcess

def generate_launch_description():

    my_launch = LaunchDescription()

    turtlesim_node = Node(
        package="turtlesim",
        executable="turtlesim_node",
        parameters=[
            {"background_r": 255},
            {"background_g": 192},
            {"background_b": 203},
        ],
    )

    dist_turtle_action_server = Node(
        package="my_first_package",
        executable="dist_turtle_action_server",
        output="screen",
    )

    my_launch.add_action(turtlesim_node)
    my_launch.add_action(dist_turtle_action_server)

    return my_launch