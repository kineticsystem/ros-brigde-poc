"""
This is a launch file to execute all bridge nodes.
"""

from launch import LaunchDescription
from launch.actions import Shutdown
from launch_ros.actions import Node


def generate_launch_description():
    """
    Create a list of launch description entities to be executed by a ros2 launch
    command.
    :returns: A List of launch description entities.
    """

    bridge = Node(
        package="bridge",
        executable="bridge_server.py",
        name="bridge_server",
        output="both",
    )

    ros_bridge = Node(
        package="rosbridge_server",
        executable="rosbridge_websocket",
        name="rosbridge_websocket",
        on_exit=Shutdown(),
        output="both",
        parameters=[
            {
                # Required to process multiple services (e.g., execute and cancel objective) in parallel
                "call_services_in_new_thread": True,
                "send_action_goals_in_new_thread": True,
                "port": 3201,
            }
        ],
    )

    ros_api = Node(
        package="rosapi",
        executable="rosapi_node",
        name="rosapi",
        parameters=[
            {
                "topics_glob": "[*]",
            },
        ],
    )

    # The list of the nodes that we want to launch
    nodes = [
        bridge,
        ros_api,
        ros_bridge,
    ]

    return LaunchDescription(nodes)
