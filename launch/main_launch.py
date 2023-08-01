import os
from typing import List

from launch_ros.actions import Node

from launch import LaunchDescription, LaunchDescriptionEntity
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.launch_context import LaunchContext
from launch.substitutions import LaunchConfiguration

# Global launch arguments and constants. Should be the same across all launch files.
ROS_PACKAGES_DIR = os.path.join(os.getenv("ROS_WORKSPACE"), "src")
GLOBAL_LAUNCH_ARGUMENTS = [
    DeclareLaunchArgument(
        name="config",
        default_value=os.path.join(ROS_PACKAGES_DIR, "global_launch", "config", "globals.yaml"),
        description="Path to ROS parameter config file.",
    ),
    # Reference: https://answers.ros.org/question/311471/selecting-log-level-in-ros2-launch-file/
    DeclareLaunchArgument(
        name="log_level",
        default_value=["info"],
        description="Logging level",
    ),
    DeclareLaunchArgument(
        name="mode",
        default_value="simulation",
        choices=["production", "simulation"],
        description="System mode.",
    ),
]

# Local launch arguments and constants
PACKAGE_NAME = "network_systems"

# Add args with DeclareLaunchArguments object(s) and utilize in setup_launch()
LOCAL_LAUNCH_ARGUMENTS = []


def generate_launch_description() -> LaunchDescription:
    """The launch file entry point. Generates the launch description for the `network_systems`
    package.

    Returns:
        LaunchDescription: The launch description.
    """
    return LaunchDescription(
        [*GLOBAL_LAUNCH_ARGUMENTS, *LOCAL_LAUNCH_ARGUMENTS, OpaqueFunction(function=setup_launch)]
    )


def setup_launch(context: LaunchContext) -> List[LaunchDescriptionEntity]:
    """Collects launch descriptions that describe the system behavior in the `network_systems`
    package.

    Args:
        context (LaunchContext): The current launch context.

    Returns:
        List[LaunchDescriptionEntity]: Launch descriptions.
    """
    launch_description_entities = list()
    launch_description_entities.append(get_cached_fib_description(context))
    return launch_description_entities


def get_cached_fib_description(context: LaunchContext) -> Node:
    """Gets the launch description for the cached_fib node.

    Args:
        context (LaunchContext): The current launch context.

    Returns:
        Node: The node object that launches the cached_fib node.
    """
    ros_parameters = [LaunchConfiguration("config").perform(context)]
    ros_arguments = [
        "--log-level",
        [f"cached_fib:=", LaunchConfiguration("log_level")],
    ]

    node = Node(
        package=PACKAGE_NAME,
        namespace="example",
        executable="example",
        name="cached_fib",
        parameters=ros_parameters,
        ros_arguments=ros_arguments,
    )

    return node
