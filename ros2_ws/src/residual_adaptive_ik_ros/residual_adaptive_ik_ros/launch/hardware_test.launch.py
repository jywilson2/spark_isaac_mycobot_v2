"""Hardware test launch — requires ENABLE_MYCOBOT_HARDWARE_TESTS=1 in the environment."""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            Node(
                package="residual_adaptive_ik_ros",
                executable="hardware_test_node",
                name="hardware_test_node",
                output="screen",
            )
        ]
    )
