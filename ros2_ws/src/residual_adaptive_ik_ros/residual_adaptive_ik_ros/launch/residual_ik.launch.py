"""Launch residual IK node in dry-run / validation_only by default."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    mode_arg = DeclareLaunchArgument(
        "mode",
        default_value="validation_only",
        description="baseline_only | supervised_residual | sac_residual | validation_only",
    )
    node = Node(
        package="residual_adaptive_ik_ros",
        executable="residual_ik_node",
        name="residual_ik_node",
        output="screen",
        parameters=[{"mode": LaunchConfiguration("mode")}],
    )
    return LaunchDescription([mode_arg, node])
