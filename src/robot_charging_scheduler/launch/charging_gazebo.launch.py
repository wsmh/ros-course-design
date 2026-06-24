"""Launch Gazebo and the robot charging scheduler."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Build the launch description for the Gazebo simulation."""
    package_share = get_package_share_directory("robot_charging_scheduler")
    gazebo_share = get_package_share_directory("gazebo_ros")
    world_file = os.path.join(package_share, "worlds", "charging_world.world")
    gazebo_launch = os.path.join(gazebo_share, "launch", "gazebo.launch.py")
    models_path = os.path.join(package_share, "models")
    gazebo_model_path = os.pathsep.join(
        [models_path, os.environ.get("GAZEBO_MODEL_PATH", "")]
    )
    set_entity_state_service = LaunchConfiguration("set_entity_state_service")

    scheduler_node = Node(
        package="robot_charging_scheduler",
        executable="charging_scheduler",
        name="charging_scheduler_node",
        output="screen",
        parameters=[
            {
                "use_gazebo": True,
                "robot_count": 6,
                "timer_period": 1.0,
                "low_battery_threshold": 75.0,
                "charge_amount_per_visit": 100.0,
                "charging_duration_ticks": 5,
                "show_battery_dashboard": True,
                "set_entity_state_service": set_entity_state_service,
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "set_entity_state_service",
                default_value="/set_entity_state",
                description="Gazebo SetEntityState service name.",
            ),
            SetEnvironmentVariable("GAZEBO_MODEL_PATH", gazebo_model_path),
            SetEnvironmentVariable("LIBGL_ALWAYS_SOFTWARE", "1"),
            SetEnvironmentVariable("QT_X11_NO_MITSHM", "1"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(gazebo_launch),
                launch_arguments={"world": world_file, "verbose": "false"}.items(),
            ),
            TimerAction(period=3.0, actions=[scheduler_node]),
        ]
    )
