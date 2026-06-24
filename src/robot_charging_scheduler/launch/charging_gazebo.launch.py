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


def _spawn_working_robot(
    package_share: str, robot_id: int, x: float, y: float
) -> Node:
    """Create a Gazebo spawn node for one working robot."""
    model_file = os.path.join(package_share, "models", "working_robot", "model.sdf")
    return Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-entity",
            f"working_robot_{robot_id}",
            "-file",
            model_file,
            "-x",
            f"{x:.2f}",
            "-y",
            f"{y:.2f}",
            "-z",
            "0.0",
        ],
        output="screen",
    )


def _spawn_charging_robot(package_share: str) -> Node:
    """Create a Gazebo spawn node for the charging robot."""
    model_file = os.path.join(package_share, "models", "charging_robot", "model.sdf")
    return Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-entity",
            "charging_robot",
            "-file",
            model_file,
            "-x",
            "0.80",
            "-y",
            "0.80",
            "-z",
            "0.0",
        ],
        output="screen",
    )


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

    initial_positions = [
        (2.0, 2.0),
        (5.0, 2.0),
        (8.0, 2.0),
        (2.0, 7.0),
        (5.0, 7.0),
        (8.0, 7.0),
    ]
    spawn_actions = [TimerAction(period=3.0, actions=[_spawn_charging_robot(package_share)])]
    for index, (x, y) in enumerate(initial_positions, start=1):
        spawn_actions.append(
            TimerAction(
                period=3.0 + index * 0.4,
                actions=[_spawn_working_robot(package_share, index, x, y)],
            )
        )

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
                "low_battery_threshold": 30.0,
                "charge_amount_per_visit": 45.0,
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
            *spawn_actions,
            TimerAction(period=8.0, actions=[scheduler_node]),
        ]
    )
