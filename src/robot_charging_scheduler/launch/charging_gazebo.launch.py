"""Launch Gazebo and the robot charging scheduler."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def _spawn_working_robot(package_share: str, robot_id: int) -> Node:
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
            "0.0",
            "-y",
            "0.0",
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
            "0.0",
            "-y",
            "0.0",
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

    spawn_nodes = [_spawn_charging_robot(package_share)]
    spawn_nodes.extend([_spawn_working_robot(package_share, i) for i in range(1, 7)])

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
            }
        ],
    )

    return LaunchDescription(
        [
            SetEnvironmentVariable("GAZEBO_MODEL_PATH", gazebo_model_path),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(gazebo_launch),
                launch_arguments={"world": world_file, "verbose": "false"}.items(),
            ),
            TimerAction(period=3.0, actions=spawn_nodes),
            TimerAction(period=6.0, actions=[scheduler_node]),
        ]
    )
