#!/usr/bin/env python3
"""ROS2 simulation for one charging robot serving multiple working robots."""

import math
import random
from dataclasses import dataclass
from typing import List, Optional

import rclpy
from rclpy.node import Node

try:
    from gazebo_msgs.msg import EntityState
    from gazebo_msgs.srv import SetEntityState
except ModuleNotFoundError:
    EntityState = None
    SetEntityState = None


@dataclass
class RobotState:
    """Store the runtime state of a working robot."""

    robot_id: int
    x: float
    y: float
    battery: float
    working: bool = True


@dataclass
class ChargingRobotState:
    """Store the charging robot position and current target."""

    x: float = 0.0
    y: float = 0.0
    target_id: Optional[int] = None


class ChargingSchedulerNode(Node):
    """Simulate and schedule one charging robot for several working robots."""

    def __init__(self) -> None:
        """Initialize parameters, robot states, and the ROS2 timer."""
        super().__init__("charging_scheduler_node")

        self.declare_parameter("robot_count", 6)
        self.declare_parameter("low_battery_threshold", 30.0)
        self.declare_parameter("full_battery", 100.0)
        self.declare_parameter("charge_amount_per_visit", 45.0)
        self.declare_parameter("work_area_size", 10.0)
        self.declare_parameter("timer_period", 1.0)
        self.declare_parameter("random_seed", 7)
        self.declare_parameter("use_gazebo", False)
        self.declare_parameter("set_entity_state_service", "/set_entity_state")

        self.robot_count = int(self.get_parameter("robot_count").value)
        self.low_battery_threshold = float(
            self.get_parameter("low_battery_threshold").value
        )
        self.full_battery = float(self.get_parameter("full_battery").value)
        self.charge_amount_per_visit = float(
            self.get_parameter("charge_amount_per_visit").value
        )
        self.work_area_size = float(self.get_parameter("work_area_size").value)
        self.timer_period = float(self.get_parameter("timer_period").value)
        self.use_gazebo = bool(self.get_parameter("use_gazebo").value)
        self.set_entity_state_service = str(
            self.get_parameter("set_entity_state_service").value
        )

        if self.robot_count <= 5:
            raise ValueError("robot_count must be greater than 5 for this assignment")

        random_seed = int(self.get_parameter("random_seed").value)
        random.seed(random_seed)

        self.tick_count = 0
        self.charging_robot = ChargingRobotState()
        self.robots = self._create_working_robots()
        self.gazebo_client = None
        if self.use_gazebo:
            if SetEntityState is None:
                raise RuntimeError(
                    "use_gazebo=True requires gazebo_msgs. "
                    "Install it with: sudo apt install ros-$ROS_DISTRO-gazebo-ros-pkgs"
                )
            self.gazebo_client = self.create_client(
                SetEntityState, self.set_entity_state_service
            )

        self.get_logger().info("机器人充电调度仿真启动")
        self._log_state("初始状态")
        self._sync_gazebo_models()
        self.create_timer(self.timer_period, self._on_timer)

    def _create_working_robots(self) -> List[RobotState]:
        """Create working robots with random positions and different battery levels."""
        robots: List[RobotState] = []
        for robot_id in range(1, self.robot_count + 1):
            robots.append(
                RobotState(
                    robot_id=robot_id,
                    x=random.uniform(0.0, self.work_area_size),
                    y=random.uniform(0.0, self.work_area_size),
                    battery=random.uniform(35.0, 95.0),
                )
            )
        return robots

    def _on_timer(self) -> None:
        """Run one simulation step: robots work, scheduler chooses target, then charges."""
        self.tick_count += 1
        self._move_working_robots()

        target = self._choose_next_target()
        if target is None:
            self.get_logger().info(f"[第 {self.tick_count} 轮] 所有机器人电量充足")
            self._log_state("当前状态")
            self._sync_gazebo_models()
            return

        self._move_charging_robot_to(target)
        self._charge_robot(target)
        self._log_state(f"第 {self.tick_count} 轮充电完成")
        self._sync_gazebo_models()

    def _move_working_robots(self) -> None:
        """Move each working robot randomly and consume 1%-2% battery per move."""
        for robot in self.robots:
            if not robot.working:
                continue

            robot.x = self._clamp(
                robot.x + random.uniform(-1.0, 1.0), 0.0, self.work_area_size
            )
            robot.y = self._clamp(
                robot.y + random.uniform(-1.0, 1.0), 0.0, self.work_area_size
            )
            robot.battery = max(0.0, robot.battery - random.uniform(1.0, 2.0))
            if robot.battery <= 0.0:
                robot.working = False
                self.get_logger().warn(f"机器人 R{robot.robot_id} 电量耗尽，停止工作")

    def _choose_next_target(self) -> Optional[RobotState]:
        """Choose the next charging target using urgency and travel distance."""
        candidates = [robot for robot in self.robots if robot.working]
        if not candidates:
            return None

        urgent_candidates = [
            robot
            for robot in candidates
            if robot.battery <= self.low_battery_threshold
            or self._estimate_remaining_moves(robot) <= 15.0
        ]
        if not urgent_candidates:
            return None

        ranked = sorted(urgent_candidates, key=self._priority_score)
        return ranked[0]

    def _priority_score(self, robot: RobotState) -> float:
        """Calculate a lower-is-better score for charging priority."""
        distance = self._distance(
            self.charging_robot.x, self.charging_robot.y, robot.x, robot.y
        )
        remaining_moves = self._estimate_remaining_moves(robot)

        battery_weight = robot.battery
        time_risk_weight = remaining_moves * 2.0
        distance_weight = distance * 1.5
        return battery_weight + time_risk_weight + distance_weight

    def _estimate_remaining_moves(self, robot: RobotState) -> float:
        """Estimate how many random moves a robot can still make."""
        average_consumption = 1.5
        return robot.battery / average_consumption

    def _move_charging_robot_to(self, target: RobotState) -> None:
        """Move the charging robot to the selected target without consuming battery."""
        distance = self._distance(
            self.charging_robot.x, self.charging_robot.y, target.x, target.y
        )
        self.charging_robot.x = target.x
        self.charging_robot.y = target.y
        self.charging_robot.target_id = target.robot_id

        self.get_logger().info(
            f"[第 {self.tick_count} 轮] 充电机器人前往 R{target.robot_id}，"
            f"移动距离 {distance:.2f}，目标电量 {target.battery:.1f}%"
        )

    def _charge_robot(self, target: RobotState) -> None:
        """Charge the selected robot by a fixed amount, capped at full battery."""
        before = target.battery
        target.battery = min(
            self.full_battery, target.battery + self.charge_amount_per_visit
        )
        if target.battery > 0.0:
            target.working = True

        self.get_logger().info(
            f"R{target.robot_id} 充电：{before:.1f}% -> {target.battery:.1f}%"
        )

    def _log_state(self, title: str) -> None:
        """Print a compact battery and position report for all working robots."""
        robot_text = " | ".join(
            [
                f"R{robot.robot_id}: {robot.battery:5.1f}% "
                f"({robot.x:4.1f},{robot.y:4.1f})"
                for robot in self.robots
            ]
        )
        self.get_logger().info(f"{title}: {robot_text}")

    def _sync_gazebo_models(self) -> None:
        """Synchronize all simulated robot states to Gazebo models."""
        if not self.use_gazebo or self.gazebo_client is None:
            return

        if not self.gazebo_client.service_is_ready():
            self.get_logger().warn(
                f"等待 Gazebo {self.set_entity_state_service} 服务就绪"
            )
            return

        self._set_gazebo_entity_pose(
            "charging_robot", self.charging_robot.x, self.charging_robot.y, 0.03
        )
        for robot in self.robots:
            self._set_gazebo_entity_pose(
                f"working_robot_{robot.robot_id}", robot.x, robot.y, 0.02
            )

    def _set_gazebo_entity_pose(
        self, entity_name: str, x: float, y: float, z: float
    ) -> None:
        """Send one model pose update request to Gazebo."""
        if self.gazebo_client is None or SetEntityState is None or EntityState is None:
            return

        request = SetEntityState.Request()
        request.state = EntityState()
        request.state.name = entity_name
        request.state.pose.position.x = x
        request.state.pose.position.y = y
        request.state.pose.position.z = z
        request.state.pose.orientation.w = 1.0
        request.state.reference_frame = "world"
        self.gazebo_client.call_async(request)

    @staticmethod
    def _distance(x1: float, y1: float, x2: float, y2: float) -> float:
        """Calculate Euclidean distance between two points."""
        return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)

    @staticmethod
    def _clamp(value: float, lower: float, upper: float) -> float:
        """Clamp a numeric value to a closed interval."""
        return max(lower, min(upper, value))


def main(args: Optional[List[str]] = None) -> None:
    """Start the ROS2 node."""
    rclpy.init(args=args)
    node = ChargingSchedulerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("用户停止仿真")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
