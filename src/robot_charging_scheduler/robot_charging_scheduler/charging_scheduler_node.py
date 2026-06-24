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

DIRECTIONS = [
    ("上", 0, 1),
    ("下", 0, -1),
    ("左", -1, 0),
    ("右", 1, 0),
]


@dataclass
class RobotState:
    """Store the runtime state of a working robot."""

    robot_id: int
    x: float
    y: float
    battery: float
    working: bool = True
    last_action: str = "等待"


@dataclass
class ChargingRobotState:
    """Store the charging robot position and current target."""

    x: float = 1.0
    y: float = 1.0
    target_id: Optional[int] = None
    charging_ticks_remaining: int = 0
    moving_to_target: bool = False


class ChargingSchedulerNode(Node):
    """Simulate and schedule one charging robot for several working robots."""

    def __init__(self) -> None:
        """Initialize parameters, robot states, and the ROS2 timer."""
        super().__init__("charging_scheduler_node")

        self.declare_parameter("robot_count", 6)
        self.declare_parameter("low_battery_threshold", 75.0)
        self.declare_parameter("full_battery", 100.0)
        self.declare_parameter("charge_amount_per_visit", 100.0)
        self.declare_parameter("work_area_size", 10.0)
        self.declare_parameter("timer_period", 1.0)
        self.declare_parameter("random_seed", 7)
        self.declare_parameter("use_gazebo", False)
        self.declare_parameter("set_entity_state_service", "/set_entity_state")
        self.declare_parameter("charging_duration_ticks", 5)
        self.declare_parameter("show_battery_dashboard", True)

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
        self.charging_duration_ticks = int(
            self.get_parameter("charging_duration_ticks").value
        )
        self.show_battery_dashboard = bool(
            self.get_parameter("show_battery_dashboard").value
        )

        if self.robot_count <= 5:
            raise ValueError("robot_count must be greater than 5 for this assignment")
        if self.charging_duration_ticks <= 0:
            raise ValueError("charging_duration_ticks must be greater than 0")

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
        self._print_battery_dashboard()
        self._sync_gazebo_models()
        self.create_timer(self.timer_period, self._on_timer)

    def _create_working_robots(self) -> List[RobotState]:
        """Create working robots on readable grid cells with safe initial battery."""
        initial_positions = [
            (2, 2),
            (5, 2),
            (8, 2),
            (2, 7),
            (5, 7),
            (8, 7),
        ]
        robots: List[RobotState] = []
        for robot_id in range(1, self.robot_count + 1):
            if robot_id <= len(initial_positions):
                x, y = initial_positions[robot_id - 1]
            else:
                x, y = self._find_free_random_cell(robots)
            robots.append(
                RobotState(
                    robot_id=robot_id,
                    x=float(x),
                    y=float(y),
                    battery=random.uniform(75.0, 100.0),
                )
            )
        return robots

    def _on_timer(self) -> None:
        """Run one simulation step: robots work, scheduler chooses target, then charges."""
        self.tick_count += 1

        if self.charging_robot.moving_to_target:
            self._move_working_robots(excluded_robot_id=self.charging_robot.target_id)
            self._retarget_if_more_urgent()
            self._move_charging_robot_one_step()
            self._log_state(f"第 {self.tick_count} 轮充电机器人前往目标")
            self._print_battery_dashboard()
            self._sync_gazebo_models()
            return

        if self.charging_robot.target_id is not None:
            self._move_working_robots(excluded_robot_id=self.charging_robot.target_id)
            self._continue_charging()
            self._log_state(f"第 {self.tick_count} 轮充电中")
            self._print_battery_dashboard()
            self._sync_gazebo_models()
            return

        self._move_working_robots()

        target = self._choose_next_target()
        if target is None:
            self.get_logger().info(f"[第 {self.tick_count} 轮] 所有机器人电量充足")
            self._log_state("当前状态")
            self._print_battery_dashboard()
            self._sync_gazebo_models()
            return

        self._dispatch_charging_robot(target)
        self._log_state(f"第 {self.tick_count} 轮派出充电机器人")
        self._print_battery_dashboard()
        self._sync_gazebo_models()

    def _move_working_robots(self, excluded_robot_id: Optional[int] = None) -> None:
        """Move each working robot one grid cell in a random free direction."""
        occupied_cells = {(int(robot.x), int(robot.y)) for robot in self.robots}
        occupied_cells.add((int(self.charging_robot.x), int(self.charging_robot.y)))

        for robot in self.robots:
            if not robot.working or robot.robot_id == excluded_robot_id:
                if robot.robot_id == excluded_robot_id:
                    robot.last_action = "充电中"
                continue

            old_cell = (int(robot.x), int(robot.y))
            occupied_cells.discard(old_cell)
            move_result = self._choose_grid_move(robot, occupied_cells)
            if move_result is None:
                occupied_cells.add(old_cell)
                robot.last_action = "周围被占用，停留"
                continue

            direction_name, next_x, next_y = move_result
            robot.x = float(next_x)
            robot.y = float(next_y)
            occupied_cells.add((next_x, next_y))
            consume = random.uniform(1.0, 2.0)
            robot.battery = max(0.0, robot.battery - consume)
            robot.last_action = f"向{direction_name}移动一格，耗电 {consume:.1f}%"
            if robot.battery <= 0.0:
                robot.working = False
                self.get_logger().warn(f"机器人 R{robot.robot_id} 电量耗尽，停止工作")

    def _choose_next_target(self) -> Optional[RobotState]:
        """Choose the next charging target using urgency and travel distance."""
        candidates = list(self.robots)
        if not candidates:
            return None

        urgent_candidates = [
            robot
            for robot in candidates
            if robot.battery <= self.low_battery_threshold
            or self._estimate_remaining_moves(robot) <= 45.0
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

        dead_robot_bonus = -100.0 if robot.battery <= 0.0 else 0.0
        battery_weight = robot.battery
        time_risk_weight = remaining_moves * 2.0
        distance_weight = distance * 1.5
        return dead_robot_bonus + battery_weight + time_risk_weight + distance_weight

    def _estimate_remaining_moves(self, robot: RobotState) -> float:
        """Estimate how many random moves a robot can still make."""
        average_consumption = 1.5
        return robot.battery / average_consumption

    def _dispatch_charging_robot(self, target: RobotState) -> None:
        """Select a target and start moving the charging robot one grid cell per tick."""
        self.charging_robot.target_id = target.robot_id
        self.charging_robot.moving_to_target = True
        target.last_action = "等待充电机器人"

        self.get_logger().info(
            f"[第 {self.tick_count} 轮] 充电机器人开始前往 R{target.robot_id}，"
            f"目标当前电量 {target.battery:.1f}%"
        )

    def _retarget_if_more_urgent(self) -> None:
        """Switch target during travel if another robot becomes clearly more urgent."""
        current_target = self._get_robot(self.charging_robot.target_id)
        new_target = self._choose_next_target()
        if current_target is None or new_target is None:
            return
        if new_target.robot_id == current_target.robot_id:
            return

        current_score = self._priority_score(current_target)
        new_score = self._priority_score(new_target)
        has_dead_robot = new_target.battery <= 0.0 and current_target.battery > 0.0
        clearly_more_urgent = new_score + 10.0 < current_score
        if has_dead_robot or clearly_more_urgent:
            self.get_logger().warn(
                f"发现更紧急目标：从 R{current_target.robot_id} 改为前往 "
                f"R{new_target.robot_id}"
            )
            current_target.last_action = "继续工作"
            self.charging_robot.target_id = new_target.robot_id
            new_target.last_action = "等待充电机器人"

    def _move_charging_robot_one_step(self) -> None:
        """Move the charging robot one grid cell toward the selected robot."""
        target = self._get_robot(self.charging_robot.target_id)
        if target is None:
            self.charging_robot.target_id = None
            self.charging_robot.moving_to_target = False
            return

        destination = self._choose_charging_cell(target)
        current_cell = (int(self.charging_robot.x), int(self.charging_robot.y))
        if current_cell == destination:
            self._start_charging_at_current_cell(target)
            return

        next_cell = self._choose_charging_step(destination)
        if next_cell == current_cell:
            self.get_logger().warn(
                f"充电机器人到 R{target.robot_id} 的路径被阻挡，等待下一轮"
            )
            return

        self.charging_robot.x = float(next_cell[0])
        self.charging_robot.y = float(next_cell[1])
        self.get_logger().info(
            f"[第 {self.tick_count} 轮] 充电机器人移动一格到 "
            f"({next_cell[0]},{next_cell[1]})，目标 R{target.robot_id}"
        )
        if next_cell == destination:
            self._start_charging_at_current_cell(target)

    def _start_charging_at_current_cell(self, target: RobotState) -> None:
        """Start multi-step charging after the charging robot reaches an adjacent cell."""
        self.charging_robot.moving_to_target = False
        self.charging_robot.target_id = target.robot_id
        self.charging_robot.charging_ticks_remaining = self.charging_duration_ticks
        target.last_action = "充电中"
        self.get_logger().info(
            f"充电机器人到达 R{target.robot_id} 相邻格，开始充电 "
            f"{self.charging_duration_ticks} 轮"
        )
        self._continue_charging()

    def _continue_charging(self) -> None:
        """Charge the current target by one visible charging step."""
        target = self._get_robot(self.charging_robot.target_id)
        if target is None:
            self.charging_robot.target_id = None
            self.charging_robot.charging_ticks_remaining = 0
            return

        before = target.battery
        charge_per_tick = self.charge_amount_per_visit / self.charging_duration_ticks
        target.battery = min(self.full_battery, target.battery + charge_per_tick)
        if target.battery > 0.0:
            target.working = True
        self.charging_robot.charging_ticks_remaining -= 1

        self.get_logger().info(
            f"R{target.robot_id} 充电中：{before:.1f}% -> {target.battery:.1f}% "
            f"({self.charging_robot.charging_ticks_remaining} 轮后结束)"
        )
        if (
            self.charging_robot.charging_ticks_remaining <= 0
            or target.battery >= self.full_battery
        ):
            self.get_logger().info(f"R{target.robot_id} 充电完成")
            target.last_action = "充电完成"
            self.charging_robot.target_id = None
            self.charging_robot.charging_ticks_remaining = 0
            self.charging_robot.moving_to_target = False

    def _log_state(self, title: str) -> None:
        """Print a compact battery and position report for all working robots."""
        robot_text = " | ".join(
            [
                f"R{robot.robot_id}: {robot.battery:5.1f}% "
                f"({int(robot.x)},{int(robot.y)}) {robot.last_action}"
                for robot in self.robots
            ]
        )
        self.get_logger().info(f"{title}: {robot_text}")

    def _print_battery_dashboard(self) -> None:
        """Print a readable real-time dashboard for every robot battery."""
        if not self.show_battery_dashboard:
            return

        lines = [
            "",
            f"========== 第 {self.tick_count} 轮机器人电量 ==========",
            "机器人 | 坐标   | 电量      | 电量条       | 状态",
            "----- | ------ | -------- | ------------ | ----------------",
        ]
        for robot in self.robots:
            filled_count = int(round(robot.battery / 10.0))
            battery_bar = "#" * filled_count + "-" * (10 - filled_count)
            lines.append(
                f"R{robot.robot_id:<4} | "
                f"({int(robot.x)},{int(robot.y)})  | "
                f"{robot.battery:6.1f}% | "
                f"[{battery_bar}] | "
                f"{robot.last_action}"
            )
        if self.charging_robot.target_id is None:
            charging_text = "当前未充电"
        elif self.charging_robot.moving_to_target:
            charging_text = f"正在前往 R{self.charging_robot.target_id}"
        else:
            charging_text = (
                f"正在给 R{self.charging_robot.target_id} 充电，"
                f"剩余 {self.charging_robot.charging_ticks_remaining} 轮"
            )
        lines.append(f"充电机器人：({int(self.charging_robot.x)},{int(self.charging_robot.y)})，{charging_text}")
        lines.append("========================================")
        self.get_logger().info("\n".join(lines))

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
            self._set_gazebo_entity_pose(
                f"robot_label_r{robot.robot_id}", robot.x, robot.y, 0.55
            )
            self._sync_battery_bar(robot)
        self._sync_charging_marker()

    def _sync_battery_bar(self, robot: RobotState) -> None:
        """Show battery level on fixed dashboard blocks from left to right."""
        visible_segments = int(math.ceil(self._clamp(robot.battery, 0.0, 100.0) / 10.0))
        y = 9.0 - (robot.robot_id - 1) * 0.55
        z = 0.18
        active_prefix = (
            "battery_y" if robot.battery <= self.low_battery_threshold else "battery_g"
        )
        inactive_prefix = "battery_g" if active_prefix == "battery_y" else "battery_y"

        for segment in range(1, 11):
            if segment <= visible_segments:
                x = 10.35 + (segment - 1) * 0.14
                self._set_gazebo_entity_pose(
                    f"{active_prefix}_r{robot.robot_id}_seg{segment}", x, y, z
                )
            else:
                self._set_gazebo_entity_pose(
                    f"{active_prefix}_r{robot.robot_id}_seg{segment}", -5.0, -5.0, -2.0
                )
            self._set_gazebo_entity_pose(
                f"{inactive_prefix}_r{robot.robot_id}_seg{segment}", -5.0, -5.0, -2.0
            )

    def _sync_charging_marker(self) -> None:
        """Show or hide the charging marker above the robot being charged."""
        target = self._get_robot(self.charging_robot.target_id)
        if target is None or self.charging_robot.moving_to_target:
            self._set_gazebo_entity_pose("charging_marker", -5.0, -5.0, -2.0)
            return
        self._set_gazebo_entity_pose("charging_marker", target.x, target.y, 0.75)

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
        request.state.twist.linear.x = 0.0
        request.state.twist.linear.y = 0.0
        request.state.twist.linear.z = 0.0
        request.state.twist.angular.x = 0.0
        request.state.twist.angular.y = 0.0
        request.state.twist.angular.z = 0.0
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

    def _find_free_random_cell(self, existing_robots: List[RobotState]) -> tuple[int, int]:
        """Find a random unoccupied grid cell for an extra working robot."""
        occupied = {(int(robot.x), int(robot.y)) for robot in existing_robots}
        while True:
            x = random.randint(1, int(self.work_area_size) - 1)
            y = random.randint(1, int(self.work_area_size) - 1)
            if (x, y) not in occupied:
                return x, y

    def _choose_grid_move(
        self, robot: RobotState, occupied_cells: set[tuple[int, int]]
    ) -> Optional[tuple[str, int, int]]:
        """Choose one valid up/down/left/right grid move without overlap."""
        directions = DIRECTIONS[:]
        random.shuffle(directions)
        for direction_name, dx, dy in directions:
            next_x = int(robot.x) + dx
            next_y = int(robot.y) + dy
            if not self._is_valid_cell(next_x, next_y):
                continue
            if (next_x, next_y) in occupied_cells:
                continue
            return direction_name, next_x, next_y
        return None

    def _choose_charging_cell(self, target: RobotState) -> tuple[int, int]:
        """Choose an adjacent free grid cell for the charging robot."""
        occupied = {
            (int(robot.x), int(robot.y))
            for robot in self.robots
            if robot.robot_id != target.robot_id
        }
        candidates: List[tuple[int, int]] = []
        for _, dx, dy in DIRECTIONS:
            x = int(target.x) + dx
            y = int(target.y) + dy
            if self._is_valid_cell(x, y) and (x, y) not in occupied:
                candidates.append((x, y))
        if not candidates:
            return int(self.charging_robot.x), int(self.charging_robot.y)
        return min(
            candidates,
            key=lambda cell: self._distance(
                self.charging_robot.x, self.charging_robot.y, cell[0], cell[1]
            ),
        )

    def _choose_charging_step(self, destination: tuple[int, int]) -> tuple[int, int]:
        """Choose one collision-free grid step toward the charging destination."""
        current_x = int(self.charging_robot.x)
        current_y = int(self.charging_robot.y)
        occupied = {(int(robot.x), int(robot.y)) for robot in self.robots}

        step_candidates: List[tuple[int, int]] = []
        if destination[0] > current_x:
            step_candidates.append((current_x + 1, current_y))
        elif destination[0] < current_x:
            step_candidates.append((current_x - 1, current_y))
        if destination[1] > current_y:
            step_candidates.append((current_x, current_y + 1))
        elif destination[1] < current_y:
            step_candidates.append((current_x, current_y - 1))

        step_candidates.extend(
            [
                (current_x + dx, current_y + dy)
                for _, dx, dy in DIRECTIONS
                if (current_x + dx, current_y + dy) not in step_candidates
            ]
        )

        for cell in step_candidates:
            if not self._is_valid_cell(cell[0], cell[1]):
                continue
            if cell in occupied and cell != destination:
                continue
            return cell
        return current_x, current_y

    def _get_robot(self, robot_id: Optional[int]) -> Optional[RobotState]:
        """Find a working robot by id."""
        if robot_id is None:
            return None
        for robot in self.robots:
            if robot.robot_id == robot_id:
                return robot
        return None

    def _is_valid_cell(self, x: int, y: int) -> bool:
        """Check whether a grid cell is inside the visible work area."""
        return 1 <= x <= int(self.work_area_size) - 1 and 1 <= y <= int(
            self.work_area_size
        ) - 1


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
