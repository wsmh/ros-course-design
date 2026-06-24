# 机器人充电调度课设

本项目使用 Ubuntu + ROS2 + Python 实现“一个充电机器人给多个工作机器人充电”的仿真。

## 题目要求对应关系

- 机器人数量大于 5 个：默认创建 6 个工作机器人。
- 工作机器人持续执行任务：每一轮都会随机移动。
- 每次移动随机消耗 1%-2% 电量：在 `_move_working_robots()` 中实现。
- 充电机器人移动不耗电：充电机器人只更新坐标，不维护自身电量。
- 根据剩余电量计划充电顺序：在 `_choose_next_target()` 和 `_priority_score()` 中实现。

## 调度策略

每一轮仿真执行以下步骤：

1. 所有工作机器人随机移动，并随机消耗 1%-2% 电量。
2. 筛选需要充电的机器人：
   - 电量低于阈值，默认 30%；
   - 或者预计剩余可移动次数小于等于 15 次。
3. 对候选机器人计算优先级分数：

```text
priority = 当前电量 + 预计剩余移动次数 * 2.0 + 充电机器人到目标距离 * 1.5
```

分数越低，说明越紧急，越优先充电。这个策略同时考虑了低电量风险和充电机器人移动距离。

## 在 Ubuntu ROS2 中运行

假设你已经安装 ROS2 Humble，并把本目录复制到 Ubuntu 中。

```bash
cd ros2_charge_ws
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
ros2 run robot_charging_scheduler charging_scheduler
```

停止程序：

```bash
Ctrl + C
```

## 可调参数

运行时可以通过 ROS2 参数调整：

```bash
ros2 run robot_charging_scheduler charging_scheduler --ros-args \
  -p robot_count:=8 \
  -p low_battery_threshold:=35.0 \
  -p charge_amount_per_visit:=50.0 \
  -p random_seed:=3
```

参数说明：

- `robot_count`：工作机器人数量，必须大于 5。
- `low_battery_threshold`：低电量阈值。
- `full_battery`：满电电量，默认 100。
- `charge_amount_per_visit`：每次服务给目标机器人补充的电量。
- `work_area_size`：二维工作区域边长。
- `timer_period`：仿真周期，单位秒。
- `random_seed`：随机种子，方便复现实验结果。

## 主要文件

- `src/robot_charging_scheduler/robot_charging_scheduler/charging_scheduler_node.py`：核心仿真与调度算法。
- `src/robot_charging_scheduler/package.xml`：ROS2 包描述。
- `src/robot_charging_scheduler/setup.py`：Python 包入口配置。
