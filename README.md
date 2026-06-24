# 机器人充电调度课设

本项目使用 Ubuntu + ROS2 + Python + Gazebo 实现“一个充电机器人给多个工作机器人充电”的图形仿真。

## 题目要求对应关系

- 机器人数量大于 5 个：默认创建 6 个工作机器人。
- 工作机器人持续执行任务：每一轮都会在网格上随机选择上下左右方向移动一格。
- 每次移动随机消耗 1%-2% 电量：只有成功移动一格才扣电。
- 充电机器人移动不耗电：充电机器人只更新坐标，不维护自身电量。
- 根据剩余电量计划充电顺序：在 `_choose_next_target()` 和 `_priority_score()` 中实现。
- Gazebo 图形仿真：蓝色圆柱表示工作机器人，橙红色圆柱表示充电机器人。
- 防重叠：每个网格同一时刻最多允许一个机器人占用。
- 充电过程：默认持续 5 轮，目标机器人旁边会出现黄色充电标识。

## 调度策略

每一轮仿真执行以下步骤：

1. 所有工作机器人在网格上随机选择上、下、左、右方向移动一格，并随机消耗 1%-2% 电量。
2. 筛选需要充电的机器人：
   - 电量低于阈值，默认 30%；
   - 或者预计剩余可移动次数小于等于 15 次。
3. 对候选机器人计算优先级分数：

```text
priority = 当前电量 + 预计剩余移动次数 * 2.0 + 充电机器人到目标距离 * 1.5
```

分数越低，说明越紧急，越优先充电。这个策略同时考虑了低电量风险和充电机器人移动距离。

## 安装依赖

假设你使用 ROS2 Humble：

```bash
sudo apt update
sudo apt install ros-humble-gazebo-ros-pkgs python3-colcon-common-extensions
```

如果你使用其他 ROS2 版本，把 `humble` 替换成你的版本名。

## 运行 Gazebo 图形仿真

在本工作区根目录运行：

```bash
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
ros2 launch robot_charging_scheduler charging_gazebo.launch.py
```

如果 Gazebo 弹窗黑屏，先清理后重新构建：

```bash
rm -rf build install log
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
ros2 launch robot_charging_scheduler charging_gazebo.launch.py
```

如果仍然黑屏，手动启用软件渲染后再运行：

```bash
export LIBGL_ALWAYS_SOFTWARE=1
export QT_X11_NO_MITSHM=1
ros2 launch robot_charging_scheduler charging_gazebo.launch.py
```

运行后会打开 Gazebo：

- 蓝色机器人 `working_robot_1` 到 `working_robot_6` 会在 10x10 区域中随机移动。
- 地面灰色网格表示地图格子，机器人每轮只会上下左右移动一格。
- 机器人每次成功移动一格消耗 1%-2% 电量，不能走出边界，也不能走到已被占用的格子。
- 每个蓝色机器人上方有 10 格电量条，每格约代表 10% 电量，移动耗电时格数减少，充电时格数增加。
- 电量高于低电量阈值时电量条为绿色，低于阈值时电量条变为黄色。
- 终端会实时显示每个机器人的当前电量百分比，例如 `R1: 83.2%`，并带有文字电量条。
- 橙红色 `charging_robot` 会每轮沿网格移动一格，逐步前往当前最需要充电的机器人相邻格，不会瞬移。
- 充电时目标机器人上方会出现黄色充电标识，默认持续 5 轮。
- 终端会输出每轮电量、充电目标、移动距离和充电前后电量。

如果 Gazebo 中看不到机器人，或者只看到一个物体，通常是旧的 world/launch 文件仍在 `install/` 中。执行下面命令清理重建：

```bash
rm -rf build install log
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
ros2 launch robot_charging_scheduler charging_gazebo.launch.py
```

当前版本已经把 6 个工作机器人和 1 个充电机器人直接写入 world 文件，不再依赖 `spawn_entity.py` 动态插入模型。Gazebo 打开后，即使调度节点还没有启动，也应该能看到 6 个蓝色机器人和 1 个橙色机器人。

如果终端一直输出 `等待 Gazebo /set_entity_state 服务就绪`，打开另一个终端检查 Gazebo 状态服务：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 service list | grep entity_state
```

正常应能看到 `/set_entity_state`。如果你的环境显示的是 `/gazebo/set_entity_state`，可以这样启动：

```bash
ros2 launch robot_charging_scheduler charging_gazebo.launch.py set_entity_state_service:=/gazebo/set_entity_state
```

停止程序：

```bash
Ctrl + C
```

## 运行控制台仿真

假设你已经安装 ROS2 Humble，并把本目录复制到 Ubuntu 中。

```bash
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
- `use_gazebo`：是否同步 Gazebo 模型位置，launch 文件中默认开启。
- `charging_duration_ticks`：一次充电持续多少轮，默认 5。
- `show_battery_dashboard`：是否在终端显示实时电量表，默认开启。

## 主要文件

- `src/robot_charging_scheduler/robot_charging_scheduler/charging_scheduler_node.py`：核心仿真与调度算法。
- `src/robot_charging_scheduler/launch/charging_gazebo.launch.py`：Gazebo 图形仿真启动文件。
- `src/robot_charging_scheduler/worlds/charging_world.world`：Gazebo 世界文件。
- `src/robot_charging_scheduler/models/`：Gazebo 机器人模型。
- `src/robot_charging_scheduler/package.xml`：ROS2 包描述。
- `src/robot_charging_scheduler/setup.py`：Python 包入口配置。
