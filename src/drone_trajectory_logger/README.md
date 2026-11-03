# 无人机航迹推算与飞行状态识别 ROS 2 模块（修复版）

本模块基于 ROS 2 实现无人机飞行状态识别与航迹记录。本版本修复了原版的
多个正确性缺陷（详见 `docs/air/drone_trajectory_logger.md`）：

- 不再把"每步 0.5 m"写死，位移由 `v·dt` / 圆弧精确积分给出；
- 状态判定不再互斥，可识别 8 种状态（含弧线飞行与右转）；
- 位姿优先来自 `/drone/odom`（`nav_msgs/Odometry`），仅超时后降级为
  指令推算并标记来源；
- 航迹写入 CSV，退出时输出里程/位移/直线度/悬停占比等架次统计；
- 纯逻辑核心 `trajectory_core.py` 不依赖 ROS，28 项单元测试全部通过。

## 运行环境

* ROS 2 Humble（Ubuntu 22.04 或 RoboStack conda）
* 依赖：rclpy, geometry_msgs, nav_msgs, matplotlib

## 快速开始

```bash
# 方式一：colcon
colcon build --packages-select drone_trajectory_logger
source install/setup.bash
ros2 launch drone_trajectory_logger main.launch.py

# 方式二：模块直跑（conda 环境）
cd src/drone_trajectory_logger
python -m drone_trajectory_logger.main --once
python -m drone_trajectory_logger.trajectory_analyzer --no-plot --exit-after 24.5

# 方式三：无 ROS 离线演示
python tools/offline_demo.py
python tools/repro_old_bugs.py

# 单元测试
python -m pytest test/test_trajectory_core.py -v
```

详细原理、验证数据与配图见
[主文档](../../docs/air/drone_trajectory_logger.md)。

## 作业信息

- 姓名：黄家欣
- 学号：2535030010
