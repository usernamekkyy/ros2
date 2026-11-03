"""复现旧版 trajectory_analyzer 的三个缺陷（仅用标准库，可直接 python 运行）。

旧版核心逻辑（原文照抄）：

    if msg.linear.x > 0:
        x += 0.5 * math.cos(theta)
        y += 0.5 * math.sin(theta)
    elif msg.angular.z > 0:
        theta += 0.2
    else:
        # 悬停等待

运行：
    python src/drone_trajectory_logger/tools/repro_old_bugs.py
"""

import math

# 旧版写死的每步位移 / 每步转角
LEGACY_STEP_M = 0.5
LEGACY_STEP_RAD = 0.2


class LegacyAnalyzer:
    """按旧版分支结构实现的航迹推算（不含 ROS 与绘图）。"""

    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.states = []

    def on_cmd(self, linear_x, angular_z):
        if linear_x > 0:
            self.states.append('cruise')
            self.x += LEGACY_STEP_M * math.cos(self.theta)
            self.y += LEGACY_STEP_M * math.sin(self.theta)
        elif angular_z > 0:
            self.states.append('turn')
            self.theta += LEGACY_STEP_RAD
        else:
            self.states.append('hover')


class TruthModel:
    """同一段指令下无人机的真实运动（独轮车精确积分）。"""

    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

    def on_cmd(self, linear_x, angular_z, dt):
        if abs(angular_z) < 1e-9:
            self.x += linear_x * dt * math.cos(self.theta)
            self.y += linear_x * dt * math.sin(self.theta)
        else:
            r = linear_x / angular_z
            th2 = self.theta + angular_z * dt
            self.x += r * (math.sin(th2) - math.sin(self.theta))
            self.y -= r * (math.cos(th2) - math.cos(self.theta))
            self.theta = th2


def case_step_length():
    """缺陷 1：写死 0.5 m/步，与指令速度、回调周期都无关。"""
    print('=' * 68)
    print('缺陷 1：每步位移写死 0.5 m，忽略线速度与时间间隔')
    print('=' * 68)
    for v, dt, n in ((1.0, 1.0, 10), (1.0, 0.05, 200), (2.0, 1.0, 5)):
        old = LegacyAnalyzer()
        new = TruthModel()
        for _ in range(n):
            old.on_cmd(v, 0.0)
            new.on_cmd(v, 0.0, dt)
        old_d = math.hypot(old.x, old.y)
        new_d = math.hypot(new.x, new.y)
        print('  v=%.1f m/s, dt=%.2fs, n=%3d -> 真实 %6.2f m | 旧版 %6.2f m | '
              '误差 %+6.1f%%' % (v, dt, n, new_d, old_d,
                                 (old_d - new_d) / new_d * 100.0))
    print('  结论：航迹长度只取决于回调次数，与真实速度无关；换 20 Hz 发布时')
    print('        同一段 10 秒飞行的航迹会缩水到真值的 1/20。')


def case_combined_motion():
    """缺陷 2：前进 + 转向被 `if linear_x > 0` 抢先判定为直线。"""
    print()
    print('=' * 68)
    print('缺陷 2：边前进边转向（最常见飞行状态）被判为"直线前进"')
    print('=' * 68)
    old = LegacyAnalyzer()
    new = TruthModel()
    v, w, dt, n = 1.0, 1.0, 1.0, 6
    for _ in range(n):
        old.on_cmd(v, w)
        new.on_cmd(v, w, dt)
    print('  输入：v=%.1f m/s 且 w=%.1f rad/s，持续 %d 步' % (v, w, n))
    print('  旧版识别状态：%s' % old.states[0])
    print('  旧版航向变化：%.3f rad   （真实应为 %.3f rad）' % (old.theta, n * w * dt))
    print('  旧版终点：(%.2f, %.2f)   真实终点：(%.2f, %.2f)'
          % (old.x, old.y, new.x, new.y))
    print('  结论：半径 1 m 的圆弧航迹被画成直线，航向完全不更新。')


def case_negative_turn():
    """缺陷 3：只判 angular_z > 0，右转（负角速度）落到 hover 分支。"""
    print()
    print('=' * 68)
    print('缺陷 3：向右旋转（angular.z < 0）被识别为"悬停等待"')
    print('=' * 68)
    old = LegacyAnalyzer()
    for _ in range(5):
        old.on_cmd(0.0, -1.0)
    print('  输入：w=-1.0 rad/s（右转）')
    print('  旧版识别状态：%s' % old.states[0])
    print('  旧版航向变化：%.3f rad' % old.theta)
    print('  结论：右转既不转向也不报错，航迹与姿态直接失真。')


def case_cmd_vel_is_not_pose():
    """缺陷 4：用指令积分代替真实位姿。"""
    print()
    print('=' * 68)
    print('缺陷 4：把 cmd_vel（期望速度）当作位姿来源')
    print('=' * 68)
    print('  指令 v=1.0 m/s 持续 10 s，但无人机受风扰/限速只飞了 7.5 m：')
    print('    指令积分得到 10.00 m，实际位移 7.50 m，航迹偏差 +33%。')
    print('  正确做法：位姿来自 /drone/odom（nav_msgs/Odometry），')
    print('            cmd_vel 只用于前向推演/丢帧时的兜底，且必须带 dt。')


if __name__ == '__main__':
    case_step_length()
    case_combined_motion()
    case_negative_turn()
    case_cmd_vel_is_not_pose()
