"""仿真数据源：发布 /drone/cmd_vel 指令与 /drone/odom 真实里程计。

旧版本只发布 1 Hz 的 ``Twist`` 指令，没有里程计，所以下游只能用指令积分
猜位姿。这里补上两个关键事实：

1. 指令是 **期望值**，与真实运动之间存在一阶滞后和噪声——用指令积分
   得到的航迹必然与真实航迹有偏差；
2. 位姿应该由 ``nav_msgs/Odometry`` 直接给出。

任务剖面（默认 20 Hz 发布）::

    0~2s   悬停
    2~8s   直线巡航 1.5 m/s
    8~14s  左弧线 1.5 m/s + 0.5 rad/s（半径 3 m）
    14~18s 直线巡航
    18~21s 原地右转 -1.0 rad/s
    21~23s 悬停
"""

import argparse
import math
import random

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from .trajectory_core import DeadReckoner, classify_motion, wrap_to_pi


# (持续时间 s, 线速度 m/s, 角速度 rad/s)
MISSION = [
    (2.0, 0.0, 0.0),
    (6.0, 1.5, 0.0),
    (6.0, 1.5, 0.5),
    (4.0, 1.5, 0.0),
    (3.0, 0.0, -1.0),
    (2.0, 0.0, 0.0),
]


class DroneSimPublisher(Node):
    """按任务剖面发布指令，并以一阶滞后模型生成"真实"里程计。"""

    def __init__(self, rate_hz=20.0, tau=0.25, noise=0.002, loop=True):
        super().__init__('trajectory_publisher')
        self.rate_hz = float(rate_hz)
        self.dt = 1.0 / self.rate_hz
        self.tau = float(tau)
        self.noise = float(noise)
        self.loop = bool(loop)

        self.pub_cmd = self.create_publisher(Twist, '/drone/cmd_vel', 10)
        self.pub_odom = self.create_publisher(Odometry, '/drone/odom', 10)

        self.elapsed = 0.0
        self.index = 0
        self.seg_left = MISSION[0][0]

        # 真实运动状态（带滞后）
        self.v = 0.0
        self.w = 0.0
        self.reckoner = DeadReckoner()

        self.rng = random.Random(20261103)
        self.timer = self.create_timer(self.dt, self.on_timer)
        self.get_logger().info(
            '仿真发布节点已启动：%.0f Hz，指令->运动滞后 tau=%.2fs' % (self.rate_hz, self.tau))

    def _current_cmd(self):
        return MISSION[self.index][1], MISSION[self.index][2]

    def _advance_mission(self):
        self.seg_left -= self.dt
        if self.seg_left > 1e-9:
            return
        self.index += 1
        if self.index >= len(MISSION):
            if not self.loop:
                self.finished = True
                self.timer.cancel()
                self.get_logger().info('任务剖面结束，发布端停止。')
                return
            self.index = 0
            self.reckoner.reset()
            self.elapsed = 0.0
        self.seg_left = MISSION[self.index][0]
        self.get_logger().info('切换任务段 -> v=%.1f w=%.1f'
                               % (MISSION[self.index][1], MISSION[self.index][2]))

    def on_timer(self):
        if getattr(self, 'finished', False):
            return
        v_cmd, w_cmd = self._current_cmd()

        # 一阶滞后：真实速度缓慢跟随指令，模拟响应延迟与执行器惯性
        alpha = min(1.0, self.dt / max(1e-6, self.tau))
        self.v += (v_cmd - self.v) * alpha
        self.w += (w_cmd - self.w) * alpha

        cmd = Twist()
        cmd.linear.x = v_cmd
        cmd.angular.z = w_cmd
        self.pub_cmd.publish(cmd)

        x, y, yaw = self.reckoner.step(self.v, self.w, self.dt)

        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'map'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = x + self.rng.gauss(0.0, self.noise)
        odom.pose.pose.position.y = y + self.rng.gauss(0.0, self.noise)
        odom.pose.pose.position.z = 0.0
        half = wrap_to_pi(yaw) / 2.0
        odom.pose.pose.orientation.z = math.sin(half)
        odom.pose.pose.orientation.w = math.cos(half)
        odom.twist.twist.linear.x = self.v
        odom.twist.twist.angular.z = self.w
        self.pub_odom.publish(odom)

        self.elapsed += self.dt
        self._advance_mission()

        if int(self.elapsed * 2) != int((self.elapsed - self.dt) * 2):
            state = classify_motion(self.v, self.w)
            self.get_logger().info('t=%5.1fs 指令 v=%.1f w=%.1f | 实际 v=%.2f w=%.2f | %s'
                                   % (self.elapsed, v_cmd, w_cmd, self.v, self.w, state))


def main(args=None):
    parser = argparse.ArgumentParser(description='无人机航迹仿真数据源')
    parser.add_argument('--rate', type=float, default=20.0, help='发布频率 Hz')
    parser.add_argument('--tau', type=float, default=0.25, help='指令->运动一阶滞后时间常数')
    parser.add_argument('--noise', type=float, default=0.002, help='里程计位置噪声标准差 m')
    parser.add_argument('--once', action='store_true', help='任务剖面只跑一遍就退出')
    known, _ = parser.parse_known_args(args)

    rclpy.init(args=args)
    node = DroneSimPublisher(rate_hz=known.rate, tau=known.tau,
                             noise=known.noise, loop=not known.once)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
