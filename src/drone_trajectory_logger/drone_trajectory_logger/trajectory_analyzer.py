"""航迹识别与记录节点。

数据来源优先级
--------------
1. ``/drone/odom``（``nav_msgs/Odometry``）——真实位姿，直接采信；
2. 若超过 ``odom_timeout`` 秒收不到里程计，退化为用 ``/drone/cmd_vel``
   按 **真实 dt** 做前向推算，并在日志/CSV 中标记 ``source=dead_reckoning``。

与原实现的区别
--------------
* 不再把"每步 0.5 m"写死，位移由 ``v * dt`` 或圆弧积分给出；
* 状态判定不再互斥，能识别弧线飞行与右转；
* 航迹写入 CSV，退出时打印里程/位移/直线度/悬停占比等指标；
* 绘图降频（默认 5 Hz）且可关闭，避免在回调里阻塞 spin 线程。
"""

import argparse
import collections
import math
import os
import time

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from .trajectory_core import (
    DeadReckoner,
    STATE_LABELS,
    TrajectoryRecorder,
    classify_motion,
    wrap_to_pi,
    yaw_from_quaternion,
)


class TrajectoryAnalyzer(Node):
    def __init__(self, csv_path=None, plot=True, odom_timeout=1.0, plot_hz=5.0,
                 exit_after=None):
        super().__init__('trajectory_analyzer')
        self.csv_path = csv_path
        self.plot_enabled = plot
        self.odom_timeout = odom_timeout
        self.plot_interval = 1.0 / max(1e-3, plot_hz)

        self.create_subscription(Odometry, '/drone/odom', self.on_odom, 20)
        self.create_subscription(Twist, '/drone/cmd_vel', self.on_cmd, 20)

        self.recorder = TrajectoryRecorder()
        self.reckoner = DeadReckoner()
        self.t0 = time.monotonic()
        self.last_odom_time = None
        self.last_cmd = (0.0, 0.0)
        self.last_cmd_time = None
        self.last_pose_time = None
        self.source = 'waiting'
        self.odom_frames = 0
        self.dead_reckoned_frames = 0

        self._init_plot()

        if exit_after:
            self.create_timer(float(exit_after), self._auto_exit)

        self.get_logger().info('轨迹识别节点已启动，等待 /drone/odom 或 /drone/cmd_vel ...')

    def _auto_exit(self):
        self.get_logger().info('到达预设运行时长，自动退出并输出统计。')
        raise SystemExit(0)

    # ------------------------------------------------------------------ 绘图
    def _init_plot(self):
        self.fig = None
        self.ax = None
        self.line = None
        self.arrow = None
        self._last_plot = 0.0
        if not self.plot_enabled:
            return
        import matplotlib
        if not os.environ.get('DISPLAY') and os.name != 'nt':
            matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        plt.rcParams['font.sans-serif'] = ['Microsoft YaHei', 'SimHei', 'DejaVu Sans']
        plt.rcParams['axes.unicode_minus'] = False
        self.plt = plt
        self.fig, self.ax = plt.subplots(figsize=(7, 5))
        self.line, = self.ax.plot([], [], 'b-', lw=1.6, label='飞行航迹')
        self.ax.set_xlabel('X / m')
        self.ax.set_ylabel('Y / m')
        self.ax.set_title('无人机飞行航迹（来源：里程计）')
        self.ax.grid(True, ls=':')
        self.ax.legend(loc='best')
        self.ax.set_aspect('equal', adjustable='datalim')
        plt.pause(0.01)

    def _redraw(self, force=False):
        if not self.plot_enabled or self.fig is None:
            return
        now = time.monotonic()
        if not force and now - self._last_plot < self.plot_interval:
            return
        self._last_plot = now
        xs = [r[1] for r in self.recorder.rows]
        ys = [r[2] for r in self.recorder.rows]
        self.line.set_data(xs, ys)
        self.ax.relim()
        self.ax.autoscale_view()
        self.ax.set_title('无人机飞行航迹（点数 %d，来源：%s）'
                          % (len(xs), self.source_label()))
        try:
            self.fig.canvas.draw_idle()
            self.fig.canvas.flush_events()
        except Exception:       # 无界面环境下忽略绘图错误，不影响记录
            self.plot_enabled = False

    # ------------------------------------------------------------------ 回调
    def source_label(self):
        return {'odom': '里程计', 'dead_reckoning': '指令推算（降级）'}.get(self.source, '等待数据')

    def _now(self):
        return time.monotonic() - self.t0

    def on_cmd(self, msg):
        self.last_cmd = (msg.linear.x, msg.angular.z)
        self.last_cmd_time = self._now()

    def on_odom(self, msg):
        t = self._now()
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = yaw_from_quaternion(q.x, q.y, q.z, q.w)

        # 状态优先用里程计的实测速度；没有 twist 时退回指令
        v = msg.twist.twist.linear.x
        w = msg.twist.twist.angular.z
        if abs(v) < 1e-6 and abs(w) < 1e-6:
            v, w = self.last_cmd
        state = classify_motion(v, w)

        self.recorder.update(t, p.x, p.y, p.z, yaw, v, w, state)

        # 同步推算器，保证降级时位置连续
        self.reckoner.x, self.reckoner.y, self.reckoner.yaw = p.x, p.y, yaw

        self.source = 'odom'
        self.last_odom_time = time.monotonic()
        self.odom_frames += 1
        self._log_if_switched(state)
        self._redraw()

    def on_cmd_fallback(self):
        """无里程计时按指令推进（由定时器以固定频率调用）。"""
        if self.last_odom_time is not None and \
                time.monotonic() - self.last_odom_time < self.odom_timeout:
            return
        if self.last_cmd_time is None:
            return

        now = self._now()
        dt = now - (self.last_pose_time if self.last_pose_time is not None else now)
        if dt <= 0.0:
            return

        v, w = self.last_cmd
        x, y, yaw = self.reckoner.step(v, w, dt)
        self.recorder.update(now, x, y, 0.0, yaw, v, w, classify_motion(v, w))
        self.last_pose_time = now
        self.source = 'dead_reckoning'
        self.dead_reckoned_frames += 1
        self._redraw()

    def _log_if_switched(self, state):
        prev = getattr(self, '_last_state', None)
        if prev != state:
            self._last_state = state
            self.get_logger().info('t=%5.2fs 状态切换 -> %s（%s）'
                                   % (self._now(), state, STATE_LABELS.get(state, state)))

    # ------------------------------------------------------------------ 收尾
    def finish(self):
        self._redraw(force=True)
        summary = self.recorder.summary()
        self.get_logger().info('=' * 56)
        self.get_logger().info('架次统计（数据来源：%s）' % self.source_label())
        self.get_logger().info('  记录点数      : %d（里程计 %d 帧 / 推算 %d 帧）'
                               % (summary['points'], self.odom_frames, self.dead_reckoned_frames))
        self.get_logger().info('  飞行时长      : %.2f s' % summary['duration_s'])
        self.get_logger().info('  航迹里程      : %.2f m' % summary['path_length_m'])
        self.get_logger().info('  起点-终点位移 : %.2f m' % summary['displacement_m'])
        self.get_logger().info('  直线度        : %.3f' % summary['straightness'])
        self.get_logger().info('  最大速度      : %.2f m/s' % summary['max_speed_mps'])
        self.get_logger().info('  平均速度      : %.2f m/s' % summary['mean_speed_mps'])
        self.get_logger().info('  悬停占比      : %.1f%%' % (summary['hover_ratio'] * 100.0))
        for state, dur in summary['state_duration_s'].items():
            self.get_logger().info('    %-14s %5.2f s' % (state, dur))
        self.get_logger().info('=' * 56)

        if self.csv_path:
            self.recorder.to_csv(self.csv_path)
            self.get_logger().info('航迹已写入 %s' % os.path.abspath(self.csv_path))


def main(args=None):
    parser = argparse.ArgumentParser(description='无人机航迹识别与记录')
    parser.add_argument('--csv', default='flight_trajectory.csv', help='航迹 CSV 输出路径')
    parser.add_argument('--no-plot', action='store_true', help='不弹绘图窗口')
    parser.add_argument('--odom-timeout', type=float, default=1.0,
                        help='超过该时长无里程计则降级为指令推算')
    parser.add_argument('--exit-after', type=float, default=None,
                        help='运行指定秒数后自动退出并打印统计')
    known, _ = parser.parse_known_args(args)

    rclpy.init(args=args)
    node = TrajectoryAnalyzer(csv_path=known.csv, plot=not known.no_plot,
                              odom_timeout=known.odom_timeout,
                              exit_after=known.exit_after)
    timer = node.create_timer(0.05, node.on_cmd_fallback)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.finish()
        node.destroy_timer(timer)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
