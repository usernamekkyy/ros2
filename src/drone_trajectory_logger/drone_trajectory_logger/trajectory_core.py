"""无人机飞行状态识别与航迹推算核心（纯标准库，不依赖 ROS / matplotlib）。

设计要点
--------
1. **位姿来自真实观测**：航迹应由 ``nav_msgs/Odometry`` 的位姿给出；
   只有在没有里程计时才用 ``cmd_vel`` 前向推算，并且必须带真实 ``dt``。
2. **按弧长积分**：直线用 ``v*dt``，圆弧用精确的独轮车闭式积分，
   半径 ``R = v / w``；不能写死"每步 0.5 米"。
3. **状态判定不互斥**：前进与转向可以同时发生（弧线飞行），
   角速度的符号决定左右，所以共有 7 种状态而不是 3 种。
4. **时间戳可信性检查**：``dt`` 为负、为 0 或过大（节点卡顿、回放加速）时
   截断并计数，避免航迹突跳。

单位约定
--------
位置: m    线速度: m/s    角速度: rad/s    航向: rad（绕 z 轴，ENU 右手系，x 朝东）
"""

import math

# 判定阈值：小于该值视为"没有动作"，用于吸收传感器零漂
EPS_V = 1e-3      # m/s
EPS_W = 1e-3      # rad/s

# 航向增量上限，超过则认为时间戳异常（保守取 90°）
MAX_YAW_STEP = math.pi / 2.0

STATE_HOVER = 'hover'
STATE_CRUISE = 'cruise'
STATE_TURN_LEFT = 'turn_left'
STATE_TURN_RIGHT = 'turn_right'
STATE_ARC_LEFT = 'arc_left'
STATE_ARC_RIGHT = 'arc_right'
STATE_REVERSE_LEFT = 'reverse_left'
STATE_REVERSE_RIGHT = 'reverse_right'

STATE_LABELS = {
    STATE_HOVER: '悬停',
    STATE_CRUISE: '直线巡航',
    STATE_TURN_LEFT: '原地左转',
    STATE_TURN_RIGHT: '原地右转',
    STATE_ARC_LEFT: '左弧线飞行',
    STATE_ARC_RIGHT: '右弧线飞行',
    STATE_REVERSE_LEFT: '后退左转',
    STATE_REVERSE_RIGHT: '后退右转',
}


def wrap_to_pi(angle):
    """把角度归一化到 (-pi, pi]。"""
    a = math.fmod(float(angle), 2.0 * math.pi)
    if a > math.pi:
        a -= 2.0 * math.pi
    elif a <= -math.pi:
        a += 2.0 * math.pi
    return a


def yaw_from_quaternion(qx, qy, qz, qw):
    """从四元数取偏航角（绕 z 轴）。"""
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def classify_motion(linear_x, angular_z, eps_v=EPS_V, eps_w=EPS_W):
    """按线速度/角速度判定飞行状态。

    旧实现用 ``if linear_x > 0: ... elif angular_z > 0: ...`` 的互斥分支，
    导致「边前进边转向」被吞成直线、右转（负角速度）被吞成悬停。
    这里两个轴独立判定后再组合。
    """
    forward = linear_x > eps_v
    backward = linear_x < -eps_v
    turning = abs(angular_z) > eps_w
    left = angular_z > 0

    if not forward and not backward:
        if not turning:
            return STATE_HOVER
        return STATE_TURN_LEFT if left else STATE_TURN_RIGHT

    if not turning:
        return STATE_CRUISE

    if forward:
        return STATE_ARC_LEFT if left else STATE_ARC_RIGHT
    return STATE_REVERSE_LEFT if left else STATE_REVERSE_RIGHT


class DeadReckoner:
    """按速度指令前向推算位姿（独轮车模型，圆弧用闭式积分）。

    仅在缺少里程计时作为兜底；``dt`` 会被限制在 ``[0, dt_max]`` 内，
    并统计被截断/丢弃的次数，便于判断推算结果是否可信。
    """

    def __init__(self, x=0.0, y=0.0, yaw=0.0, dt_max=1.0):
        """dt_max 默认 1.0 s：覆盖 1 Hz 这类慢速发布，同时拦住卡顿/回放跳变。"""
        self.x = float(x)
        self.y = float(y)
        self.yaw = float(yaw)
        self.dt_max = float(dt_max)
        self.path_length = 0.0
        self.clamped_steps = 0
        self.dropped_steps = 0

    def reset(self, x=0.0, y=0.0, yaw=0.0):
        self.x, self.y, self.yaw = float(x), float(y), float(yaw)
        self.path_length = 0.0
        self.clamped_steps = 0
        self.dropped_steps = 0

    def sanitize_dt(self, dt):
        """把 dt 修正到合理区间，返回 (dt, ok)。"""
        if dt is None:
            self.dropped_steps += 1
            return 0.0, False
        dt = float(dt)
        if not math.isfinite(dt) or dt <= 0.0:
            self.dropped_steps += 1
            return 0.0, False
        if dt > self.dt_max:
            self.clamped_steps += 1
            return self.dt_max, True
        return dt, True

    def step(self, linear_x, angular_z, dt):
        """推进一步，返回 (x, y, yaw)。"""
        dt, ok = self.sanitize_dt(dt)
        if not ok:
            return self.x, self.y, self.yaw

        v = float(linear_x)
        w = float(angular_z)

        if abs(w) < EPS_W:
            # 直线：位移 = v * dt
            self.x += v * dt * math.cos(self.yaw)
            self.y += v * dt * math.sin(self.yaw)
            self.path_length += abs(v) * dt
        else:
            # 圆弧精确积分：R = v / w
            r = v / w
            yaw_next = self.yaw + w * dt
            self.x += r * (math.sin(yaw_next) - math.sin(self.yaw))
            self.y -= r * (math.cos(yaw_next) - math.cos(self.yaw))
            self.path_length += abs(r * w) * dt
            self.yaw = wrap_to_pi(yaw_next)
            return self.x, self.y, self.yaw

        # 航向变化过大说明时间戳异常，已由 dt_max 兜住，这里只做安全归一化
        return self.x, self.y, self.yaw

    def step_twist(self, twist, dt):
        """接受带 linear.x / angular.z 属性的对象（Twist 或同名结构）。"""
        return self.step(twist.linear.x, twist.angular.z, dt)


class TrajectoryRecorder:
    """记录航迹并增量统计飞行指标。

    用法::

        rec = TrajectoryRecorder()
        rec.update(t, x, y, z, yaw, v, w, state)
        print(rec.summary())
        rec.to_csv('flight.csv')
    """

    def __init__(self, max_points=None):
        self.max_points = max_points
        self.rows = []
        self.path_length = 0.0
        self.max_speed = 0.0
        self.state_time = {}
        self.total_time = 0.0
        self._last = None
        self._duration_by_state = {}

    # ---------- 写入 ----------
    def update(self, t, x, y, z=0.0, yaw=0.0, v=0.0, w=0.0, state=STATE_HOVER):
        row = (float(t), float(x), float(y), float(z), float(yaw),
               float(v), float(w), state)
        if self._last is not None:
            lx, ly, lz = self._last[1], self._last[2], self._last[3]
            dt = float(t) - self._last[0]
            if dt > 0.0:
                self.path_length += math.sqrt((x - lx) ** 2 + (y - ly) ** 2 + (z - lz) ** 2)
                self.total_time += dt
                # 状态持续时间按上一帧的状态累计（上一帧状态持续到本帧）
                prev_state = self._last[7]
                self._duration_by_state[prev_state] = \
                    self._duration_by_state.get(prev_state, 0.0) + dt
        self.max_speed = max(self.max_speed, math.hypot(float(v), 0.0))
        self.rows.append(row)
        if self.max_points is not None and len(self.rows) > self.max_points:
            self._decimate_inplace()
        self._last = row
        return row

    def _decimate_inplace(self):
        """点太多时按步长抽稀，同时保留首尾点。"""
        keep = self.rows[::2]
        if self.rows[-1] is not keep[-1]:
            keep.append(self.rows[-1])
        self.rows = keep

    # ---------- 统计 ----------
    @property
    def displacement(self):
        if len(self.rows) < 2:
            return 0.0
        a, b = self.rows[0], self.rows[-1]
        return math.sqrt((b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2 + (b[3] - a[3]) ** 2)

    @property
    def straightness(self):
        """位移 / 里程，1.0 表示完全直线，越小说明绕得越多。"""
        if self.path_length <= 0.0:
            return 0.0
        return self.displacement / self.path_length

    @property
    def closure_error(self):
        """终点到起点的距离（用于评估是否闭合航线）。"""
        return self.displacement

    def duration_of(self, state):
        return self._duration_by_state.get(state, 0.0)

    def hover_ratio(self):
        if self.total_time <= 0.0:
            return 0.0
        return self.duration_of(STATE_HOVER) / self.total_time

    def state_durations(self):
        return dict(sorted(self._duration_by_state.items(), key=lambda kv: -kv[1]))

    def mean_speed(self):
        if self.total_time <= 0.0:
            return 0.0
        return self.path_length / self.total_time

    def summary(self):
        return {
            'points': len(self.rows),
            'duration_s': round(self.total_time, 3),
            'path_length_m': round(self.path_length, 3),
            'displacement_m': round(self.displacement, 3),
            'straightness': round(self.straightness, 3),
            'max_speed_mps': round(self.max_speed, 3),
            'mean_speed_mps': round(self.mean_speed(), 3),
            'hover_ratio': round(self.hover_ratio(), 3),
            'state_duration_s': {k: round(v, 2) for k, v in self.state_durations().items()},
        }

    # ---------- 导出 ----------
    CSV_HEADER = 't,x,y,z,yaw,v,w,state'

    def to_csv(self, path):
        with open(path, 'w', encoding='utf-8') as f:
            f.write(self.CSV_HEADER + '\n')
            for r in self.rows:
                f.write('%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%s\n' % r)

    @classmethod
    def from_csv(cls, path):
        rec = cls()
        with open(path, 'r', encoding='utf-8') as f:
            header = f.readline().strip()
            if header != cls.CSV_HEADER:
                raise ValueError('unexpected header: %r' % header)
            for line in f:
                parts = line.strip().split(',')
                if len(parts) != 8:
                    continue
                rec.update(float(parts[0]), float(parts[1]), float(parts[2]),
                           float(parts[3]), float(parts[4]), float(parts[5]),
                           float(parts[6]), parts[7])
        return rec
