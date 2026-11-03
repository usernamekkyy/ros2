"""trajectory_core 单元测试。

运行：
    cd src/drone_trajectory_logger
    python -m pytest test/test_trajectory_core.py -v

不依赖 ROS / matplotlib，可在任意 Python 3.8+ 环境执行。
"""

import math
import os
import sys

import pytest

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from drone_trajectory_logger import trajectory_core as tc   # noqa: E402


# ---------------------------------------------------------------- 状态识别

def test_hover_state():
    assert tc.classify_motion(0.0, 0.0) == tc.STATE_HOVER
    assert tc.classify_motion(1e-6, -1e-6) == tc.STATE_HOVER


def test_cruise_state():
    assert tc.classify_motion(1.0, 0.0) == tc.STATE_CRUISE


def test_arc_left_regression():
    """回归旧版缺陷：v>0 且 w>0 被误判为直线（旧版走 if linear_x>0 分支）。"""
    assert tc.classify_motion(1.0, 0.5) == tc.STATE_ARC_LEFT


def test_arc_right_regression():
    assert tc.classify_motion(1.0, -0.5) == tc.STATE_ARC_RIGHT


def test_turn_right_negative_regression():
    """回归旧版缺陷：w<0 时 `elif angular.z > 0` 不成立，被吞成悬停。"""
    assert tc.classify_motion(0.0, -1.0) == tc.STATE_TURN_RIGHT
    assert tc.classify_motion(0.0, 1.0) == tc.STATE_TURN_LEFT


def test_reverse_states():
    assert tc.classify_motion(-0.5, 0.0) == tc.STATE_CRUISE
    assert tc.classify_motion(-0.5, 1.0) == tc.STATE_REVERSE_LEFT
    assert tc.classify_motion(-0.5, -1.0) == tc.STATE_REVERSE_RIGHT


def test_all_states_have_labels():
    for state in (tc.STATE_HOVER, tc.STATE_CRUISE, tc.STATE_TURN_LEFT,
                  tc.STATE_TURN_RIGHT, tc.STATE_ARC_LEFT, tc.STATE_ARC_RIGHT,
                  tc.STATE_REVERSE_LEFT, tc.STATE_REVERSE_RIGHT):
        assert state in tc.STATE_LABELS


# ---------------------------------------------------------------- 航迹推算

def test_straight_line_uses_speed_and_dt():
    """1 m/s 走 1 秒 = 1 米；旧版写死 0.5 米/步。"""
    dr = tc.DeadReckoner()
    for _ in range(10):
        dr.step(1.0, 0.0, 0.1)
    assert dr.x == pytest.approx(1.0, abs=1e-9)
    assert dr.y == pytest.approx(0.0, abs=1e-9)
    assert dr.path_length == pytest.approx(1.0, abs=1e-9)


def test_step_length_independent_of_callback_rate():
    """同一段 10 秒飞行，1 Hz 与 20 Hz 推出相同航迹（旧版相差 20 倍）。"""
    slow = tc.DeadReckoner()
    for _ in range(10):
        slow.step(1.0, 0.0, 1.0)

    fast = tc.DeadReckoner()
    for _ in range(200):
        fast.step(1.0, 0.0, 0.05)

    assert slow.x == pytest.approx(10.0, abs=1e-9)
    assert fast.x == pytest.approx(10.0, abs=1e-9)
    assert abs(slow.x - fast.x) < 1e-9


def test_arc_radius_and_quarter_turn():
    """v=2, w=1 -> 半径 2m，左转 90° 后应到 (2, 2)。"""
    dr = tc.DeadReckoner()
    steps = 100
    dt = (math.pi / 2.0) / steps        # 总转角 90°
    for _ in range(steps):
        dr.step(2.0, 1.0, dt)
    assert dr.x == pytest.approx(2.0, abs=1e-6)
    assert dr.y == pytest.approx(2.0, abs=1e-6)
    assert dr.yaw == pytest.approx(math.pi / 2.0, abs=1e-6)


def test_full_circle_returns_to_origin():
    """转满一圈回到起点，里程 = 周长 2*pi*R。"""
    dr = tc.DeadReckoner()
    radius = 3.0
    v, w = radius * 1.0, 1.0
    steps = 720
    dt = (2.0 * math.pi) / steps
    for _ in range(steps):
        dr.step(v, w, dt)
    assert dr.x == pytest.approx(0.0, abs=1e-6)
    assert dr.y == pytest.approx(0.0, abs=1e-6)
    assert dr.path_length == pytest.approx(2.0 * math.pi * radius, rel=1e-6)


def test_arc_right_symmetric():
    dr = tc.DeadReckoner()
    steps = 100
    dt = (math.pi / 2.0) / steps
    for _ in range(steps):
        dr.step(2.0, -1.0, dt)
    assert dr.x == pytest.approx(2.0, abs=1e-6)
    assert dr.y == pytest.approx(-2.0, abs=1e-6)
    assert dr.yaw == pytest.approx(-math.pi / 2.0, abs=1e-6)


def test_dt_clamped_for_stalls():
    """节点卡顿导致 dt 过大时截断，航迹不突跳。"""
    dr = tc.DeadReckoner(dt_max=0.2)
    dr.step(1.0, 0.0, 5.0)
    assert dr.x == pytest.approx(0.2, abs=1e-9)
    assert dr.clamped_steps == 1


def test_invalid_dt_dropped():
    dr = tc.DeadReckoner()
    dr.step(1.0, 0.0, 0.0)
    dr.step(1.0, 0.0, -1.0)
    dr.step(1.0, 0.0, float('nan'))
    dr.step(1.0, 0.0, None)
    assert dr.x == 0.0
    assert dr.dropped_steps == 4


def test_dead_reckoner_reset():
    dr = tc.DeadReckoner()
    dr.step(1.0, 0.0, 1.0)
    dr.reset()
    assert (dr.x, dr.y, dr.yaw, dr.path_length) == (0.0, 0.0, 0.0, 0.0)


# ---------------------------------------------------------------- 几何工具

def test_wrap_to_pi():
    assert tc.wrap_to_pi(0.0) == pytest.approx(0.0, abs=1e-12)
    assert tc.wrap_to_pi(math.pi + 0.1) == pytest.approx(-math.pi + 0.1, abs=1e-12)
    assert tc.wrap_to_pi(-math.pi - 0.1) == pytest.approx(math.pi - 0.1, abs=1e-12)
    # ±pi 边界等价，取 (-pi, pi] 表示
    assert abs(tc.wrap_to_pi(3.0 * math.pi)) == pytest.approx(math.pi, abs=1e-12)
    assert abs(tc.wrap_to_pi(-3.0 * math.pi)) == pytest.approx(math.pi, abs=1e-12)
    assert tc.wrap_to_pi(2.0 * math.pi) == pytest.approx(0.0, abs=1e-12)


@pytest.mark.parametrize('yaw_deg', [0.0, 30.0, 90.0, -90.0, 179.0])
def test_yaw_from_quaternion(yaw_deg):
    yaw = math.radians(yaw_deg)
    qz, qw = math.sin(yaw / 2.0), math.cos(yaw / 2.0)
    assert tc.yaw_from_quaternion(0.0, 0.0, qz, qw) == pytest.approx(yaw, abs=1e-9)


# ---------------------------------------------------------------- 航迹统计

def test_recorder_straight_line_metrics():
    rec = tc.TrajectoryRecorder()
    for i in range(11):
        rec.update(i * 0.1, i * 0.1 * 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, tc.STATE_CRUISE)
    assert rec.path_length == pytest.approx(1.0, abs=1e-9)
    assert rec.displacement == pytest.approx(1.0, abs=1e-9)
    assert rec.straightness == pytest.approx(1.0, abs=1e-9)


def test_recorder_circle_metrics():
    rec = tc.TrajectoryRecorder()
    radius, v, w = 2.0, 2.0, 1.0
    steps = 360
    dt = (2.0 * math.pi) / steps
    x, y, yaw = radius, 0.0, math.pi / 2.0   # 从 (R,0) 起逆时针
    for i in range(steps + 1):
        rec.update(i * dt, x, y, 0.0, yaw, v, w, tc.STATE_ARC_LEFT)
        yaw += w * dt
        x, y = radius * math.cos(yaw - math.pi / 2.0), radius * math.sin(yaw - math.pi / 2.0)
    assert rec.path_length == pytest.approx(2.0 * math.pi * radius, rel=1e-3)
    assert rec.displacement < 0.05          # 闭合
    assert rec.straightness < 0.01


def test_recorder_hover_ratio():
    rec = tc.TrajectoryRecorder()
    # 0~1s 悬停，1~2s 巡航
    for i in range(11):
        state = tc.STATE_HOVER if i <= 10 else tc.STATE_CRUISE
        rec.update(i * 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, state)
    for i in range(11, 21):
        rec.update(i * 0.1, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, tc.STATE_CRUISE)
    assert rec.hover_ratio() == pytest.approx(0.5, abs=0.06)


def test_recorder_csv_roundtrip(tmp_path):
    rec = tc.TrajectoryRecorder()
    for i in range(5):
        rec.update(i * 0.2, i * 0.5, i * 0.1, 1.0, 0.3, 1.2, 0.1, tc.STATE_CRUISE)
    path = tmp_path / 'flight.csv'
    rec.to_csv(str(path))
    back = tc.TrajectoryRecorder.from_csv(str(path))
    assert len(back.rows) == len(rec.rows)
    assert back.path_length == pytest.approx(rec.path_length, rel=1e-4)
    assert back.rows[0][7] == tc.STATE_CRUISE


def test_recorder_rejects_bad_csv_header(tmp_path):
    p = tmp_path / 'bad.csv'
    p.write_text('a,b,c\n1,2,3\n', encoding='utf-8')
    with pytest.raises(ValueError):
        tc.TrajectoryRecorder.from_csv(str(p))


def test_recorder_decimation_keeps_endpoints():
    rec = tc.TrajectoryRecorder(max_points=20)
    for i in range(200):
        rec.update(i * 0.1, i * 0.1, 0.0, 0.0, 0.0, 1.0, 0.0, tc.STATE_CRUISE)
    assert len(rec.rows) <= 40
    assert rec.rows[-1][0] == pytest.approx(19.9, abs=1e-9)


def test_summary_keys():
    rec = tc.TrajectoryRecorder()
    rec.update(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, tc.STATE_HOVER)
    rec.update(0.1, 0.1, 0.0, 0.0, 0.0, 1.0, 0.0, tc.STATE_CRUISE)
    s = rec.summary()
    for key in ('points', 'duration_s', 'path_length_m', 'displacement_m',
                'straightness', 'max_speed_mps', 'mean_speed_mps', 'hover_ratio'):
        assert key in s
    assert s['points'] == 2
