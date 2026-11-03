"""离线演示：同一条任务剖面下，旧版/新版/真实位姿三种航迹的差异。

不需要 ROS，直接运行：

    python src/drone_trajectory_logger/tools/offline_demo.py

输出：
    docs/img/air/drone_trajectory_logger_compare.png
    （若指定 --out 可改路径）

三组航迹：
  truth   —— 带一阶滞后的真实运动（等价于里程计）
  new     —— 新版：按 v*dt / 圆弧积分从头推算（无里程计兜底场景）
  old     —— 旧版：写死 0.5 m/步、0.2 rad/步、互斥状态判定
"""

import argparse
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.abspath(os.path.join(HERE, '..')))

from drone_trajectory_logger.trajectory_core import (   # noqa: E402
    DeadReckoner, TrajectoryRecorder, classify_motion, wrap_to_pi,
)

# 与 main.py 的 MISSION 保持一致
MISSION = [
    (2.0, 0.0, 0.0),
    (6.0, 1.5, 0.0),
    (6.0, 1.5, 0.5),
    (4.0, 1.5, 0.0),
    (3.0, 0.0, -1.0),
    (2.0, 0.0, 0.0),
]

DT = 0.05          # 20 Hz
TAU = 0.25         # 指令->运动一阶滞后


class LegacyReckoner:
    """旧版逻辑（原文照抄的等价实现）。"""

    def __init__(self):
        self.x = self.y = self.theta = 0.0
        self.path = 0.0

    def step(self, v_cmd, w_cmd, dt):
        if v_cmd > 0:
            dx = 0.5 * math.cos(self.theta)
            dy = 0.5 * math.sin(self.theta)
            self.x += dx
            self.y += dy
            self.path += 0.5
        elif w_cmd > 0:
            self.theta += 0.2
            self.path += 0.0
        # 其余情况（含 w<0）按悬停处理，不动


def simulate():
    truth = DeadReckoner()
    new_dr = DeadReckoner()
    old = LegacyReckoner()

    rec_truth = TrajectoryRecorder()
    rec_new = TrajectoryRecorder()
    rec_old = TrajectoryRecorder()

    v_act = w_act = 0.0
    t = 0.0
    err_new, err_old = [], []

    for dur, v_cmd, w_cmd in MISSION:
        n = int(round(dur / DT))
        for _ in range(n):
            alpha = min(1.0, DT / TAU)
            v_act += (v_cmd - v_act) * alpha
            w_act += (w_cmd - w_act) * alpha

            # 真实位姿（里程计）
            tx, ty, tyaw = truth.step(v_act, w_act, DT)
            rec_truth.update(t, tx, ty, 0.0, tyaw, v_act, w_act,
                             classify_motion(v_act, w_act))

            # 新版：只有指令，按真实 dt 推算
            nx, ny, nyaw = new_dr.step(v_cmd, w_cmd, DT)
            rec_new.update(t, nx, ny, 0.0, nyaw, v_cmd, w_cmd,
                           classify_motion(v_cmd, w_cmd))

            # 旧版
            old.step(v_cmd, w_cmd, DT)
            rec_old.update(t, old.x, old.y, 0.0, wrap_to_pi(old.theta),
                           v_cmd, w_cmd, 'cruise' if v_cmd > 0 else 'hover')

            err_new.append(math.hypot(nx - tx, ny - ty))
            err_old.append(math.hypot(old.x - tx, old.y - ty))
            t += DT

    return {
        'truth': rec_truth, 'new': rec_new, 'old': rec_old,
        't': [i * DT for i in range(len(err_new))],
        'err_new': err_new, 'err_old': err_old,
    }


def print_report(res):
    print('=' * 72)
    print('同一条任务剖面下的航迹对比（23 秒，20 Hz 发布）')
    print('=' * 72)
    print('%-10s %14s %14s %14s' % ('指标', '真实(里程计)', '新版推算', '旧版推算'))
    rows = [('记录点数', 'points', '%d'), ('航迹里程 m', 'path_length_m', '%.2f'),
            ('起点-终点 m', 'displacement_m', '%.2f'),
            ('直线度', 'straightness', '%.3f'),
            ('最大速度 m/s', 'max_speed_mps', '%.2f')]
    s_truth = res['truth'].summary()
    s_new = res['new'].summary()
    s_old = res['old'].summary()
    for label, key, fmt in rows:
        print('%-10s ' % label + ' '.join(
            (fmt % s[key]).rjust(14) for s in (s_truth, s_new, s_old)))
    print('-' * 72)
    print('终点位置误差: 新版 %.2f m | 旧版 %.2f m'
          % (res['err_new'][-1], res['err_old'][-1]))
    print('最大位置误差: 新版 %.2f m | 旧版 %.2f m'
          % (max(res['err_new']), max(res['err_old'])))
    print()
    print('旧版状态识别（同一剖面）:')
    states = {}
    for r in res['old'].rows:
        states[r[7]] = states.get(r[7], 0) + 1
    print('  旧版只出现 %s 两种状态（真实的弧线飞行/右转全部丢失）' % list(states))
    states_new = {}
    for r in res['new'].rows:
        states_new[r[7]] = states_new.get(r[7], 0) + 1
    print('  新版识别出 %s' % {k: v for k, v in sorted(states_new.items())})


def make_figure(res, out_path):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    plt.rcParams['font.sans-serif'] = ['Microsoft YaHei', 'SimHei', 'DejaVu Sans']
    plt.rcParams['axes.unicode_minus'] = False

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12.5, 5.4))

    for rec, label, style, color, lw in (
            (res['truth'], '真实航迹（里程计）', '-', '#1f77b4', 2.4),
            (res['new'], '新版：按 dt 推算', '--', '#2ca02c', 1.6),
            (res['old'], '旧版：写死 0.5 m/步', ':', '#d62728', 1.8)):
        xs = [r[1] for r in rec.rows]
        ys = [r[2] for r in rec.rows]
        ax1.plot(xs, ys, style, color=color, lw=lw, label=label)

    ax1.plot(0, 0, 'ko', ms=6)
    ax1.annotate('起点', (0, 0), textcoords='offset points', xytext=(6, -12), fontsize=9)
    tx, ty = res['truth'].rows[-1][1], res['truth'].rows[-1][2]
    ax1.plot(tx, ty, 'k*', ms=13)
    ax1.annotate('终点', (tx, ty), textcoords='offset points', xytext=(6, 6), fontsize=9)
    ax1.set_xlabel('X / m')
    ax1.set_ylabel('Y / m')
    ax1.set_title('三种航迹对比（23 s 任务剖面）')
    ax1.grid(True, ls=':')
    ax1.legend(loc='lower right', fontsize=9)
    ax1.set_aspect('equal', adjustable='datalim')

    # 局部放大：真实航迹与新版推算紧贴在一起，旧版一去不返
    axins = ax1.inset_axes([0.32, 0.12, 0.40, 0.40])
    for rec, style, color, lw in ((res['truth'], '-', '#1f77b4', 2.2),
                                  (res['new'], '--', '#2ca02c', 1.5)):
        xs = [r[1] for r in rec.rows]
        ys = [r[2] for r in rec.rows]
        axins.plot(xs, ys, style, color=color, lw=lw)
    axins.plot(0, 0, 'ko', ms=4)
    axins.set_xlim(-1.5, 10.0)
    axins.set_ylim(-2.5, 6.5)
    axins.set_xticks(range(0, 10, 2))
    axins.grid(True, ls=':', lw=0.5)
    axins.tick_params(labelsize=7)
    axins.set_title('局部放大（真实 vs 新版）', fontsize=8)
    ax1.indicate_inset_zoom(axins, edgecolor='gray')

    ax2.plot(res['t'], res['err_old'], ':', color='#d62728', lw=1.8, label='旧版')
    ax2.plot(res['t'], res['err_new'], '--', color='#2ca02c', lw=1.6, label='新版推算')
    ax2.fill_between(res['t'], 0, res['err_old'], color='#d62728', alpha=0.08)
    ax2.set_xlabel('时间 / s')
    ax2.set_ylabel('与真实位姿的位置误差 / m')
    ax2.set_title('位置误差随时间增长')
    ax2.grid(True, ls=':')
    ax2.legend(loc='upper left', fontsize=9)

    ax2.annotate('终点误差 %.1f m' % res['err_old'][-1],
                 (res['t'][-1], res['err_old'][-1]),
                 textcoords='offset points', xytext=(-96, 6),
                 fontsize=9, color='#d62728')
    ax2.annotate('终点误差 %.1f m' % res['err_new'][-1],
                 (res['t'][-1], res['err_new'][-1]),
                 textcoords='offset points', xytext=(-96, -14),
                 fontsize=9, color='#2ca02c')

    fig.tight_layout()
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    fig.savefig(out_path, dpi=160)
    print('图已保存: %s' % out_path)


def main():
    parser = argparse.ArgumentParser()
    repo_root = os.path.abspath(os.path.join(HERE, '..', '..', '..'))
    default_out = os.path.join(repo_root, 'docs', 'img', 'air',
                               'drone_trajectory_logger_compare.png')
    parser.add_argument('--out', default=default_out)
    known, _ = parser.parse_known_args()

    res = simulate()
    print_report(res)
    make_figure(res, known.out)


if __name__ == '__main__':
    main()
