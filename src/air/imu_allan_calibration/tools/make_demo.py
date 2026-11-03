"""离线演示：对三种等级的仿真 IMU 做静态标定，生成 Allan 曲线对比图与报告。

运行：
    python src/air/imu_allan_calibration/tools/make_demo.py

输出：
    docs/img/air/imu_allan_calibration.png
    src/air/imu_allan_calibration/calibration_report.json
"""

import json
import math
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.abspath(os.path.join(HERE, '..')))

import imu_core as imu   # noqa: E402

# (名称, 零偏 °/s, ARW N °/√s, RRW K °/s/√s, 颜色)
# 谷底位置 τ* = N·√3/K 需明显小于数据时长，V 形才完整
SENSORS = [
    ('消费级 MEMS', 0.20, 0.050, 0.0010, '#d62728'),
    ('车规级 MEMS', 0.05, 0.010, 0.0003, '#ff7f0e'),
    ('战术级光纤',  0.01, 0.002, 0.0001, '#1f77b4'),
]

DURATION_S = 3600.0
FS = 100.0


def run():
    results = []
    for name, bias, n_true, k_true, color in SENSORS:
        t0 = time.time()
        data = imu.simulate_static_gyro(DURATION_S, FS, bias=bias,
                                        arw_psd=n_true ** 2,
                                        rrw_psd=k_true ** 2, seed=hash(name) % 2**31)
        report = imu.calibrate(data, FS, trim_s=30.0)
        report['name'] = name
        report['truth'] = {'bias': bias, 'N': n_true, 'K': k_true}
        report['color'] = color
        report['sim_seconds'] = round(time.time() - t0, 1)
        results.append(report)
        print('=' * 70)
        print('%s（零偏 %.3f °/s, N=%.4f °/√s, K=%.5f °/s/√s）'
              % (name, bias, n_true, k_true))
        print('  数据: %.0f s @ %.0f Hz（%d 样本，仿真+标定耗时 %.1fs）'
              % (DURATION_S, FS, report['samples'], report['sim_seconds']))
        print('  零偏估计: %.4f °/s（真值 %.3f，误差 %.1f%%）'
              % (report['bias'], bias, abs(report['bias'] - bias) / bias * 100))
        print('  ARW  N: %.4f °/√s（真值 %.4f，误差 %.1f%%）'
              % (report['N'], n_true, abs(report['N'] - n_true) / n_true * 100))
        print('  零偏不稳定性 B: %.5f °/s（谷底 τ=%.2f s）'
              % (report['B'], report['tau_at_min']))
        if report['K'] is not None:
            k_line = '%.5f °/s/√s（真值 %.5f，误差 %.1f%%）' % (
                report['K'], k_true, abs(report['K'] - k_true) / k_true * 100)
        else:
            k_line = '未估出（数据时长不足以覆盖随机游走段）'
        print('  RRW  K: %s' % k_line)

    # 导出 JSON（去掉曲线数据以保持精简）
    slim = []
    for r in results:
        slim.append({k: v for k, v in r.items() if k not in ('taus', 'adevs', 'color')})
    json_path = os.path.join(HERE, '..', 'calibration_report.json')
    with open(json_path, 'w', encoding='utf-8') as f:
        json.dump(slim, f, ensure_ascii=False, indent=2)
    print('\n报告已写入: %s' % os.path.abspath(json_path))
    return results


def make_figure(results, out_path):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    plt.rcParams['font.sans-serif'] = ['Microsoft YaHei', 'SimHei', 'DejaVu Sans']
    plt.rcParams['axes.unicode_minus'] = False

    fig, ax = plt.subplots(figsize=(9.5, 6))

    for r in results:
        ax.loglog(r['taus'], r['adevs'], '-', color=r['color'], lw=1.8,
                  label='%s（实测）' % r['name'])
        # 真值参考线：N/√τ 与 K·√(τ/3)
        n_true, k_true = r['truth']['N'], r['truth']['K']
        taus = r['taus']
        ax.loglog(taus, [n_true / math.sqrt(t) for t in taus], ':',
                  color=r['color'], lw=0.9, alpha=0.65)
        ax.loglog(taus, [k_true * math.sqrt(t / 3.0) for t in taus], '--',
                  color=r['color'], lw=0.9, alpha=0.65)
        # N 与 B 标记
        ax.loglog([1.0], [r['N']], 'o', color=r['color'], ms=6,
                  mfc='white', mew=1.5)
        ax.loglog([r['tau_at_min']], [r['B']], '*', color=r['color'], ms=11)

    ax.loglog([], [], 'k:', lw=1, label='真值参考线 N/√τ（上）与 K·√(τ/3)（下）')
    ax.loglog([], [], 'ko', mfc='white', ms=6, label='拟合 N（τ=1s）')
    ax.loglog([], [], 'k*', ms=11, label='拟合 B（谷底×0.664）')

    ax.set_xlabel('聚类时间 τ / s')
    ax.set_ylabel('Allan 标准差 σ(τ) / (°/s)')
    ax.set_title('三种等级 IMU 的静态 Allan 方差标定（3600 s @ 100 Hz）')
    ax.grid(True, which='both', ls=':', alpha=0.5)
    ax.legend(loc='lower left', fontsize=8.5)

    fig.tight_layout()
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    fig.savefig(out_path, dpi=160)
    print('图已保存: %s' % out_path)


def main():
    repo_root = os.path.abspath(os.path.join(HERE, '..', '..', '..', '..'))
    default_out = os.path.join(repo_root, 'docs', 'img', 'air',
                               'imu_allan_calibration.png')
    results = run()
    make_figure(results, default_out)


if __name__ == '__main__':
    main()
