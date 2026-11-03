"""imu_core 单元测试。

运行：
    cd src/air/imu_allan_calibration
    python -m pytest test/test_imu_core.py -v
"""

import math
import os
import sys

import pytest

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

import imu_core as imu   # noqa: E402


# ---------------------------------------------------------------- 仿真器

def test_simulator_deterministic_with_seed():
    a = imu.simulate_static_gyro(10.0, 100.0, bias=0.1, arw_psd=0.05 ** 2, seed=42)
    b = imu.simulate_static_gyro(10.0, 100.0, bias=0.1, arw_psd=0.05 ** 2, seed=42)
    c = imu.simulate_static_gyro(10.0, 100.0, bias=0.1, arw_psd=0.05 ** 2, seed=7)
    assert a == b
    assert a[:10] != c[:10]
    assert len(a) == 1000


def test_simulator_rejects_bad_args():
    with pytest.raises(ValueError):
        imu.simulate_static_gyro(0.0, 100.0)
    with pytest.raises(ValueError):
        imu.simulate_static_gyro(1.0, 0.0)


def test_simulator_bias_recovered_in_mean():
    """白噪声/随机游走为零时，均值应精确等于零偏。"""
    data = imu.simulate_static_gyro(5.0, 100.0, bias=0.25, seed=1)
    mean, std, n = imu.static_bias_estimate(data, 100.0, trim_s=0.0)
    assert mean == pytest.approx(0.25, abs=1e-12)
    assert std == 0.0
    assert n == 500


# ---------------------------------------------------------------- Allan 方差

def test_allan_needs_enough_samples():
    with pytest.raises(ValueError):
        imu.allan_deviation([1.0] * 8, 100.0)


def test_allan_pure_white_noise_recovers_arw():
    """纯白噪声：adev ≈ N/√τ，N=sqrt(psd)。"""
    true_n = 0.01                       # (单位/√s)
    data = imu.simulate_static_gyro(600.0, 100.0, arw_psd=true_n ** 2, seed=3)
    taus, adevs = imu.allan_deviation(data, 100.0)
    params = imu.fit_noise_params(taus, adevs)
    assert params['N'] == pytest.approx(true_n, rel=0.15)


def test_allan_pure_random_walk_recovers_rrw():
    """纯速率随机游走：adev ≈ K·√(τ/3)。"""
    true_k = 0.0005
    data = imu.simulate_static_gyro(3600.0, 100.0, rrw_psd=true_k ** 2, seed=4)
    taus, adevs = imu.allan_deviation(data, 100.0)
    params = imu.fit_noise_params(taus, adevs)
    assert params['K'] == pytest.approx(true_k, rel=0.25)


def test_allan_curve_v_shape_for_combined_noise():
    """白噪声+随机游走组合：曲线先降后升（V 形）。

    谷底位置 τ* = N·√3/K，需落在数据时长内（m_max = n/9 → τ_max ≈ 180s）。
    取 N=0.005、K=0.0005 → τ* ≈ 17 s。
    """
    data = imu.simulate_static_gyro(1800.0, 100.0,
                                    arw_psd=0.005 ** 2, rrw_psd=0.0005 ** 2,
                                    seed=5)
    taus, adevs = imu.allan_deviation(data, 100.0)
    assert len(taus) >= 5
    params = imu.fit_noise_params(taus, adevs)
    assert params['N'] is not None and params['K'] is not None
    assert taus[0] < params['tau_at_min'] < taus[-1]
    # V 形：谷底两侧的 adev 都明显高于谷底
    i_min = adevs.index(min(adevs))
    assert i_min not in (0, len(adevs) - 1)
    assert adevs[0] > 2.0 * adevs[i_min]
    assert adevs[-1] > 1.2 * adevs[i_min]


def test_allan_constant_signal_zero_deviation():
    """恒定信号（无噪声）的 Allan 标准差应接近 0（浮点累计误差量级）。"""
    taus, adevs = imu.allan_deviation([1.0] * 900, 100.0)
    assert all(a < 1e-12 for a in adevs)


def test_allan_constant_bias_does_not_inflate_adev():
    """固定零偏会被相邻差分消掉，不抬升 Allan 曲线（真正的零偏不稳定性
    来自零偏的慢波动，由 B 系数描述）。"""
    noisy = imu.simulate_static_gyro(600.0, 100.0, arw_psd=0.01 ** 2, seed=21)
    biased = [v + 0.5 for v in noisy]
    _, a1 = imu.allan_deviation(noisy, 100.0)
    _, a2 = imu.allan_deviation(biased, 100.0)
    assert a2 == pytest.approx(a1, rel=1e-6)


def test_allan_taus_monotonic_grid():
    data = imu.simulate_static_gyro(60.0, 50.0, seed=9)
    taus, _ = imu.allan_deviation(data, 50.0)
    assert all(b > a for a, b in zip(taus, taus[1:]))
    # 网格按几何步长（约 1.3 倍）分布，相邻比在 1~2.2 之间
    for a, b in zip(taus, taus[1:]):
        assert 1.0 < b / a < 2.3


# ---------------------------------------------------------------- 拟合

def test_fit_rejects_mismatched_lengths():
    with pytest.raises(ValueError):
        imu.fit_noise_params([1.0], [1.0, 2.0])
    with pytest.raises(ValueError):
        imu.fit_noise_params([], [])


def test_fit_bias_coefficient_definition():
    """B 定义为谷底 × 0.664（IEEE 952 惯例），验证实现一致。"""
    taus = [0.1, 1.0, 10.0, 100.0]
    adevs = [0.05, 0.01, 0.002, 0.004]      # 谷底在 τ=10
    params = imu.fit_noise_params(taus, adevs)
    assert params['tau_at_min'] == 10.0
    assert params['B'] == pytest.approx(0.002 * 0.664, rel=1e-9)


# ---------------------------------------------------------------- 标定一站式

def test_calibrate_report_keys():
    data = imu.simulate_static_gyro(300.0, 100.0, bias=0.03,
                                    arw_psd=0.02 ** 2, seed=12)
    report = imu.calibrate(data, 100.0, trim_s=5.0)
    for key in ('samples', 'fs', 'bias', 'bias_std', 'N', 'B', 'K',
                'tau_at_min', 'taus', 'adevs'):
        assert key in report
    assert report['bias'] == pytest.approx(0.03, abs=0.005)
    assert report['samples'] == 29500          # 丢掉了前 5 s 收敛段
    assert len(report['taus']) == len(report['adevs'])


def test_bias_estimate_trims_convergence():
    """前 5 秒注入异常值，trim 后应不受影响。"""
    data = imu.simulate_static_gyro(20.0, 10.0, bias=0.1, seed=13)
    for i in range(50):            # 前 5 秒
        data[i] += 5.0
    bias, _, _ = imu.static_bias_estimate(data, 10.0, trim_s=5.0)
    assert bias == pytest.approx(0.1, abs=0.02)


def test_units_are_agostic():
    """同一份数据无论按 rad/s 还是 deg/s 解读，N 的相对值不变。"""
    d = imu.simulate_static_gyro(300.0, 100.0, arw_psd=0.01 ** 2, seed=14)
    taus, adevs = imu.allan_deviation(d, 100.0)
    p1 = imu.fit_noise_params(taus, adevs)
    d2 = [v * 57.2958 for v in d]
    taus2, adevs2 = imu.allan_deviation(d2, 100.0)
    p2 = imu.fit_noise_params(taus2, adevs2)
    assert p2['N'] == pytest.approx(p1['N'] * 57.2958, rel=0.2)
