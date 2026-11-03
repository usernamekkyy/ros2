"""IMU 静态标定核心：Allan 方差与噪声参数拟合（纯标准库，无第三方依赖）。

模型
----
静态放置的陀螺仪输出建模为三部分之和::

    omega_meas(t) = b + n_white(t) + w(t)

* ``n_white``：角度随机游走（白噪声），功率谱密度 ``N²``，
  单位 (单位/√s)² 量纲上体现为 ARW 系数 ``N``；
* ``w``：速率随机游走，功率谱密度 ``K²``；
* ``b``：固定零偏（bias instability ``B`` 体现为零偏的慢波动下限）。

Allan 方差对这三者分别以 ``τ^-1/2``、``τ^0``（谷底）、``τ^+1/2`` 的斜率
出现在双对数曲线上，因此可以从一条曲线同时估出 ``N / B / K``。

单位约定：本模块不关心具体单位（rad/s 或 °/s 皆可），
输入输出保持同一单位，拟合结果与输入同单位。
"""

import math
import random


# ---------------------------------------------------------------- 数据仿真

def simulate_static_gyro(duration_s, fs, bias=0.0,
                         arw_psd=0.0, rrw_psd=0.0, seed=None):
    """生成静态陀螺仪数据。

    参数
    ----
    duration_s : 采样时长（秒）
    fs         : 采样频率（Hz）
    bias       : 固定零偏（与数据同单位）
    arw_psd    : 角度随机游走功率谱密度，白噪声强度（(单位)^2/Hz），
                 理论 ARW 系数 N = sqrt(arw_psd)
    rrw_psd    : 速率随机游走功率谱密度（(单位/s)^2/Hz），
                 理论 RRW 系数 K = sqrt(rrw_psd)
    seed       : 随机种子（保证可复现）

    返回
    ----
    list[float]：长度 duration_s * fs 的角速率序列。
    """
    if duration_s <= 0.0 or fs <= 0.0:
        raise ValueError('duration_s and fs must be positive')
    rng = random.Random(seed)
    dt = 1.0 / fs
    n = int(round(duration_s * fs))
    sigma_white = math.sqrt(arw_psd / dt)        # 白噪声离散化标准差
    sigma_rw = math.sqrt(rrw_psd * dt)           # 随机游走每步增量标准差
    state = 0.0
    out = []
    for _ in range(n):
        state += rng.gauss(0.0, sigma_rw)
        out.append(bias + state + rng.gauss(0.0, sigma_white))
    return out


def simulate_accel_static(duration_s, fs, bias=0.0,
                          vrw_psd=0.0, rrw_psd=0.0, seed=None):
    """加速度计静态数据的别名（模型与陀螺相同，系数含义对应 VRW/RRW）。"""
    return simulate_static_gyro(duration_s, fs, bias, vrw_psd, rrw_psd, seed)


# ---------------------------------------------------------------- Allan 方差

def allan_deviation(data, fs, max_points_per_decade=12):
    """计算重叠型 Allan 标准差。

    参数
    ----
    data : list[float]，等间隔采样
    fs   : 采样频率 Hz
    max_points_per_decade : 每个十倍程内最多取多少个聚类长度（控制计算量）

    返回
    ----
    (taus, adevs)：两个等长 list，τ 单位秒，adev 与数据同单位。
    """
    if fs <= 0.0:
        raise ValueError('fs must be positive')
    n = len(data)
    if n < 9:
        raise ValueError('need at least 9 samples for Allan deviation')

    dt = 1.0 / fs
    # 相位（角度）序列：θ_i = cumsum(omega) * dt
    theta = []
    acc = 0.0
    for v in data:
        acc += v * dt
        theta.append(acc)

    m_max = n // 9            # 保证每段至少 9 个不重叠聚类，估计才可信
    # 几何步长（×1.3）覆盖 [1, m_max]，比纯 2 的幂分辨率高，
    # 否则真实的谷底 τ* 往往落不到任何采样点上
    ms = []
    m = 1
    while m <= m_max:
        ms.append(m)
        m = max(m + 1, int(m * 1.3))
    # 每十倍程抽稀到 max_points_per_decade 个点
    if max_points_per_decade and len(ms) > 1:
        picked = []
        decade_start = 0
        for i in range(1, len(ms) + 1):
            if i == len(ms) or math.log10(ms[i] / ms[decade_start]) >= 1.0:
                block = ms[decade_start:i]
                step = max(1, len(block) // max_points_per_decade)
                picked.extend(block[::step])
                if block[-1] not in picked[-step if step > 1 else -1:]:
                    picked.append(block[-1])
                decade_start = i
        ms = sorted(set(picked))

    taus, adevs = [], []
    for m in ms:
        tau = m * dt
        # 重叠型：θ_{i+2m} - 2θ_{i+m} + θ_i
        s = 0.0
        count = 0
        for i in range(0, n - 2 * m):
            d = theta[i + 2 * m] - 2.0 * theta[i + m] + theta[i]
            s += d * d
            count += 1
        adev = math.sqrt(s / (2.0 * tau * tau * count))
        taus.append(tau)
        adevs.append(adev)
    return taus, adevs


# ---------------------------------------------------------------- 参数拟合

def fit_noise_params(taus, adevs):
    """从 Allan 曲线拟合 (N, B, K)。

    * N（角度随机游走）：左段（τ < 1 s）斜率 -1/2，N = mean(adev·√τ)
    * B（零偏不稳定性）：曲线最低点，B = adev_min × 0.664
    * K（速率随机游走）：右段（τ > 10·τ_min）斜率 +1/2，K = mean(adev·√(3/τ))

    返回 dict：{'N': ..., 'B': ..., 'K': ..., 'tau_at_min': ...}。
    若某段数据不足，对应项返回 None。
    """
    if len(taus) != len(adevs) or not taus:
        raise ValueError('taus/adevs must be same non-empty length')

    tau_min_idx = min(range(len(adevs)), key=lambda i: adevs[i])
    tau_min = taus[tau_min_idx]

    # --- N：取 τ ∈ [0.1·τ_min, 1s] 且位于最低点左侧的段 ---
    n_vals = [a * math.sqrt(t) for t, a in zip(taus, adevs)
              if t <= max(1.0, tau_min) and t <= tau_min]
    # 更稳妥：只用最低点左侧、τ<=1s 的数据
    n_vals = [a * math.sqrt(t) for t, a in zip(taus, adevs)
              if t < tau_min and t <= 1.0]
    N = sum(n_vals) / len(n_vals) if n_vals else None

    # --- B：最低点 ---
    B = adevs[tau_min_idx] * 0.664

    # --- K：最低点右侧（斜率 +1/2 区），判据 τ>=3·τ_min，逐级回退 ---
    right = [(t, a) for t, a in zip(taus, adevs) if t > tau_min]
    k_vals = [a * math.sqrt(3.0 / t) for t, a in right if t >= 3.0 * tau_min]
    if not k_vals:
        k_vals = [a * math.sqrt(3.0 / t) for t, a in right if t >= 2.0 * tau_min]
    if not k_vals:
        k_vals = [a * math.sqrt(3.0 / t) for t, a in right]
    K = sum(k_vals) / len(k_vals) if k_vals else None

    return {'N': N, 'B': B, 'K': K, 'tau_at_min': tau_min}


# ---------------------------------------------------------------- 零偏估计

def static_bias_estimate(data, fs, trim_s=10.0):
    """静态零偏估计：丢弃前 trim_s 秒（滤波器收敛），返回 (bias, std, n)。"""
    if fs <= 0.0:
        raise ValueError('fs must be positive')
    start = int(round(trim_s * fs))
    trimmed = data[start:] if start < len(data) else data
    n = len(trimmed)
    if n == 0:
        raise ValueError('no samples left after trimming')
    mean = sum(trimmed) / n
    var = sum((v - mean) ** 2 for v in trimmed) / max(1, n - 1)
    return mean, math.sqrt(var), n


# ---------------------------------------------------------------- 一站式标定

def calibrate(data, fs, trim_s=10.0, max_points_per_decade=12):
    """对一段静态数据完成完整标定，返回报告 dict。"""
    bias, std, n = static_bias_estimate(data, fs, trim_s)
    taus, adevs = allan_deviation(data, fs, max_points_per_decade)
    params = fit_noise_params(taus, adevs)
    return {
        'samples': n,
        'fs': fs,
        'bias': bias,
        'bias_std': std,
        'N': params['N'],
        'B': params['B'],
        'K': params['K'],
        'tau_at_min': params['tau_at_min'],
        'taus': taus,
        'adevs': adevs,
    }
