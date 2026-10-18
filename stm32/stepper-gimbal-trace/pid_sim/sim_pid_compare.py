#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
步进电机云台追踪闭环 新旧控制器仿真对比
=================================================
目的: 在 PC 上用一阶云台模型 + 移动靶点 + K230 帧采样,
      对比优化前(v5.0 纯 PD + 泄漏积分前馈) 与 优化后
      (v6.1 平滑增益调度 + 近区积分(抗饱和) + 解析前馈) 的追踪表现。

指标: 稳态滞后 / 整定时间 / 过冲 / RMS 抖动。

用法: python sim_pid_compare.py
输出: pid_sim/pid_compare.png (或 .svg) + 终端指标表
"""
import math
import random

# ----------------------- 固件参数 (与 main.c 一致) -----------------------
PAN_GAIN      = 0.005   # 速度增益 (1/px), 200px->全速
CTRL_KD       = 0.0005  # 接近阻尼
PID_DEADZONE  = 2.0     # 死区(px)
FF_GAIN       = 1.0     # 前馈增益
FF_TRACE_MAX  = 0.85    # 活跃循迹速度上限
CTRL_D_ALPHA  = 0.35    # 不完全微分 IIR 系数
FF_RATE_ALPHA = 0.50    # 前馈 err_rate EMA 系数

# v6.1 增益调度: 按"是否定点"切换 (跟踪 vs 定点), 而非按 |err|
KP_TRACK_SCALE = 1.15   # 跟踪中: 适度 Kp -> 低滞后
KD_TRACK_SCALE = 0.85   # 跟踪中: 降阻尼 -> 允许快速跟随
KP_SETTLE_SCALE= 2.50   # 定点中: 高 Kp -> 快速闭隙克服 PD 蠕动
KD_SETTLE_SCALE= 2.00   # 定点中: 高 Kd -> 抑过冲

# 云台模型 (一阶, 速度限幅 + 加速度限幅)
V_MAX = 200.0    # vel_cmd=1 -> 200 px/s
A_MAX = 320.0    # 加速度限幅 px/s^2 (近似 stepper 加减速)

# 卡尔曼 (与固件一致: q=20, r=25)
KF_Q, KF_R = 20.0, 25.0


def kalman_update(kf, z):
    if not kf['init']:
        kf['x'] = z; kf['p'] = 1.0; kf['init'] = True
        return kf['x']
    p_pred = kf['p'] + kf['q']
    k = p_pred / (p_pred + kf['r'])
    kf['x'] += k * (z - kf['x'])
    kf['p'] = (1.0 - k) * p_pred
    return kf['x']


# ============================ 旧控制器 v5.0 ============================
class OldCtrl:
    """纯 PD 速度控制器 + 泄漏积分式前馈 (原固件行为)。"""
    def __init__(self):
        self.kp = PAN_GAIN
        self.kd = CTRL_KD
        self.d_alpha = CTRL_D_ALPHA
        self.prev_err = 0.0
        self.deriv_f = 0.0
        self.prev_cmd = 0.0      # 泄漏积分状态
        self.prev_targ_err = 0.0
        self.rate_f = 0.0
        self.last_tick = 0.0

    def reset(self):
        self.prev_err = self.deriv_f = 0.0
        self.prev_cmd = 0.0
        self.prev_targ_err = 0.0
        self.rate_f = 0.0

    def update(self, err, t, settling):
        dt = t - self.last_tick
        if dt < 1e-4: dt = 1e-4
        if dt > 1.0: dt = 1.0
        self.last_tick = t

        out = self.kp * err
        derr_raw = (err - self.prev_err) / dt
        self.prev_err = err
        self.deriv_f = self.deriv_f * (1 - self.d_alpha) + derr_raw * self.d_alpha
        out += self.kd * self.deriv_f

        # 泄漏积分前馈
        if (not settling) and FF_GAIN > 0:
            err_rate_raw = (err - self.prev_targ_err) / dt
            self.rate_f = self.rate_f * (1 - FF_RATE_ALPHA) + err_rate_raw * FF_RATE_ALPHA
            v_des = self.prev_cmd + self.kp * self.rate_f   # 泄漏积分
            out += FF_GAIN * v_des
            if out >  FF_TRACE_MAX: out =  FF_TRACE_MAX
            if out < -FF_TRACE_MAX: out = -FF_TRACE_MAX
        self.prev_targ_err = err

        # 限幅 + 记录泄漏状态
        if out >  1.0: out =  1.0
        if out < -1.0: out = -1.0
        self.prev_cmd = out
        return out


# ============================ 新控制器 v6.1 ============================
class NewCtrl:
    """平滑增益调度 + 不完全微分 + 目标速度前馈(恢复正确的 cmd 累加式)。

    关键修正: 速度控制器的前馈必须用"目标速度指令" v_target/V_MAX,
    而 kp≈1/V_MAX(200px→全速), 故 cmd_prev 累加项恰好携带该前馈速度,
    移项后即隐式 PI, 才能把匀速滞后压到≈0。旧版 '泄漏积分' 实为正确前馈。
    新版只额外叠加 平滑增益调度(far/near Kp·Kd), 不改前馈本质。"""
    def __init__(self):
        self.kp = PAN_GAIN
        self.kd = CTRL_KD
        self.d_alpha = CTRL_D_ALPHA
        self.prev_err = 0.0
        self.deriv_f = 0.0
        self.ff_cmd = 0.0       # 前馈速度指令累加(=v_target/V_MAX 估计)
        self.rate_f = 0.0
        self.prev_targ_err = 0.0
        self.last_tick = 0.0

    def reset(self):
        self.prev_err = self.deriv_f = 0.0
        self.ff_cmd = 0.0
        self.rate_f = 0.0
        self.prev_targ_err = 0.0

    def update(self, err, t, settling):
        dt = t - self.last_tick
        if dt < 1e-4: dt = 1e-4
        if dt > 1.0: dt = 1.0
        self.last_tick = t

        # 增益调度: 按"是否定点"切换 (而非 |err|)
        #  跟踪中(靶在动): 高 Kp 低 Kd -> 低滞后快跟
        #  定点中(靶停)  : 低 Kp 高 Kd -> 抑过冲、平滑入位
        if settling:
            kp_eff = self.kp * KP_SETTLE_SCALE
            kd_eff = self.kd * KD_SETTLE_SCALE
        else:
            kp_eff = self.kp * KP_TRACK_SCALE
            kd_eff = self.kd * KD_TRACK_SCALE

        out = kp_eff * err
        derr_raw = (err - self.prev_err) / dt
        self.prev_err = err
        self.deriv_f = self.deriv_f * (1 - self.d_alpha) + derr_raw * self.d_alpha
        out += kd_eff * self.deriv_f

        # 目标速度前馈: cmd 累加式 (等价于 v_target/V_MAX 前馈 + 隐式 PI)
        if (not settling) and FF_GAIN > 0:
            err_rate_raw = (err - self.prev_targ_err) / dt
            self.rate_f = self.rate_f * (1 - FF_RATE_ALPHA) + err_rate_raw * FF_RATE_ALPHA
            v_des = self.ff_cmd + self.kp * self.rate_f   # kp≈1/Vmax 转指令单位
            out += FF_GAIN * v_des
            if out >  FF_TRACE_MAX: out =  FF_TRACE_MAX
            if out < -FF_TRACE_MAX: out = -FF_TRACE_MAX
            self.ff_cmd = out      # 记录(含 P/FF)供下帧累加
        else:
            self.ff_cmd = 0.0      # 定点时清零前馈累加, 防 creep/过冲
        self.prev_targ_err = err

        out = max(-1.0, min(1.0, out))
        return out


# ============================ 云台 + 仿真循环 ============================
def run_sim(ctrl_factory, seed=42):
    random.seed(seed)
    t = 0.0
    dt = 1.0 / 30.0
    T_end = 18.0

    gimbal = 0.0      # 云台实际位置 (px)
    vel = 0.0         # 实际角速度 (px/s)
    target = 0.0      # 靶点位置 (px)

    ctrl = ctrl_factory()
    kf = {'x': 0.0, 'p': 1.0, 'q': KF_Q, 'r': KF_R, 'init': False}

    # 场景: 移动 -> 停 -> 反向移动 -> 停
    # (start, end, velocity)
    segs = [(0.0, 4.0, 60.0), (4.0, 9.0, 0.0),
            (9.0, 13.0, -80.0), (13.0, 18.0, 0.0)]

    ts, tgt, gim, errs, cmds = [], [], [], [], []
    while t < T_end:
        # 靶点速度
        v_t = 0.0
        for (a, b, vv) in segs:
            if a <= t < b:
                v_t = vv
                break
        target += v_t * dt
        # K230 检测抖动 (±1.5px)
        meas_noise = random.uniform(-1.5, 1.5)
        raw_err = (target + meas_noise) - gimbal
        kf_err = kalman_update(kf, raw_err)

        settling = (abs(v_t) < 1e-3)  # 靶点静止 -> 定点(settling)
        cmd = ctrl.update(kf_err, t, settling)

        # 减速区(settling 时按 |err| 线性降速)
        if settling:
            dec_zone = 60.0
            a = abs(kf_err)
            shape = (a - PID_DEADZONE) / (dec_zone - PID_DEADZONE)
            shape = max(0.0, min(1.0, shape))
            if abs(cmd) > shape:
                cmd = math.copysign(shape, cmd) if shape > 0 else 0.0
        # 死区
        if abs(kf_err) < PID_DEADZONE:
            cmd = 0.0

        # 云台模型: 速度跟随命令(带加速度限幅)
        des_vel = cmd * V_MAX
        dv = des_vel - vel
        dv = max(-A_MAX * dt, min(A_MAX * dt, dv))
        vel += dv
        gimbal += vel * dt

        ts.append(t); tgt.append(target); gim.append(gimbal)
        errs.append(kf_err); cmds.append(cmd)
        t += dt
    return ts, tgt, gim, errs, cmds


def analyze(ts, tgt, gim, errs):
    # 稳态滞后: 第一段匀速后期 (2.5~4s) 平均 |err|
    def avg_err(t0, t1):
        s = n = 0.0
        for t, e in zip(ts, errs):
            if t0 <= t < t1:
                s += e; n += 1
        return s / n if n else 0
    lag = avg_err(2.5, 4.0)          # 带符号滞后
    rms = 0.0; n = 0
    for t, e in zip(ts, errs):
        if 2.5 <= t < 4.0:
            rms += e * e; n += 1
    rms = math.sqrt(rms / n) if n else 0

    # 过冲: 第 4s 停后 1s 内最大 |err|
    over = 0.0
    for t, e in zip(ts, errs):
        if 4.0 <= t < 5.0:
            over = max(over, abs(e))
    # 整定时间: 停后首次 |err|<3px 并维持 0.5s
    settle_t = None
    good_since = None
    for t, e in zip(ts, errs):
        if t < 4.0:
            continue
        if abs(e) < 3.0:
            if good_since is None:
                good_since = t
            elif t - good_since >= 0.5:
                settle_t = good_since
                break
        else:
            good_since = None
    # 反向段稳态滞后 (10.5~13s)
    lag2 = avg_err(10.5, 13.0)
    return {
        'steady_lag': lag, 'rms_jitter': rms,
        'overshoot': over, 'settle_time': settle_t,
        'steady_lag_rev': lag2,
    }


def main():
    to, tgto, gimo, erro, cmdo = run_sim(OldCtrl)
    tn, tgtn, gimn, errn, cmdn = run_sim(NewCtrl)
    mo = analyze(to, tgto, gimo, erro)
    mn = analyze(tn, tgtn, gimn, errn)

    print("=" * 64)
    print(" 指标            优化前(v5.0)      优化后(v6.1)      改善")
    print("-" * 64)
    def row(name, a, b, fmt, unit=""):
        imp = ""
        if abs(a) > 1e-9:
            pct = (a - b) / abs(a) * 100.0
            imp = f"{(abs(pct)):.0f}%↓" if pct > 0 else f"{(-pct):.0f}%↑"
        print(f" {name:<14} {a:{fmt}}{b:{fmt}}  {imp}")
    row("稳态滞后(px)", mo['steady_lag'], mn['steady_lag'], '>8.3f')
    row("反向滞后(px)", mo['steady_lag_rev'], mn['steady_lag_rev'], '>8.3f')
    row("RMS抖动(px)", mo['rms_jitter'], mn['rms_jitter'], '>8.3f')
    row("过冲(px)", mo['overshoot'], mn['overshoot'], '>8.3f')
    st_o = "∞" if mo['settle_time'] is None else f"{mo['settle_time']-4.0:.2f}s"
    st_n = "∞" if mn['settle_time'] is None else f"{mn['settle_time']-4.0:.2f}s"
    print(f" {'整定时间':<14} {st_o:>8} {st_n:>8}")
    print("=" * 64)

    # -------- 绘图 (matplotlib 优先, 缺失回退 SVG) --------
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 7), sharex=True)
        ax1.plot(to, tgto, 'k--', lw=1.5, label='靶点 Target')
        ax1.plot(to, gimo, 'r-', lw=1.0, alpha=0.8, label='v5.0 云台')
        ax1.plot(tn, gimn, 'b-', lw=1.0, alpha=0.8, label='v6.1 云台')
        ax1.set_ylabel('位置 (px)')
        ax1.set_title('云台追踪位置对比')
        ax1.legend(loc='upper left', fontsize=8)
        ax1.grid(alpha=0.3)
        ax2.plot(to, erro, 'r-', lw=1.0, alpha=0.8, label='v5.0 误差')
        ax2.plot(tn, errn, 'b-', lw=1.0, alpha=0.8, label='v6.1 误差')
        ax2.axhline(PID_DEADZONE, color='gray', ls=':', lw=0.8)
        ax2.axhline(-PID_DEADZONE, color='gray', ls=':', lw=0.8)
        ax2.set_ylabel('跟踪误差 (px)')
        ax2.set_xlabel('时间 (s)')
        ax2.set_title('跟踪误差对比 (灰线=死区 ±2px)')
        ax2.legend(loc='upper right', fontsize=8)
        ax2.grid(alpha=0.3)
        fig.tight_layout()
        fig.savefig('pid_sim/pid_compare.png', dpi=110)
        print("\n[OK] 图表已保存: pid_sim/pid_compare.png")
    except Exception as e:
        # SVG 回退
        _svg_fallback(to, tgto, gimo, tn, gimn, erro, errn)
        print(f"\n[WARN] matplotlib 不可用 ({e}), 已生成 SVG 回退图。")


def _svg_fallback(to, tgto, gimo, tn, gimn, erro, errn):
    W, H = 900, 600
    def sx(t): return 40 + (t / 18.0) * (W - 60)
    ymin = min(min(tgto), min(gimo), min(gimn), min(erro), min(errn)) - 5
    ymax = max(max(tgto), max(gimo), max(gimn), max(erro), max(errn)) + 5
    def sy(v): return H - 40 - (v - ymin) / (ymax - ymin) * (H - 80)
    parts = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{H}">']
    parts.append(f'<rect width="{W}" height="{H}" fill="#fff"/>')
    for (xs, ys, col, lbl) in [
        (to, tgto, 'black', 'Target'), (to, gimo, 'red', 'v5.0'),
        (tn, gimn, 'blue', 'v6.1')]:
        d = 'M ' + ' L '.join(f'{sx(x):.1f},{sy(y):.1f}' for x, y in zip(xs, ys))
        parts.append(f'<path d="{d}" fill="none" stroke="{col}" stroke-width="1"/>')
    parts.append(f'<text x="10" y="20">位置对比 (红=v5.0 蓝=v6.1 黑=靶点)</text>')
    parts.append('</svg>')
    with open('pid_sim/pid_compare.svg', 'w') as f:
        f.write('\n'.join(parts))


if __name__ == '__main__':
    main()
