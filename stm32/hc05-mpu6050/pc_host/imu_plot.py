#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ICM45686 姿态数据 PC 上位机（接收板 -> USART1 串口 -> 本程序）
说明: 接收板通过 USART1(PA9) 打印 ASCII 姿态行，本程序只解析这些文本行，
      与板间蓝牙链路(JDY-31/HC-05, USART2)无关，换成蓝牙后照常可用。
依赖: pip install -r requirements.txt

用法:
    python imu_plot.py COM3        # Windows，串口号按需修改
    python imu_plot.py /dev/ttyUSB0 # Linux

接收端 BluePill 以 115200 波特打印如下格式的文本行:
    R:+12.34 P:-4.56 Y:1.23 T:25.10
本程序解析后用 matplotlib 实时绘制 Roll/Pitch/Yaw 曲线。
"""
import re
import sys
import serial
import matplotlib.pyplot as plt
from collections import deque

PORT = sys.argv[1] if len(sys.argv) > 1 else "COM3"
BAUD = 115200
WINDOW = 300  # 显示最近多少个采样点

PAT = re.compile(r"R:([-\d.]+)\s+P:([-\d.]+)\s+Y:([-\d.]+)\s+T:([-\d.]+)")

roll = deque([0.0] * WINDOW, maxlen=WINDOW)
pitch = deque([0.0] * WINDOW, maxlen=WINDOW)
yaw = deque([0.0] * WINDOW, maxlen=WINDOW)

plt.ion()
fig, ax = plt.subplots(figsize=(10, 5))
ln_r, = ax.plot(roll, label="Roll")
ln_p, = ax.plot(pitch, label="Pitch")
ln_y, = ax.plot(yaw, label="Yaw")
ax.set_ylim(-180, 180)
ax.set_title("Attitude from ICM45686 via NRF24L01")
ax.set_xlabel("sample")
ax.set_ylabel("degree")
ax.legend(loc="upper right")
ax.grid(True)

print(f"Listening on {PORT} @ {BAUD} ... (Ctrl+C to exit)")
try:
    with serial.Serial(PORT, BAUD, timeout=1) as ser:
        while True:
            line = ser.readline().decode(errors="ignore").strip()
            if not line:
                continue
            m = PAT.search(line)
            if not m:
                continue
            r = float(m.group(1))
            p = float(m.group(2))
            y = float(m.group(3))
            roll.append(r)
            pitch.append(p)
            yaw.append(y)
            ln_r.set_ydata(roll)
            ln_p.set_ydata(pitch)
            ln_y.set_ydata(yaw)
            ax.set_xlim(0, WINDOW - 1)
            plt.pause(0.01)
except KeyboardInterrupt:
    print("\nbye")
except serial.SerialException as e:
    print(f"无法打开串口 {PORT}: {e}")
