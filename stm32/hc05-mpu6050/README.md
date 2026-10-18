# STM32F103 双板蓝牙 MPU6050 姿态传输

两块 STM32F103C8T6（BluePill）通过 HC-05 蓝牙模块互传 MPU6050 姿态数据，接收板 OLED 实时显示 roll/pitch/yaw，PC 上位机可选。

## 硬件

| 组件 | 数量 | 备注 |
|------|------|------|
| STM32F103C8T6 BluePill | 2 | 发射板 + 接收板 |
| HC-05 蓝牙模块 | 2 | 主从一体，支持 AT 指令 |
| MPU6050 六轴传感器 | 1 | 接发射板 I2C |
| 0.96" OLED (SSD1306) | 1 | 接接收板 I2C |
| USB-TTL 转串口 | 1 | 一次性配 HC-05 用 |

## 接线

### 发射板（mpu6050/）

| STM32 | 外设 |
|-------|------|
| PA2 | HC-05 RXD |
| PA3 | HC-05 TXD |
| PB6 | OLED SCL / MPU6050 SCL |
| PB7 | OLED SDA / MPU6050 SDA |
| PC13 | 板载 LED（状态指示） |
| 3.3V / GND | HC-05 VCC / GND |

### 接收板（nrf24_receiver/）

| STM32 | 外设 |
|-------|------|
| PA2 | HC-05 RXD |
| PA3 | HC-05 TXD |
| PB6 | OLED SCL |
| PB7 | OLED SDA |
| PA9 | 调试串口 TX（可选，115200 输出诊断信息） |
| 3.3V / GND | HC-05 VCC / GND |

> PA2↔HC-05 RXD、PA3↔HC-05 TXD，两板一致。

## HC-05 配置 —— 方案 A（免 BIND，电赛首选）

**原理**：两块 HC-05 成功配对一次后，模块自动记忆上次连接地址，之后上电自动重连。不需要 `AT+BIND`，规避 BIND 指令的各种兼容性 bug。

**前提**：HC-05 单独接 USB-TTL（TX↔RX 交叉、GND 共地、VCC 3.3V），按 KEY 键上电进 AT 模式（灯慢闪约 2 秒一次），串口助手 38400 波特率。

### 主机（发射板）

```
AT               ; 回 OK，确认进 AT 模式
AT+ROLE=1        ; 设为主机
AT+CMODE=1       ; 连任意从机（免 BIND）
AT+PSWD=1234     ; 配对密码，主从必须一致（默认 1234）
AT+RESET         ; 重启生效（部分模块静默，拔电等效）
```

### 从机（接收板）

```
AT               ; 回 OK
AT+ROLE=0        ; 设为从机
AT+CMODE=1       ; 可被任意主机连接
AT+PSWD=1234     ; 与主机一致
AT+RESET
```

配置掉电保存，一次配好永久生效。配完把 HC-05 插回 STM32，正常上电（不按 KEY）即为透明透传模式。

**首次配对**：两板都上电后，从机 HC-05 会自动被主机搜到并连接（蓝灯常亮即为成功）。这次配对地址会被双方记忆，以后上电自动回连。

### 关键注意事项

- **上电时不要按 HC-05 的 KEY 键**（KEY 拉高会进 AT 模式，无法透传数据）。
- HC-05 透传默认波特率 9600，固件已匹配（`JDY31_BAUD = 9600`）。
- `AT+CMODE=1` 让主机连接任意从机，无需查 MAC、无需 BIND。现场只有这一对 HC-05 时，不会误连。
- `AT+UART=9600,1,0` 部分克隆模块静默无响应，可跳过——默认就是 9600。
- `AT+RESET` 同样可能静默，直接拔 USB-TTL 断电即等效重启。

## 固件

### 编译

每个板子目录是独立的 PlatformIO 工程：

```bash
cd mpu6050    # 或 nrf24_receiver
pio run
```

### 预编译固件（立创 Web Flasher 可直接烧录）

| 板子 | 文件 |
|------|------|
| 发射板 | `mpu6050/firmware_tx.txt` |
| 接收板 | `nrf24_receiver/firmware_rx.txt` |

烧录后正常上电（不按 HC-05 KEY），发射板 OLED 显示 `S:xxxx`（发送计数递增），接收板 OLED 显示姿态读数。

## PC 上位机（可选）

```bash
cd pc_host
pip install -r requirements.txt
python imu_plot.py
```

接收板通过 PA9（USART1@115200）输出姿态数据，上位机实时绘图。

## 目录结构

```
├── README.md               # 本文件
├── mpu6050/                 # 发射板 PlatformIO 工程
│   ├── platformio.ini
│   ├── src/main.c           # 主程序：MPU6050 + 姿态解算 + 蓝牙发送
│   ├── src/jdy31.c/h        # 串口透传驱动（USART2）
│   ├── firmware_tx.txt      # 预编译固件（TI-TXT）
│   └── firmware_tx.hex      # 预编译固件（Intel HEX）
├── nrf24_receiver/          # 接收板 PlatformIO 工程
│   ├── platformio.ini
│   ├── src/main.c           # 主程序：蓝牙接收 + OLED 显示
│   ├── src/jdy31.c/h        # 串口透传驱动（USART2）
│   ├── firmware_rx.txt      # 预编译固件（TI-TXT）
│   └── firmware_rx.hex      # 预编译固件（Intel HEX）
└── pc_host/                 # PC 上位机
    ├── imu_plot.py
    └── requirements.txt
```

## 帧格式

USART2 透明透传，每帧 18 字节：

```
[0xAA][0x55] + attitude_pkt_t (16 字节)
```

`attitude_pkt_t` = `float roll` + `float pitch` + `float yaw` + `float temperature`（小端，与旧 nRF24 协议完全一致，上位机无需修改）。

## 常见问题

**Q: 蓝灯快闪不常亮（连不上）**
- 确认主从都配了 `AT+CMODE=1` 且 `AT+PSWD` 一致
- 第一次配对需要两板上电后等几秒
- 手机搜蓝牙，确认两个 HC-05 都在广播/可见

**Q: 蓝灯常亮但 OLED 无数据**
- 确认两块板都烧的是正常透传固件（`firmware_tx` / `firmware_rx`），不是配置工具
- 确认上电时没按 HC-05 的 KEY 键
- 接收板 PA9 接串口助手 115200，看 `[pkts=N]` 是否递增

**Q: OLED 显示重叠**
- 已修复（v2），接收板 `firmware_rx.txt` 是修复版
