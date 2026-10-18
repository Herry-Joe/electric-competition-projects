# STM32F103C8T6 + TB6612 驱动 28BYJ-48 双轴云台 Demo

## 项目概述

使用 STM32F103C8T6 (Blue Pill) + **两片** TB6612FNG 双 H 桥驱动模块，控制 **两台** 28BYJ-48 微型减速步进电机，实现 **双轴云台（Pan 水平 + Tilt 俯仰）** 的连续旋转 / 角度定位 / 来回扫描。

> ⚠️ **与早期版本的区别**：本项目最初为单轴（1 个 TB6612）。现已升级为双轴，需要 **2 个 TB6612** —— 每个 28BYJ-48 各占一个 TB6612 的两个 H 桥。

### 功能特性

| 功能 | 说明 |
|------|------|
| 双轴控制 | Pan（水平）+ Tilt（俯仰）独立控制，可同时运动 |
| 步进模式 | 全步进（大力矩）/ 半步进（平滑） |
| 速度控制 | 步间延时 900~20000 µs 可调（两轴独立或同时调） |
| 角度控制 | 精确旋转指定角度（半步步进分辨率约 0.088°/步） |
| 扫描模式 | 双轴自动来回摆动（±N°，到限位自动换向） |
| 按键控制 | 4 个按键：加速 / 减速 / 换向 / 模式切换（MODE 切换活动轴） |
| 串口控制 | UART 指令集，可切换 Pan/Tilt 分别控制，可接上位机/蓝牙 |
| **OLED 调试屏** | SSD1306（128×64，软件 I2C PB8/PB9）实时显示模式 / 双轴状态 / 串口命令(RX) / 回执(ACK)，免串口助手也能看调试信息 |
| **舞蹈模式 🎵** | 6 阶段编排舞蹈：热身→甩头→旋转→反旋→加速冲刺→收尾 |
| **上电空闲** | 上电不转，收到串口/蓝牙命令后才动（蓝牙不断连） |

### 硬件参数

- **28BYJ-48 步进电机（每台）**
  - 额定电压：5V DC
  - 相数：4 相（5 线单极性）
  - 减速比：1:64
  - 步距角：5.625°/64 ≈ 0.088°（半步步进）
  - 全步进一圈：2048 步（输出轴）｜半步进一圈：4096 步（输出轴）

- **TB6612FNG 电机驱动（每片）**
  - 双 H 桥，每桥持续电流 1.2A（峰值 3.2A）
  - 逻辑电压：2.7~5.5V（兼容 3.3V）
  - 电机电压：2.5~13.5V
  - 支持 CW / CCW / 短刹车 / 停止

---

## 硬件连线

### 总体接线图（双 TB6612）

```
                    ┌──────────────┐
  3.3V  ──────────▶ │ VCC          │
  5V    ──────────▶ │ VM           │
  GND   ──────────▶ │ GND   AOUT1 │────▶ 橙线 (Coil 1)
                    │        AOUT2 │────▶ 粉线 (Coil 2)
  PA0   ──────────▶ │ PWMA   BOUT1 │────▶ 黄线 (Coil 3)
  PA4   ──────────▶ │ AIN1    BOUT2 │────▶ 蓝线 (Coil 4)
  PA5   ──────────▶ │ AIN2         │
  PA1   ──────────▶ │ PWMB         │   5V ─▶ 红线 (公共端)
  PA6   ──────────▶ │ BIN1         │
  PA7   ──────────▶ │ BIN2         │
  PB0   ──────────▶ │ STBY         │
                    └──────┬───────┘
                           │ STBY 两片共用，只需接一个脚 (PB0)
                    ┌──────┴───────┐
  5V    ──────────▶ │ VM           │
  GND   ──────────▶ │ GND   AOUT1 │────▶ 橙线 (Coil 1)  [Tilt 电机]
  PB1   ──────────▶ │ PWMA   AOUT2 │────▶ 粉线 (Coil 2)
  PB3   ──────────▶ │ AIN1    BOUT1 │────▶ 黄线 (Coil 3)
  PB4   ──────────▶ │ AIN2    BOUT2 │────▶ 蓝线 (Coil 4)
  PB5   ──────────▶ │ BIN1         │   5V ─▶ 红线 (公共端)
  PB6   ──────────▶ │ BIN2         │
  PB7   ──────────▶ │ PWMB         │
                    └──────────────┘
                     TB6612 #2 (Tilt)

  按键: PB12(UP) PB13(DOWN) PB14(DIR) PB15(MODE)  ── 另一端接 GND (内部上拉)
  串口: PA9(TX) ── USB-TTL RX, PA10(RX) ── USB-TTL TX
  SWD : PA13(SWDIO) PA14(SWCLK) ── ST-Link (保留用于调试/烧录)

  JDY-31 蓝牙模块 (接 USART2):
    VCC ── 3.3V
    GND ── GND
    TXD ── PA3 (USART2_RX)
    RXD ── PA2 (USART2_TX)
    波特率 9600

  OLED 调试屏 (SSD1306, 软件 I2C PB8/PB9):
    VCC ── 3.3V
    GND ── GND
    SCL ── PB8 (软件 I2C, 位敲)
    SDA ── PB9 (软件 I2C, 位敲)
    I2C 地址 0x3C (自动探测; 0x3D 也兼容)
```

### 详细引脚对应表

**TB6612 #1（Pan / 水平电机）**

| STM32F103C8T6 | TB6612 #1 | 功能 |
|:---:|:---:|:---|
| PA0 | PWMA | A 桥 PWM（GPIO 输出高 = 全速） |
| PA1 | PWMB | B 桥 PWM（GPIO 输出高 = 全速） |
| PA4 | AIN1 | A 桥控制 1 |
| PA5 | AIN2 | A 桥控制 2 |
| PA6 | BIN1 | B 桥控制 1 |
| PA7 | BIN2 | B 桥控制 2 |

**TB6612 #2（Tilt / 俯仰电机）**

| STM32F103C8T6 | TB6612 #2 | 功能 |
|:---:|:---:|:---|
| PB1 | PWMA | A 桥 PWM（高 = 全速） |
| PB7 | PWMB | B 桥 PWM（高 = 全速） |
| PB3 | AIN1 | A 桥控制 1 |
| PB4 | AIN2 | A 桥控制 2 |
| PB5 | BIN1 | B 桥控制 1 |
| PB6 | BIN2 | B 桥控制 2 |

**公共使能与信号**

| STM32F103C8T6 | TB6612 | 功能 |
|:---:|:---:|:---|
| PB0 | STBY | 两片 TB6612 共用，高 = 工作，低 = 待机 |
| PA9 | USB-TTL RX | 调试串口 TX（115200bps） |
| PA10 | USB-TTL TX | 调试串口 RX |
| PB8 | OLED SCL | I2C1 重映射 → SSD1306 时钟线（需 4.7kΩ 上拉，模块自带则不用） |
| PB9 | OLED SDA | I2C1 重映射 → SSD1306 数据线（需 4.7kΩ 上拉，模块自带则不用） |

| TB6612 输出 | 28BYJ-48 | 说明 |
|:---:|:---:|:---|
| AOUT1 | 橙线 | 线圈 1 |
| AOUT2 | 粉线 | 线圈 2 |
| BOUT1 | 黄线 | 线圈 3 |
| BOUT2 | 蓝线 | 线圈 4 |
| — | 红线 | **直接接 5V 电源**（不经过 TB6612） |

> **供电提醒**：每台 28BYJ-48 工作电流约 200mA，两台 + 控制板建议 5V ≥ 2A 电源。TB6612 的 VM 接 5V（与电机同压）。

---

## TB6612 驱动 28BYJ-48 的原理

28BYJ-48 是**单极性（unipolar）**步进电机，5 根线中红线是线圈中心抽头（公共端），接 **+5V**。

TB6612 每个 H 桥有两个输出（OUT1/OUT2），控制真值表：

| IN1 | IN2 | OUT1 | OUT2 | 效果 |
|:---:|:---:|:---:|:---:|:---|
| 0 | 0 | 高阻 | 高阻 | 线圈断电 |
| 0 | 1 | L | H | 线圈 1 通电（OUT1→GND） |
| 1 | 0 | H | L | 线圈 2 通电（OUT2→GND） |
| 1 | 1 | L | L | 刹车（两线圈同时通电） |

**关键**：线圈一端（公共端）接 +5V，另一端接 TB6612 输出。当 OUTx = **L (0V)** 时，线圈两端有压差 → 通电励磁；当 OUTx = **H (5V)** 时，线圈两端同电位 → 无电流。通过 8 拍半步步进序列逐相切换即可驱动。本代码即利用此特性，把「哪个线圈该接 0V」映射成 AINx/BINx 的控制位。

---

## 软件架构

```
main.c
├── SystemClock_Config()    → 72MHz 系统时钟 (HSE 8MHz ×9 PLL)
├── GPIO_Init()             → 引脚初始化 (含释放 PB3/PB4 JTAG 复用)
├── USART1_Init()           → 调试串口 115200bps
├── USART2_Init()           → 蓝牙串口 9600bps
├── I2C1_Init()             → OLED I2C1 总线 (PB8/PB9 重映射)
├── TIM2_Init()             → Pan 步进节拍定时器 (1MHz/1µs 分辨率)
├── TIM3_Init()             → Tilt 步进节拍定时器
├── Stepper_Init()          → 初始化 Pan / Tilt 两个电机实例
└── while(1)
    ├── Serial_Execute()    → 主循环解析串口命令 (非 ISR)
    ├── Gimbal_KeyScan()    → 按键扫描 (50ms 消抖)
    ├── Gimbal_Dance()      → 舞蹈模式编排动作 (6 阶段)
    ├── OLED_Refresh()      → 每 200ms 刷新 SSD1306 调试屏 (无屏自动跳过)
    └── LED 心跳            → 双轴都空闲时 500ms 慢闪

中断:
├── TIM2_IRQHandler         → Stepper_TIM_IRQHandler(Pan)  推进一拍
├── TIM3_IRQHandler         → Stepper_TIM_IRQHandler(Tilt) 推进一拍
├── USART1_IRQHandler       → 仅缓冲字节, 置 s_cmd_ready 标志 (命令在主循环处理)
└── SysTick_Handler         → HAL 时基
```

### 云台工作模式（按键 MODE 切换）

| 模式 | 含义 | 按键行为 |
|:---:|:---|:---|
| **Pan 手动** | 控制水平电机 | UP 加速 / DOWN 减速 / DIR 换向 |
| **Tilt 手动** | 控制俯仰电机 | UP 加速 / DOWN 减速 / DIR 换向 |
| **双轴扫描** | Pan + Tilt 同时 ±90° 来回摆动 | UP/DOWN 同时调两轴速度 |
| **串口模式** | 上位机完全接管 | 按键无效，等待串口命令 |
| **舞蹈模式 🎵** | 6 阶段编排舞蹈，约 17s 自动收尾 | UP/DOWN 调整体速度，MODE 提前退出 |

> 上电默认进入 **串口控制模式**：电机空闲，收到串口/蓝牙命令后才动。按键 MODE 可循环切换 Pan→Tilt→Sweep→Serial→Dance。

### 步进序列（半步步进 8 拍，默认）

```
拍号  | IN1A/IN2A | IN1B/IN2B | 励磁线圈
-----|-----------|-----------|----------
  0  |   0,1     |   0,0     | 橙
  1  |   0,1     |   0,1     | 橙 + 黄
  2  |   0,0     |   0,1     | 黄
  3  |   1,0     |   0,1     | 粉 + 黄
  4  |   1,0     |   0,0     | 粉
  5  |   1,0     |   1,0     | 粉 + 蓝
  6  |   0,0     |   1,0     | 蓝
  7  |   0,1     |   1,0     | 蓝 + 橙
```

---

## 编译与烧录（VS Code + PlatformIO）

### 1. 安装必备工具

| 工具 | 下载 |
|------|------|
| VS Code | https://code.visualstudio.com/ |
| PlatformIO 插件 | VS Code 扩展商店搜索 `PlatformIO IDE` |

> PlatformIO 会自动下载 arm-none-eabi-gcc 工具链和 STM32CubeF1 HAL 框架，无需手动安装。

### 2. 打开项目

```
VS Code → 文件 → 打开文件夹 → 选择 stm32_tb6612_28byj48_demo/
```

PlatformIO 自动识别 `platformio.ini` 并配置编译环境（首次打开会下载依赖，约 3~5 分钟）。

### 3. 编译

点击 VS Code 底部状态栏的 **✓ (Build)** 按钮，或终端执行：

```bash
pio run
```

编译产物在 `.pio/build/bluepill_f103c8/`：`firmware.elf` / `firmware.hex` / `firmware.bin`。

### 4. 烧录

**方式一：ST-Link（推荐，已设 CPUTAPID 兼容克隆版）**

```bash
pio run -t upload
```

> 若使用淘宝克隆版 ST-Link 报 `init mode failed`，`platformio.ini` 已加如下配置兼容：
> ```ini
> upload_flags =
>     -c
>     set CPUTAPID 0
> ```
> 仍失败时见下方「烧录排错」。

**方式二：串口 ISP（USB-TTL，免 ST-Link）**

1. 改 `platformio.ini`：
   ```ini
   upload_protocol = serial
   upload_port = COM3        ; 改成你的串口号
   ```
2. BOOT0 跳线帽接 **1**（高电平），BOOT1 接 **0**，按 RESET
3. 点 Upload；烧录完把 BOOT0 跳回 **0**，再按 RESET 启动

### 5. 串口监视

```bash
pio device monitor
```
波特率 115200，可同时看电机输出与发命令（已开启回显）。

---

## 操作说明

### 上电行为

上电后 **所有电机保持空闲**（线圈不通电），等待串口/蓝牙命令：
- 收到 `c` / `a` / `+N` / `-N` / `rN` / `d` 等命令电机开始转动
- 命令执行完成后停止并保持力矩（云台不下垂）
- 蓝牙 JDY-31 模块上电稳定后才通电，不会因电机浪涌电流断连

### 按键操作

| 按键 | 引脚 | 功能 | 说明 |
|------|:---:|------|------|
| **UP** | PB12 | 加速 | 步间延时 -200µs（若已停则启动） |
| **DOWN** | PB13 | 减速 | 步间延时 +200µs |
| **DIR** | PB14 | 换向 | CW ↔ CCW |
| **MODE** | PB15 | 模式切换 | Pan → Tilt → 双轴扫描 → 串口 → **舞蹈** → Pan |

> 在 Pan / Tilt 手动模式下，按键只作用于「当前活动轴」；扫描模式下 UP/DOWN 同时调两轴速度。

### 串口命令（115200bps，8N1，回车结束）

命令先选择目标电机（`p` / `t`），再发动作指令：

| 命令 | 功能 | 示例 |
|------|------|------|
| `p` | 选中 Pan（水平）电机 | `p` |
| `t` | 选中 Tilt（俯仰）电机 | `t` |
| `f` | 全步步进模式 | `f` |
| `h` | 半步步进模式 | `h` |
| `c` | 顺时针 (CW) | `c` |
| `a` | 逆时针 (CCW) | `a` |
| `s` | 停止 | `s` |
| `+N` | 前进 N 步 | `+2048`（转一圈） |
| `-N` | 后退 N 步 | `-512` |
| `rN` | 旋转 N 度 | `r90`（转 90°） |
| `wN` | 扫描模式 ±N 度 | `w180`（±90° 扫描） |
| `?` | 查询两轴状态 | `?` |

> 例：`t r45` → Tilt 电机转 45°；`p w90` → Pan 电机 ±45° 扫描。

### OLED 调试屏（辅助串口调试）

接上 SSD1306（128×64，I2C 地址 0x3C 或 0x3D，PB8=SCL、PB9=SDA）后，屏幕上电即显示当前状态，**无需打开串口助手也能看到系统运行情况**：

> **接线**：OLED 用软件 I2C（位敲方式，和你项目中 U8g2 库一样可靠），PB8/PB9 为开漏输出需外部 4.7kΩ 上拉到 3.3V（多数模块自带上拉则免）。VCC→3.3V（部分模块需 5V）。

```
== 28BYJ-48 GIMBAL ==
MODE:SERIAL
PAN :RUN CW MAN 2.0ms
TILT:SWP CCW 2.0ms
SEL:PAN
RX:c
ACK:OK
UPT:12.3s
```

| 行 | 含义 |
|----|------|
| `MODE:` | 当前云台模式：PAN / TILT / SWEEP / SERIAL / DANCE |
| `PAN:` / `TILT:` | 该轴状态：`RUN/IDLE/STEP/ANG` 运行状态、`CW/CCW` 方向、`MAN/SWP/SER` 工作模式、当前速度（ms/步） |
| `SEL:` | 串口当前选中的电机（Pan 或 Tilt） |
| `RX:` | 最近一次收到的串口命令（自动转大写显示） |
| `ACK:` | 最近一次命令处理结果：`OK` / `ERR` |
| `UPT:` | 系统已运行时间（秒） |

> 这正是「串口收到了但没反馈」的克星：发命令后 `RX` 立刻显示你发了什么、`ACK` 显示 `OK`/`ERR`，配合板载 LED 闪一下，**命令有没有被正确执行一目了然**。
>
> OLED 缺省用 `HAL_I2C_IsDeviceReady` 探测，没接屏幕时自动跳过、不影响电机与串口功能；接了屏幕但显示乱码，先检查 I2C 地址（0x3C 还是 0x3D）和 4.7kΩ 上拉电阻。

---

### 舞蹈模式 🎵

按 MODE 切换到舞蹈模式（Pan → Tilt → 扫描 → 串口 → **舞蹈**），云台按编排自动表演约 17 秒：

| 阶段 | 名字 | 时长 | 动作 |
|:---:|:---|:---:|:---|
| 0 | 🔥 **热身** | 2s | 双轴慢速同向 |
| 1 | 🤯 **甩头** | 3s | 双轴每 350ms 快速换向抖动 |
| 2 | 🔄 **旋转** | 3s | Pan 极限速度狂转 + Tilt 上扬扫描 |
| 3 | 🔄 **反旋** | 3s | Pan 反向狂转 + Tilt 低头扫描 |
| 4 | ⚡ **加速冲刺** | 4s | 从 2.5ms 逐渐加速到 1ms，交错换向 |
| 5 | 🌊 **缓步收尾** | 2s | 慢速同向，自动切回 Pan 手动模式 |

> 舞蹈中按 **UP/DOWN** 可调整体速度，按 **MODE** 提前退出。

---

## 蓝牙控制 (JDY-31)

STM32 的 **USART2（PA2/PA3）** 接 JDY-31 蓝牙模块，波特率 9600，复用同一套串口命令协议。USART1（PA9/PA10）仍留给 USB-TTL 调试，两者都能控制云台。

### 接线

```
JDY-31          STM32F103C8T6
VCC   ────────  3.3V
GND   ────────  GND
TXD   ────────  PA3 (USART2_RX)
RXD   ────────  PA2 (USART2_TX)
```

> ⚠️ JDY-31 是 **3.3V** 模块，不要接 5V。默认波特率 9600（可用 AT 指令改）。

### 电脑上位机 (Web Serial，推荐)

JDY-31 是 **SPP 经典蓝牙**（不是 BLE），不能用 Web Bluetooth。本项目的 `bluetooth_app/webserial.html` 改用 **Web Serial API**，电脑版 Chrome / Edge 可直接连蓝牙配对出的 COM 口：

```
bluetooth_app/
├── webserial.html  # Web Serial 控制面板 (JDY-31 SPP, 电脑 Chrome/Edge 用)
├── index.html      # Web Bluetooth 面板 (仅适用于 BLE 模块, 如 JDY-31-BLE)
└── README.md       # 使用说明
```

**使用步骤：**
1. Windows 蓝牙配对 JDY-31（密码 `1234`），设备管理器 → 端口会出现一个 COM 口
2. 因为 Web Serial 需要安全上下文，不能直接双击打开 file://，要起本地服务：
   ```bash
   cd bluetooth_app
   python -m http.server 8000
   ```
3. 浏览器打开 `http://localhost:8000/webserial.html`
4. 点「连接串口」→ 选蓝牙 COM 口 → 即可遥控（方向键 / 速度 / 跳舞 `d`）

> 若不想用网页，也可用任意串口助手（SSCOM / 友善串口 / Putty）连 COM 口，波特率 9600，直接发命令（如 `p c`、`t r90`、`d`）。

### 手机端

JDY-31 为 SPP 经典蓝牙，手机的 Web Bluetooth 不可用。安卓需安装 **SPP 串口类 App**（如 *Serial Bluetooth Terminal*），连上 JDY-31 后发同样的命令即可。`bluetooth_app/index.html`（Web Bluetooth）仅适用于 **BLE 模块**（JDY-31-BLE 等），对当前 SPP 模块无效。

> 跳舞模式现在也可以通过蓝牙发送 `d` 触发，不依赖物理 MODE 键。

---

## 调试与排错

### 运行类

| 现象 | 可能原因 | 解决方案 |
|------|----------|----------|
| 电机不转 | STBY 没拉高 | 检查 PB0 是否输出高（代码已默认拉高） |
| 电机不转 | VM 供电不足 | 两台 28BYJ-48 共约 400mA，5V 电源需 ≥ 2A |
| 电机抖动不转 | 4 根线圈线序错 | 按橙/粉/黄/蓝 ↔ AOUT1/AOUT2/BOUT1/BOUT2 核对 |
| 电机发热 | 步间延时太小 | 按 DOWN 降速 |
| 丢步 / 转不到角度 | 转速过快或负载大 | 用全步进（`f`）或降速 |
| Tilt 电机无反应 | PB3/PB4 仍被 JTAG 占用 | 确认已执行 SWJ 复用释放（见下方「注意事项」） |

### 烧录类（ST-Link 连不上）

典型报错：`Error: init mode failed (unable to connect to the target)`

排查顺序：
1. **接线**：ST-Link 的 `SWCLK→PA14`、`SWDIO→PA13`、`GND→GND`；若 Blue Pill 不靠 USB 供电，ST-Link `3.3V→板 3.3V`。
2. **板子有电**：Blue Pill 的 PWR LED 是否亮。
3. **排针虚焊**：Blue Pill 排针常虚焊，重点查 SWCLK/SWDIO。
4. **克隆版兼容**：`platformio.ini` 已加 `CPUTAPID 0`，仍失败可改 `upload_protocol = cmsis-dap` 或用串口 ISP。
5. **按住 RESET 再点 Upload，松手瞬间连接**。
6. **终极方案**：串口 ISP（BOOT0=1），完全不依赖 ST-Link。

---

## 注意事项（已知坑）

### 1. PB3 / PB4 是 JTAG 引脚，需释放才能当 GPIO

Tilt 电机控制用了 PB3~PB6，其中 **PB3（JTDO）、PB4（JNTRST）** 默认是 JTAG 调试引脚。代码在 `GPIO_Init()` 中调用了：

```c
__HAL_AFIO_REMAP_SWJ_NOJTAG();   /* 仅关 JTAG, 保留 SWD (PA13/PA14) 可继续烧录/调试 */
```

⚠️ **注意**：早期版本用的是 `__HAL_AFIO_REMAP_SWJ_DISABLE()`，该宏会**同时关闭 JTAG 和 SWD**，烧录一次后再上电 SWD 将无法连接（除非用「按住 RESET 烧录」）。现已改为 `NOJTAG`——同样释放了 PB3/PB4 给 Tilt 电机，但**保留 SWD 调试接口不受影响**，可随时用 ST-Link 再烧录。如无特殊需要，请保持 `NOJTAG`。

### 2. 编译错误已修复（早期踩坑记录）

| 问题 | 原因 | 处理 |
|------|------|------|
| `HSE_VALUE` / `HSI_VALUE` 重定义 | 框架 `stm32f1xx_hal_conf.h` 已定义 | 已从 `main.h` 删除，仅保留 `SYSCLK_FREQ` 等自用宏 |
| `htim2` / `htim3` 未声明 | 在 `main.c` 定义，`stepper.c` 看不到 | 在 `main.h` 加 `extern` 声明 |
| `fabsf` / `atoi` 隐式声明 | 缺头文件 / nano 库链接问题 | 改为条件判断 + `#include <stdlib.h>`，避免依赖 `math.h` |

### 3. 串口命令解析移到主循环

为避免在 USART 中断里调用 `printf` 造成阻塞/重入，串口 **仅缓冲字节**（ISR 中），命令的解析与 `printf` 输出放在 `while(1)` 主循环的 `Serial_Execute()` 中执行。

### 4. newlib syscall 桩函数

项目链接了 newlib-nano，但它会尝试引用 `_read`、`_lseek`、`_fstat`、`_isatty`、`_close` 等底层 syscall。代码中提供了这些函数的空桩，避免链接警告。不影响功能。

### 5. ST-Link 烧录

**① 假阳性（可忽略）**
OpenOCD 烧录完成后退出时偶尔会报 `*** [upload] Error 1`，但实际输出中有 `** Programming Finished **` + `** Verified OK **`，说明已烧录成功。这是 OpenOCD 复位退出码的假阳性，不影响代码运行。

**② 真错误（必须修）：`invalid command name "cortex_m"`**
若日志出现 `none separate` 之后紧跟 `invalid command name "cortex_m"`，说明是 **OpenOCD 0.12（PlatformIO `tool-openocd@3.1200.0`）** 的兼容问题——该版本已**删除** `cortex_m reset_config` 这个 Tcl 命令。
- 旧版 `platformio.ini` 里为了解决复位失败而写的 `upload_flags = ... -c "cortex_m reset_config sysresetreq"` 在 0.12 会直接解析报错、导致烧录根本无法开始。
- 修复方法：删掉这一行即可。ST-Link 走 **HLA 模式**，复位默认就走内核软件复位 `SYSRESETREQ`，无需也不能再写 `cortex_m` 命令。当前 `platformio.ini` 仅保留 `-c "reset_config none"`（不依赖硬件 NRST 脚）。
- 验证：本地跑 `openocd -f interface/stlink.cfg -f target/stm32f1x.cfg -c "reset_config none" -c shutdown`，无 `cortex_m` 报错即说明配置正确。

---

## 文件结构

```
stm32_tb6612_28byj48_demo/
├── platformio.ini              # PlatformIO 配置 (board=bluepill_f103c8, stlink+CPUTAPID)
├── include/
│   ├── main.h                  # 主头文件 (引脚/宏/extern 句柄)
│   ├── stepper.h               # 步进电机驱动接口 (支持多实例)
│   ├── ssd1306.h               # SSD1306 OLED 驱动接口 (I2C)
│   ├── oled_debug.h            # OLED 调试界面接口
│   └── stm32f1xx_hal_conf.h    # HAL 库配置 (精简)
├── src/
│   ├── main.c                  # 主程序 (双轴/模式/舞蹈/蓝牙/OLED/中断/printf重定向)
│   ├── stepper.c               # 步进驱动 (步进序列/速度/角度/扫描/软加速)
│   ├── ssd1306.c               # SSD1306 OLED 驱动 (软件 I2C 位敲, PB8/PB9)
│   └── oled_debug.c            # OLED 调试界面 (状态/串口RX/ACK 显示)
├── bluetooth_app/
│   ├── webserial.html          # Web Serial 控制面板 (JDY-31 SPP, 电脑 Chrome/Edge)
│   ├── index.html              # Web Bluetooth 面板 (仅适用于 BLE 模块)
│   └── README.md               # App 使用说明
├── .gitignore
└── README.md                   # 本文件
```

> HAL 驱动库、CMSIS、启动文件、链接脚本均由 PlatformIO 框架自动管理（在 `.pio/` 目录）。

---

## 更新记录

- **v6 (软件 I2C + 全面修稳)**：
  - OLED 从硬件 I2C1 重映射改**软件 I2C 位敲**（PB8/PB9，和 U8g2 库同样可靠），解决 I2C1 重映射兼容问题
  - 修复 USART1/USART2 **RXNE 中断未开启**关键 bug（之前命令发过去收不到）
  - 上电电机**线圈不通电**→蓝牙不再因电流浪涌断连；收到串口命令才通电运转
  - 电机控制优化：默认速度避开 28BYJ-48 共振区（2000µs→3000µs）、8 步软加速启停更平稳、停止保持力矩不下垂
  - 清理 `platformio.ini`：删除 OpenOCD 0.12 不兼容的 `cortex_m reset_config` 和有害的 `reset_config none`
  - SWJ 复用从 `DISABLE` 改 `NOJTAG`（保留 SWD 调试接口）
  - `webserial.html` 全面升级：**键盘快捷键**（方向键/WASD/Space/数字键调速度）、**自定义命令输入框**、**# OK/# ERR 彩色高亮**、轴状态指示
  - 新增 `.workbuddy/` 到 `.gitignore`

- **v5 (OLED 调试屏)**：新增 SSD1306（128×64）OLED 通过 **I2C1 重映射 PB8/PB9** 接入，实时显示云台模式 / Pan·Tilt 状态速度方向 / 串口选中轴 / 最近命令(RX) / 回执(ACK) / 运行时间，解决「串口收到了但没反馈」问题；`GPIO_Init()` 的 SWJ 复用由 `DISABLE` 改为 `NOJTAG`（保留 SWD 可继续烧录）。
- **v4 (蓝牙)**：USART2（PA2/PA3）接 JDY-31 蓝牙模块（9600bps），复用串口命令协议；新增 `bluetooth_app/index.html` Web 蓝牙控制面板；串口命令新增 `vN`(设速度) 和 `d`(舞蹈)；状态信息同时发到 USART1/USART2。
- **v4.1 (蓝牙修正)**：JDY-31 实为 SPP 经典蓝牙，Web Bluetooth 连不上；改用 **Web Serial API** 新增 `bluetooth_app/webserial.html`，电脑 Chrome/Edge 连蓝牙 COM 口即可遥控。原 `index.html` 仅适用于 BLE 模块。
- **v3 (舞蹈)**：新增 6 阶段舞蹈模式（Gimbal_Dance）；Tilt 改为 270° 限位扫描保护机械结构；双轴上电自传；补 newlib syscall 桩函数清链接警告；清理 `platformio.ini` 配置；ST-Link `upload_flags` 格式修正；OpenOCD 假阳性说明。
- **v2 (双轴)**：升级为 Pan + Tilt 双轴云台，新增第二个 TB6612 与 TIM3；按键 MODE 切换活动轴与扫描模式；串口支持 `p`/`t` 选轴；上电自传 Pan 电机；串口命令解析移出 ISR。
- **v1→PIO**：从 Makefile 迁移到 VS Code + PlatformIO 编译。
- **修复**：HSE/HSI 重定义、htim2/htim3 extern、fabsf/atoi 编译错误；`platformio.ini` 增加 `CPUTAPID 0` 兼容克隆 ST-Link。

---

## 扩展方向

- **闭环定位**：加角度传感器（电位器 / 编码器 / MPU6050）做位置闭环。
- **蓝牙遥控**：接 HC-05 替代有线串口。
- **S 曲线加减速**：减少高速丢步。
- **微步进**：用 TB6612 PWM 输出正弦波细分。
- **三轴**：再加一个 TB6612 + 电机做 Roll 轴。

---

## 许可

本项目仅供学习与电赛培训使用。
