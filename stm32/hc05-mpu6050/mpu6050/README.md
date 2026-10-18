# MPU6050 二维舵机云台

基于 STM32F103C8T6 + MPU6050 + SG90 舵机的姿态感知二轴云台系统。

## 功能

- 实时检测模块姿态（Roll / Pitch / Yaw + 温度）
- 0.96寸 OLED 实时显示姿态角
- 两个 SG90 舵机跟随模块姿态旋转
  - **俯仰舵机**：前后倾斜 → 上下
  - **偏航舵机**：水平旋转 → 左右
- 上电自动陀螺仪零偏校准

## 硬件接线

| 功能 | STM32 引脚 | 说明 |
|------|------------|------|
| OLED VCC | PB6 | 推挽输出高电平供电 |
| OLED GND | PB7 | 推挽输出低电平 |
| OLED SCL | PB8 | 软件I2C1 |
| OLED SDA | PB9 | 软件I2C1 |
| MPU6050 SCL | PB10 | 软件I2C2 |
| MPU6050 SDA | PB11 | 软件I2C2 |
| MPU6050 VCC | 3.3V | GY-521 模块自带稳压 |
| MPU6050 GND | GND | 共地 |
| 偏航舵机信号 | PA6 | TIM3 CH1 PWM |
| 俯仰舵机信号 | PA7 | TIM3 CH2 PWM |
| 舵机 VCC | 5V | 需独立5V供电 |
| 舵机 GND | GND | 共地 |

## 项目结构

```
mpu6050/
├── platformio.ini      # PlatformIO 构建配置
├── README.md           # 本文档
└── src/
    ├── main.c           # 主程序：初始化 + 主循环
    ├── myi2c.c/h        # 双路软件I2C驱动（I2C1 OLED + I2C2 MPU6050）
    ├── mpu6500.c/h      # MPU6050/MPU6500 传感器驱动
    ├── attitude.c/h     # 互补滤波姿态解算
    ├── servo.c/h        # SG90 舵机 PWM 控制（TIM3）
    ├── oled_draw.c/h    # OLED 帧缓冲图形驱动
    ├── OLED_Font.h      # 8x16 字体声明
    └── font_data.c      # 字体数据定义
```

## 编译与烧录

```bash
# 编译
pio run

# 烧录
pio run --target upload

# 串口监视器
pio device monitor
```

## 关键参数

| 参数 | 值 |
|------|-----|
| 系统时钟 | 72MHz (HSE + PLL×9) |
| OLED | 128×64 SSD1306 |
| MPU6050 量程 | 陀螺±250dps / 加速度±2G |
| DLPF 带宽 | 44Hz |
| 采样率 | 200Hz |
| 舵机 PWM | 50Hz, 0.5ms~2.5ms |
| 姿态解算周期 | ~50ms (20Hz) |

## 姿态解算

互补滤波器融合加速度计和陀螺仪：

```
angle = 0.98 × (angle + gyro × dt) + 0.02 × accel_angle
```

- 陀螺仪：短期精确，会漂移
- 加速度计：长期稳定，受振动影响
- 互补滤波：取两者优点

**注意**：无磁力计，Yaw 会缓慢漂移（约0.5°/分钟）。

## 舵机控制

| 动作 | 传感器数据 | 舵机 |
|------|-----------|------|
| 前后倾斜（点头） | Roll | 俯仰舵机上下 |
| 水平旋转（转头） | 陀螺仪Z轴角速度 | 偏航舵机左右 |

偏航使用角速度积分 + 缓慢回中算法，静止时自动回到90°。

## 常见问题

### OLED 乱码
- 初始化序列必须完整，不能省略或添加命令
- 确认 SSD1306 驱动（非 SH1106）

### MPU6050 通信失败
1. 检查 PB10(SCL)、PB11(SDA) 接线
2. 确认 3.3V 供电
3. GY-521 AD0 悬空 = 地址 0x68

### 陀螺仪数据为 0
- I2C 连续读的 ACK 逻辑必须正确：中间字节 ACK，最后字节 NACK
- MPU6050 需先复位(0x80)再唤醒(0x01)，中间等 100ms

### Roll 不变
- GY-521 模块轴向可能和数据手册不一致
- 需实测确认哪个轴随倾斜变化，在 attitude.c 中调整

### 舵机不动
- 舵机必须用独立 5V 供电，不能用 STM32 3.3V
- 检查 PA6/PA7 接线

## 参考项目

- 平衡车入门教程 — MPU6050 驱动 + 互补滤波
- kriswiner/MPU6050 — Madgwick 滤波 + 校准
- ElectronicCats/mpu6050 — DMP 固件加载
