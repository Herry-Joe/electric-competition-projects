# electric-competition-projects

全国大学生电子设计竞赛相关项目整合仓库。该仓库用于集中归档 `Herry-Joe` 名下电赛相关代码与文档，保留各项目原有目录与资料，便于统一检索与后续维护。

> 说明：本仓库保留了目标仓库原有 `LICENSE`；各子项目的许可证以其原始仓库为准（见下表“许可证状态”）。

## 已整合子项目（含来源与归档说明）

| 子项目目录 | 原始仓库 | 主要硬件/功能（按原仓库文档） | 归档说明 | 许可证状态 |
|---|---|---|---|---|
| `k230/threshold-tracking/` | [Herry-Joe/k230-threshold-tracking](https://github.com/Herry-Joe/k230-threshold-tracking) | 庐山派 K230 + ST7701，LAB 阈值追踪/形状识别，串口坐标输出 | 已迁移原仓库代码、配置、工具与文档，保留 `README.md` | 待确认（原仓库未见独立 LICENSE 文件） |
| `k230/runner/` | [Herry-Joe/K230-RUNNER](https://github.com/Herry-Joe/K230-RUNNER) | K230/CanMV MicroPython + LVGL 跑酷游戏 | 已迁移源码、字体说明与 docs 文档 | MIT（子目录内保留原 `LICENSE`） |
| `stm32/hc05-mpu6050/` | [Herry-Joe/stm32-hc05-mpu6050](https://github.com/Herry-Joe/stm32-hc05-mpu6050) | 双 STM32F103 + HC-05 + MPU6050 姿态传输，含接收端与上位机 | 已迁移发射端/接收端/上位机目录及文档 | 待确认（原仓库未见独立 LICENSE 文件） |
| `stm32/28byj48-gimbal/` | [Herry-Joe/stm32-28byj48-gimbal](https://github.com/Herry-Joe/stm32-28byj48-gimbal) | STM32F103 + TB6612 + 28BYJ-48 双轴云台控制 | 已迁移 PlatformIO 工程、固件文件、蓝牙 App 与文档 | 待确认（原仓库未见独立 LICENSE 文件） |
| `stm32/stepper-gimbal-trace/` | [Herry-Joe/stepper-gimbal-trace](https://github.com/Herry-Joe/stepper-gimbal-trace) | 步进云台追踪相关代码，含 K230 与 STM32 ULN2003 目录、PID 仿真与项目评审文档 | 已迁移源码与评审文档，保留原有结构 | 待确认（原仓库未见独立 LICENSE 文件） |
| `integrated-systems/laser-targeting-servo-gimbal/` | [Herry-Joe/laser-targeting-servo-gimbal](https://github.com/Herry-Joe/laser-targeting-servo-gimbal) | K230 视觉 + STM32 云台控制 + 蓝牙上位机，激光打靶系统 | 已迁移 `k230`、`stm32_servo_gimbal`、`bluetooth_app` 及文档 | 待确认（原仓库未见独立 LICENSE 文件） |

## 目录结构

```text
electric-competition-projects/
├── LICENSE
├── README.md
├── k230/
│   ├── threshold-tracking/
│   └── runner/
├── stm32/
│   ├── hc05-mpu6050/
│   ├── 28byj48-gimbal/
│   └── stepper-gimbal-trace/
└── integrated-systems/
    └── laser-targeting-servo-gimbal/
```

## 使用说明

1. 按子项目进入对应目录，优先阅读该目录内 `README.md` 与硬件接线说明。  
2. K230 项目请使用对应固件/运行环境（CanMV/MicroPython 等）；STM32 项目按各自工程方式（HAL/PlatformIO 等）编译烧录。  
3. 集成类项目（如 `integrated-systems/laser-targeting-servo-gimbal/`）需同时准备视觉端与控制端硬件，并按其文档完成串口/蓝牙联调。  
4. 本仓库为归档整合入口；若需追溯历史提交、Issue 或原始上下文，请跳转到“原始仓库”链接查看。

## 迁移边界说明

- 已尽量完整迁移各源仓库中的有效源码、配置、资源与文档。  
- 未迁移会导致嵌套仓库或与当前仓库冲突的 Git 元数据（如 `.git`）以及 GitHub Actions 元数据（`.github/`）。  
- 各子项目原有 README 的相对链接已检查，在当前目录布局下可用。  
