# 特别调试手册 — 脱机阈值追踪 v4.0

面向电赛现场调试的速查。程序已在 v4.0 内置多种调试手段，目标是**不接 IDE 也能定位问题**。

---

## 1. 调试 HUD（按 `DBG` 开关）

屏幕底部出现青色状态行（再次按 `DBG` 关闭）：

```
FPS:30.5  MEM:812kB
TX:1234  ERR:0  UART:OK
LST:(432,210)
thr L[20~80] A[-30~40] B[-50~60]
```

| 行 | 含义 | 用途 |
|----|------|------|
| FPS | 实时帧率（指数平滑） | <15 说明检测/绘制偏重，可缩 ROI、关 SHAPE |
| MEM | `gc.mem_free()` 剩余内存(kB) | 持续下降→内存泄漏，留意大对象/未及时释放 |
| TX / ERR | 串口累计发送 / 失败次数 | ERR 增长→串口未初始化或写异常（UART:NO 表示 uart 为 None） |
| LST | 最近一次发送的坐标 | 与 STM32 收到值核对，确认坐标映射正确 |
| thr | 当前 6 通道 LAB 阈值 | 与 MASK 编辑器对照，确认已 Save |

> Save 成功后 1.5 秒内会追加一行 `CFG SAVED OK`，确认落盘。

---

## 2. 串口链路自检（按 `TEST`）

- K230 发一帧 `FLAG.bit3=1` 的自检帧，坐标填 `(0x7FFF, 0x7FFF)`。
- 用 PC 端 `tools/uart_link_test.py` 监听实际接到 PC 的那路串口（K230 直连 PC，或 STM32 把数据回传）：
  ```bash
  pip install pyserial
  python tools/uart_link_test.py COMx 115200
  ```
- 收到 `[TEST] x=32767 y=32767 ... crc=OK` 即通路正常。
- 若 ERR 计数增长而 TX 不动 → `UART2` 初始化失败，检查 GPIO05/06 复用与接线。

---

## 3. 配置持久化（按 `Save`）

- 当前 `thresholds` + `roi` 写入 SD 卡 `/sdcard/threshold_config.json`（ujson 格式）。
- 每次开机 `load_config()` 自动读取；文件不存在则用出厂默认（`config/threshold_default.json` 同值）。
- 现场标定一次后落盘，**断电不丢**，避免每轮重启重新调阈值。
- 想恢复出厂：删掉 SD 卡上的 `threshold_config.json` 再重启。

---

## 4. 错误兜底

- **摄像头初始化失败**：屏幕显示红字 `CAMERA INIT FAILED / check FPC / power / lens`，程序保持 LVGL 存活不黑屏。先查排线、供电、镜头遮挡。
- **运行时异常**：`draw_frame` 内各步骤已 `try/except` 并打印栈，单帧异常不会整死循环；主循环 `BaseException` 会 `print_exception` 并通过串口发一帧 `valid=False` 通知主控。

---

## 5. 现场提帧率 / 抗干扰清单

1. 按 `ROI` 缩小搜索区（全屏→半屏→中央），`find_blobs` 像素跳采已开 2×2。
2. 关闭不需要的模式：只开 `Track` 时不要开 `Shape`。
3. 阈值要尽量"窄"——MASK 里把背景压成全黑，减少误检与合并开销。
4. `area_threshold=400` / `pixels_threshold=150` 已过滤小噪点；若目标很小可调小（但会增计算）。
5. HUD 看 FPS，若仍低，考虑 `TO_IDE=False` 关闭 IDE 推流。

---

## 6. 已知坑

- **坐标翻转/偏移**：K230 相机与屏分辨率一致(800×480)，坐标为显示空间；若 STM32 端图像旋转，需在 STM32 侧做映射，本程序只负责"看到的坐标"。
- **LAB 范围**：L 取值 0~100（代码内已按 0~100 归一），A/B 为 -128~127。MASK 编辑器滑块范围与之对应。
- **cv2 不可用**：屏上 `cv2:N`，形状识别走几何近似，圆/椭圆/三角精度有限；关键赛题建议以色块追踪为主。
- **LVGL 限制**：面板显隐用 `set_pos` 移出屏幕，无原生隐藏 API；形状多选用按钮底色表示选中态。
