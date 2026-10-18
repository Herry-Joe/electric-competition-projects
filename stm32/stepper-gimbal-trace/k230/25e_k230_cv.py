# =============================================================================
# 庐山派 K230 矩形循迹视觉 (Linux + OpenCV 运行时)
# -----------------------------------------------------------------------------
# 在官方 25e_k230_cv.py 基础上改造:
#   * 检测: 二值化 + findContours + 近似4边形, 取"最对"的一个矩形
#           (中心偏好 + 规整度 + 宽高比门限 + 时间一致性, 抗误检测/抗抖动)
#   * 循迹: 把矩形周长离散成点列, 设点以恒定弧长速度沿边行走
#           (转角减速 + 转角驻留, 让云台有时间转90度)
#   * 输出: 通过 UART 发给 STM32 云台, 协议与 k230/main.py(CanMV板)逐字节一致:
#           0x3C 0x3B [XH][XL][YH][YL][FLAG] CRC8 0x01 0x01 (10字节)
#           X/Y = 轨迹目标点相对画面中心的误差像素(与 CanMV 中心参考模式同语义)
#           FLAG bit0=valid bit1=hit(整圈完成) bit2=hi_conf
#   * 安全: 每帧 gc.collect() + os.exitpoint(); 每帧检测(庐山派算力足);
#           UART 打开失败不致命, 仍可跑视觉+显示(纯验证模式)
#
# 注意: 误差在 640x480 空间(原 CanMV 用 800 宽), 比原板略小,
#       STM32 侧 Kp 可能需略调大 -> 用串口调参命令(如 `g p kp`/`ff`)微调。
# =============================================================================

import os
import gc
import time
import math
import cv2
import numpy as np
import image
import serial  # pyserial: pip install pyserial (庐山派一般已带)

from media.sensor import *
from media.display import *
from media.media import *

# ---------------------------------------------------------------------------
# 分辨率
# ---------------------------------------------------------------------------
FRAME_WIDTH = 640
FRAME_HEIGHT = 480

# LCD 分辨率 (庐山派常见 ST7701 800x480)
LCD_WIDTH = 800
LCD_HEIGHT = 480

# 常见 LCD：ST7701；若不符改成 Display.NT35516 / Display.GC9503
DISPLAY_TYPE = Display.ST7701

# ---------------------------------------------------------------------------
# 检测: 二值化 + 抗误检测评分
# ---------------------------------------------------------------------------
BINARIZE = True            # 用二值化替代纯 Canny(边更干净/更快/抗弱光误检)
BINARY_LO = 0
BINARY_HI = 110            # 比此更暗的像素视为目标(黑框/暗矩形) -> 二值化为白
MORPH_OPEN = True          # 形态学开运算去噪点(可选)

MIN_AREA = 800             # 面积下限(像素), 小于的碎块忽略
MAX_AREA = 60000           # 面积上限(像素), 大于的(整屏)忽略
MIN_RECT_W = 30            # 最小宽(像素)
MIN_RECT_H = 30            # 最小高(像素)
MIN_ASPECT = 0.4           # 宽高比下限(太细长=误检, 拒)
MAX_ASPECT = 2.5           # 宽高比上限(太扁长=误检, 拒)
RECT_TARGET_ASPECT = 1.0   # 目标宽高比(非方形矩形改成实际值)

CENTER_WEIGHT = 1.0        # 抗误检主力: 偏离画面中心越多扣分越重
REG_WEIGHT = 0.3           # 规整度: 宽高比越接近目标越优
TRACK_WEIGHT = 0.3         # 时间一致性: 与上帧目标接近更优(抗单帧闪烁误切换)
APPROX_RATIO = 0.02        # findContours 多边形近似精度

# ---------------------------------------------------------------------------
# 轨迹: 周长行走
# ---------------------------------------------------------------------------
TRACE_SPEED = 70.0         # 行走速度(像素/秒, 在 640x480 空间)
TRACE_SAMPLE_SPACING = 4.0 # 周长离散采样间距(像素)
CORNER_SLOWDOWN_ARC = 30.0 # 距转角多近开始减速(弧长像素)
CORNER_SLOW_FACTOR = 0.4   # 转角处最低速度比例(0.4=减速到40%)
CORNER_DWELL_MS = 250      # 到达转角顶点驻留(ms), 让云台定点
CORNER_DWELL_ENTER = 12.0  # 进入转角前多少弧长内触发驻留
TRACE_DIR = 1              # 1=顺时针, -1=逆时针
TRACE_LOOP = "repeat"      # repeat | once | pingpong
TRACE_SMOOTH = True        # 对目标点做 EMA 平滑(抗帧抖动)
TRACE_ALPHA_NEAR = 0.6
TRACE_ALPHA_MIDDLE = 0.35
TRACE_ALPHA_FAR = 0.2
TRACE_NEAR_DISTANCE = 8.0
TRACE_FAR_DISTANCE = 40.0
MISS_LIMIT = 12            # 连续漏检多少帧后判定目标丢失(发 valid=0 安全停)
DETECT_INTERVAL = 1        # 每几帧重检一次(庐山派算力足, 默认每帧)

# ---------------------------------------------------------------------------
# UART (STM32 协议)
# ---------------------------------------------------------------------------
UART_BAUDRATE = 115200
# 候选设备节点(按你板子实际接 STM32 的串口改/增): 庐山派常见 /dev/ttyS0 或 /dev/ttyS5
UART_CANDIDATES = ["/dev/ttyS0", "/dev/ttyS5", "/dev/ttyAMA0", "/dev/ttyUSB0"]
UART_SEND_INTERVAL = 1     # 每几帧发一次(默认每帧)

# STM32 期望帧(与 k230/main.py 发送一致):
#   0x3C 0x3B [XH][XL][YH][YL][FLAG] [CRC8] 0x01 0x01   (10 字节)
#   FLAG bit0=valid, bit1=hit(整圈完成), bit2=hi_conf
# ---------------------------------------------------------------

def crc8(data):
    """CRC-8, poly 0x07, init 0x00, MSB-first. 与 STM32 K230_CRC8 算法一致。"""
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ 0x07) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
    return crc & 0xFF


def send_binary(uart, x, y, valid, hit, hi_conf):
    """发送 10 字节坐标帧给 STM32。返回写入字节数(0=未发送)。"""
    if uart is None:
        return 0
    buf = bytearray(10)
    buf[0] = 0x3C
    buf[1] = 0x3B
    x16 = int(max(-32768, min(32767, x)))
    y16 = int(max(-32768, min(32767, y)))
    buf[2] = (x16 >> 8) & 0xFF
    buf[3] = x16 & 0xFF
    buf[4] = (y16 >> 8) & 0xFF
    buf[5] = y16 & 0xFF
    f = 0
    if valid:
        f |= 1
    if hit:
        f |= 2
    if hi_conf:
        f |= 4
    buf[6] = f
    buf[7] = crc8(bytes([buf[2], buf[3], buf[4], buf[5], buf[6]]))
    buf[8] = 1
    buf[9] = 1
    try:
        return uart.write(buf)
    except Exception:
        return 0


def open_uart():
    """依次尝试候选设备节点, 第一个能打开的即用。全失败返回 None(纯视觉模式)。"""
    for dev in UART_CANDIDATES:
        try:
            s = serial.Serial(dev, UART_BAUDRATE, timeout=0.01)
            print("UART open:", dev)
            return s
        except Exception as e:
            print("UART skip", dev, "->", e)
    print("WARNING: 未打开任何 UART 设备, 进入纯视觉模式(不发给 STM32)")
    return None


# ---------------------------------------------------------------------------
# 检测
# ---------------------------------------------------------------------------
def approx_to_corners(approx):
    """cv2.approxPolyDP 的 4 点按角度绕质心排序, 得到稳定环形顺序(便于周长行走)。"""
    pts = [(int(p[0][0]), int(p[0][1])) for p in approx]
    if len(pts) < 3:
        return pts
    cx = sum(p[0] for p in pts) / len(pts)
    cy = sum(p[1] for p in pts) / len(pts)
    pts.sort(key=lambda p: math.atan2(p[1] - cy, p[0] - cx), reverse=True)
    return pts


def detect_rects(gray):
    """输入灰度图, 返回候选矩形列表: [(corners4, area, aspect), ...]。"""
    if BINARIZE:
        # 暗目标 -> 白(255): THRESH_BINARY_INV
        _, mask = cv2.threshold(gray, BINARY_HI, 255, cv2.THRESH_BINARY_INV)
    else:
        mask = gray
    if MORPH_OPEN:
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cands = []
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < MIN_AREA or area > MAX_AREA:
            continue
        epsilon = APPROX_RATIO * cv2.arcLength(cnt, True)
        approx = cv2.approxPolyDP(cnt, epsilon, True)
        if len(approx) != 4:
            continue
        x, y, w, h = cv2.boundingRect(approx)
        if w < MIN_RECT_W or h < MIN_RECT_H:
            continue
        short = min(w, h)
        long = max(w, h)
        if short <= 0:
            continue
        aspect = long / short
        if aspect < MIN_ASPECT or aspect > MAX_ASPECT:
            continue
        fill = (area / (w * h)) if (w * h) > 0 else 0
        if fill < 0.5:
            continue
        corners = approx_to_corners(approx)
        cands.append((corners, area, aspect))
    return cands


def choose_rect(cands, prev_corners):
    """抗误检测: 对每个候选打分(面积 + 中心偏好 + 规整度 + 时间一致性), 取最高。"""
    cx0 = FRAME_WIDTH // 2
    cy0 = FRAME_HEIGHT // 2
    best = None
    best_score = -1e18
    for corners, area, aspect in cands:
        ccx = sum(p[0] for p in corners) / 4
        ccy = sum(p[1] for p in corners) / 4
        center_pen = (abs(ccx - cx0) + abs(ccy - cy0)) * CENTER_WEIGHT
        reg_pen = abs(aspect - RECT_TARGET_ASPECT) * area * REG_WEIGHT
        score = area - center_pen - reg_pen
        if prev_corners is not None:
            pccx = sum(p[0] for p in prev_corners) / 4
            pccy = sum(p[1] for p in prev_corners) / 4
            dtc = math.sqrt((ccx - pccx) ** 2 + (ccy - pccy) ** 2)
            score -= (dtc / 50.0) * area * TRACK_WEIGHT
        if score > best_score:
            best_score = score
            best = corners
    return best


def rect_drifted(old, new, frac=0.25):
    """矩形整体平移是否超过 frac*尺寸 -> 需要重建轨迹。"""
    if old is None or new is None:
        return True
    ocx = sum(p[0] for p in old) / 4
    ocy = sum(p[1] for p in old) / 4
    ncx = sum(p[0] for p in new) / 4
    ncy = sum(p[1] for p in new) / 4
    w = abs(old[1][0] - old[0][0]) or 1
    h = abs(old[2][1] - old[1][1]) or 1
    d = math.sqrt((ncx - ocx) ** 2 + (ncy - ocy) ** 2)
    return d > frac * max(w, h)


# ---------------------------------------------------------------------------
# 轨迹: 矩形周长离散点列 + 沿边行走
# ---------------------------------------------------------------------------
class RectTrajectory:
    """闭合矩形周长, 离散成密集点列, 设点以恒定弧长速度沿边行走。"""

    def __init__(self, corners, spacing):
        self.corners = corners
        self.spacing = spacing
        self.samples = []
        self.total_len = 0.0
        self._build()

    def _build(self):
        self.samples = []
        self.corner_arcs = []
        self.corner_points = []
        n = len(self.corners)
        idx = 0
        for i in range(n):
            x0, y0 = self.corners[i]
            x1, y1 = self.corners[(i + 1) % n]
            self.corner_arcs.append(idx * self.spacing)
            self.corner_points.append((x0, y0))
            seg_len = math.sqrt((x1 - x0) ** 2 + (y1 - y0) ** 2)
            steps = max(1, int(seg_len / self.spacing))
            for s in range(steps):
                t = s / steps
                self.samples.append((x0 + (x1 - x0) * t, y0 + (y1 - y0) * t))
                idx += 1
        self.total_len = len(self.samples) * self.spacing

    def count(self):
        return len(self.samples)

    def point_at_index(self, idx):
        if not self.samples:
            return None
        return self.samples[idx % len(self.samples)]

    def point_at_arc(self, arc):
        if not self.samples:
            return None
        idx = int(arc / self.spacing) % len(self.samples)
        return self.samples[idx]

    def nearest_index(self, point):
        if not self.samples or point is None:
            return 0
        best = 0
        best_d = 1e18
        px, py = point
        for i, (sx, sy) in enumerate(self.samples):
            d = (sx - px) ** 2 + (sy - py) ** 2
            if d < best_d:
                best_d = d
                best = i
        return best


def arc_dist_to_nearest_corner(arc, corner_arcs, total):
    """环形(考虑回绕)距最近转角的弧长。"""
    best = total
    for ca in corner_arcs:
        d = abs(arc - ca)
        if d > total * 0.5:
            d = total - d
        if d < best:
            best = d
    return best


def next_corner_ahead(arc, direction, corner_arcs, total, enter):
    """返回 direction 方向上距 arc 不超过 enter 弧长的最近转角弧长, 否则 None。"""
    best_arc = None
    best_d = None
    for ca in corner_arcs:
        if direction > 0:
            d = (ca - arc) % total
        else:
            d = (arc - ca) % total
        if 0.0 <= d <= enter:
            if best_d is None or d < best_d:
                best_d = d
                best_arc = ca
    return best_arc


# ---------------------------------------------------------------------------
# 主循环
# ---------------------------------------------------------------------------
def main():
    sensor = None
    clock = time.clock()
    uart = open_uart()

    # 轨迹状态
    traj = None
    traj_corners = None
    traj_locked = False
    traj_arc = 0.0
    traj_dir = TRACE_DIR
    traj_done = False
    traj_dwell_until = 0.0       # ms 时间戳
    traj_dwell_arc = 0.0
    last_traj_t = time.time()
    missed = 0

    # 平滑状态
    smooth_x = 0.0
    smooth_y = 0.0
    smooth_init = False

    frame_id = 0
    last_fps_print = 0.0

    try:
        os.exitpoint(os.EXITPOINT_ENABLE)

        sensor = Sensor(width=1280, height=960, fps=90)
        sensor.reset()
        sensor.set_framesize(width=FRAME_WIDTH, height=FRAME_HEIGHT)
        sensor.set_pixformat(Sensor.RGB888)

        Display.init(DISPLAY_TYPE, width=LCD_WIDTH, height=LCD_HEIGHT, to_ide=True)
        MediaManager.init()

        sensor.run()
        time.sleep(0.5)

        while True:
            os.exitpoint()
            clock.tick()

            img = sensor.snapshot()
            # 庐山派 to_numpy_ref() 对 RGB888 返回 RGB 顺序(HWC).
            # cv2 内部按 BGR 处理, 故先转 BGR; 显示前再转回 RGB.
            frame_rgb = img.to_numpy_ref()
            frame = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            frame_id += 1

            # --- 检测 ---
            run_det = (frame_id == 1) or (frame_id % DETECT_INTERVAL == 0)
            if run_det:
                cands = detect_rects(gray)
                chosen = choose_rect(cands, traj_corners if traj_locked else None)
                if chosen is not None:
                    missed = 0
                    if (not traj_locked) or rect_drifted(traj_corners, chosen):
                        traj = RectTrajectory(chosen, TRACE_SAMPLE_SPACING)
                        traj_corners = chosen
                        # 从离画面中心最近的边点起步
                        start_idx = traj.nearest_index((FRAME_WIDTH // 2, FRAME_HEIGHT // 2))
                        traj_arc = start_idx * TRACE_SAMPLE_SPACING
                        traj_dir = TRACE_DIR
                        traj_done = False
                        traj_dwell_until = 0.0
                        traj_locked = True
                else:
                    missed += 1
                    if missed > MISS_LIMIT:
                        traj_locked = False

            # --- 沿周长推进设点 ---
            desired_s = None
            progress = 0.0
            if traj_locked and traj is not None:
                now = time.time()
                dt = now - last_traj_t
                last_traj_t = now
                if dt < 0:
                    dt = 0.0
                if dt > 0.2:
                    dt = 0.2           # 丢帧保护: dt 不爆炸

                total = traj.total_len
                if total > 0 and not traj_done:
                    if traj_dwell_until > 0 and (now * 1000.0) < traj_dwell_until:
                        # 转角驻留: 冻结在顶点, 让云台定点
                        traj_arc = traj_dwell_arc
                    else:
                        if traj_dwell_until > 0:
                            # 驻留刚结束: 越过转角一点, 避免重复触发
                            traj_arc = traj_dwell_arc + traj_dir * TRACE_SAMPLE_SPACING
                            traj_dwell_until = 0.0

                        # 转角减速
                        d_corner = arc_dist_to_nearest_corner(traj_arc, traj.corner_arcs, total)
                        if d_corner >= CORNER_SLOWDOWN_ARC:
                            speed_scale = 1.0
                        else:
                            t = d_corner / CORNER_SLOWDOWN_ARC
                            speed_scale = CORNER_SLOW_FACTOR + (1.0 - CORNER_SLOW_FACTOR) * t
                        traj_arc += traj_dir * TRACE_SPEED * speed_scale * dt

                        # 环路处理
                        if TRACE_LOOP == "repeat":
                            traj_arc %= total
                        elif TRACE_LOOP == "pingpong":
                            if traj_arc >= total:
                                traj_arc = total
                                traj_dir = -1
                            elif traj_arc <= 0:
                                traj_arc = 0
                                traj_dir = 1
                        else:  # once
                            if traj_arc >= total:
                                traj_arc = total
                                traj_done = True
                            elif traj_arc <= 0:
                                traj_arc = 0
                                traj_done = True

                        # 到达转角顶点 -> 触发驻留
                        if CORNER_DWELL_MS > 0 and not traj_done:
                            ca = next_corner_ahead(traj_arc, traj_dir,
                                                   traj.corner_arcs, total,
                                                   CORNER_DWELL_ENTER)
                            if ca is not None:
                                traj_dwell_arc = ca
                                traj_dwell_until = now * 1000.0 + CORNER_DWELL_MS
                                traj_arc = ca

                desired = traj.point_at_arc(traj_arc)
                if desired is not None:
                    if TRACE_SMOOTH:
                        d_corner = arc_dist_to_nearest_corner(traj_arc, traj.corner_arcs, total)
                        if d_corner < CORNER_SLOWDOWN_ARC:
                            # 近转角用原始点, 避免 EMA 把 90 度转角抹圆
                            smooth_x, smooth_y = desired
                            smooth_init = True
                            desired_s = desired
                        elif not smooth_init:
                            smooth_x, smooth_y = desired
                            smooth_init = True
                            desired_s = desired
                        else:
                            d = max(abs(desired[0] - smooth_x), abs(desired[1] - smooth_y))
                            if d <= TRACE_NEAR_DISTANCE:
                                a = TRACE_ALPHA_NEAR
                            elif d <= TRACE_FAR_DISTANCE:
                                a = TRACE_ALPHA_MIDDLE
                            else:
                                a = TRACE_ALPHA_FAR
                            smooth_x += (desired[0] - smooth_x) * a
                            smooth_y += (desired[1] - smooth_y) * a
                            desired_s = (int(round(smooth_x)), int(round(smooth_y)))
                    else:
                        desired_s = desired
                    progress = (traj_arc % total) / total if total > 0 else 0.0

            # --- 计算误差并发送 (误差 = 目标点 - 画面中心) ---
            if desired_s is not None:
                ex = int(round(desired_s[0] - FRAME_WIDTH // 2))
                ey = int(round(desired_s[1] - FRAME_HEIGHT // 2))
                valid = True
                hi_conf = True
            else:
                ex = 0
                ey = 0
                valid = False
                hi_conf = False

            if frame_id % UART_SEND_INTERVAL == 0:
                send_binary(uart, ex, ey, valid, traj_done, hi_conf)

            # --- OSD 绘制 (在 BGR frame 上, cv2 颜色元祖为 BGR, 绘制正确) ---
            if traj_corners is not None:
                cv2.polylines(frame,
                              [np.array([[int(p[0]), int(p[1])] for p in traj_corners],
                                        dtype=np.int32)],
                              True, (0, 255, 0), 2)
            if traj is not None and traj.samples:
                cv2.polylines(frame,
                              [np.array([[int(p[0]), int(p[1])] for p in traj.samples],
                                        dtype=np.int32)],
                              True, (255, 255, 0), 1)
            if desired_s is not None:
                cv2.circle(frame, desired_s, 6, (0, 0, 255), -1)
                cv2.line(frame, (FRAME_WIDTH // 2, FRAME_HEIGHT // 2),
                         desired_s, (0, 255, 255), 2)

            fps = clock.fps()
            cv2.putText(frame, "FPS: %.1f" % fps, (30, 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)
            cv2.putText(frame, "Trace: prog=%.2f %s" % (progress, "DONE" if traj_done else ""),
                        (30, 110), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            cv2.putText(frame, "UART: %s  Err:(%d,%d) valid=%d" % (
                            "ON" if uart else "OFF", ex, ey, 1 if valid else 0),
                        (30, 145), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)

            # 铺满 800x480 LCD (BGR -> RGB 还原, 因显示接口期望 RGB888)
            show_bgr = cv2.resize(frame, (LCD_WIDTH, LCD_HEIGHT))
            show_np = cv2.cvtColor(show_bgr, cv2.COLOR_BGR2RGB)
            show_img = image.Image(LCD_WIDTH, LCD_HEIGHT, image.RGB888,
                                   alloc=image.ALLOC_REF, data=show_np)
            Display.show_image(show_img)

            gc.collect()

    except KeyboardInterrupt:
        print("user stop")
    except BaseException as e:
        print("Exception:", e)
    finally:
        if isinstance(sensor, Sensor):
            sensor.stop()
        Display.deinit()
        os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
        time.sleep_ms(100)
        MediaManager.deinit()


if __name__ == "__main__":
    main()
