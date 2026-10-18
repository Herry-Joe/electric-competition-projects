# ============================================================
# reference_official_rect_detect.py  (优化版 · 可运行)
# 针对本项目(步进电机云台 · 矩形循迹)裁剪的"成功案例"运行时参考。
# 来源: kendryte/canmv_k230 官方矩形检测例程, 已按本项目硬件对齐并优化。
#
# —— 适配当前硬件(与 k230/main.py 完全一致)——
#   相机 GC2093  1280x720@90  (Sensor id=2)
#   显示 ST7701   800x480      (视频直通 + OSD 叠层)
#   检测层灰度    320x192      (PROC_W x PROC_H)
#
# —— 相对旧版的优化点 ——
#   1. 只识别一个矩形: 二值化后 find_rects, 按"面积+中心偏好+规整度+时间一致性"
#      打分取最高分的那个(外框)。光取"最大"会被画面里别的暗色大块抢走 -> 加
#      几何/空间先验挑"对的那个", 抗误检测(旋钮见下方 CENTER_WEIGHT 等)。
#   2. 二值化识别: 灰度图 img.binary() 阈值分割, find_rects 跑在二值图上 ——
#      边更干净、误检少、比 Canny 边缘快得多(本项目实测 Canny 易卡顿)。
#   3. 高帧率: 检测只用 CHN1 灰度 320x192, 砍掉 CHN2 RGB565 激光通道(省带宽);
#      每帧 gc.collect(); 不碰 cv_lite 灰度 strided buffer(曾卡死); 不做激光检测。
#   4. 绘制直接画在 OSD 画布(800x480): 外框绿、内框红, 角点打叉, 支持倾斜四边形。
#
# 运行: K230 上直接  python reference_official_rect_detect.py
# 调参: 改下面 BINARY_LO/HI (暗框亮背景取低灰度段); 光照变化大就调这两个值,
#       或设 BINARIZE=False 退回对原灰度图直接 find_rects。
# ============================================================

import time, os, gc
from media.sensor import *
from media.display import *
from media.media import *
import image


# ---- 与 k230/main.py 保持一致的硬件常量 ----
SENSOR_ID = 2
SENSOR_INPUT_W = 1280
SENSOR_INPUT_H = 720
SENSOR_FPS = 90
HMIRROR = False
VFLIP = False
FRAME_W = 800
FRAME_H = 480
PROC_W = 320
PROC_H = 192
IDE_JPEG_QUALITY = 50

# ---- 检测调参旋钮 ----
# 二值化阈值(暗框/亮背景): 保留灰度 ∈ [BINARY_LO, BINARY_HI] 的像素。
# 目标通常是黑框白纸 -> 取低灰度段; 光照变化大就改这两个值。
BINARY_LO = 0
BINARY_HI = 110
# 是否二值化预处理(关掉则对原灰度图直接 find_rects)
BINARIZE = True
# find_rects 阈值: 二值图上线条清晰, 可设低一些(越大越严、矩形越大才认)
RECT_THRESHOLD = 2000
# 面积下限(像素): 小于它的碎块忽略
RECT_MIN_AREA = 400

# ---- 抗误检测(挑选"对的那个"矩形)旋钮 ----
# 只取"最大"会被画面里任何大暗块(桌子阴影/书本/杂物)抢走。加几何+空间先验:
#   目标矩形应在画面中央准星附近, 且宽高比接近目标(不是细长阴影条)。
#   再叠加时间一致性: 与上帧目标位置/尺寸接近的候选更优, 抗单帧闪烁误检。
RECT_TARGET_ASPECT = 1.0   # 目标宽高比(长边/短边); 非方形矩形改成实际值
CENTER_WEIGHT = 1.0        # 中心偏好权重(越大越要求矩形居中, 抗偏心误检)
REG_WEIGHT = 0.3           # 规整偏好权重(宽高比越接近目标越优)
MIN_ASPECT = 0.25          # 宽高比下限(太细长拒掉, 如阴影缝)
MAX_ASPECT = 4.0           # 宽高比上限(太扁长拒掉)
TRACK_WEIGHT = 0.3         # 时间一致性权重(0=关; 抗闪烁误检, 过大则死黏旧目标)
_prev_rect = None          # 上一帧锁定目标(时间一致性用)


def detect_rect(img):
    """输入 320x192 灰度图, 返回得分最高的一个 Rect(无则 None)。

    流程: 二值化 -> find_rects(抗旋转/仿射, 能检倾斜) -> 对每个候选打分
    (面积 + 中心偏好 + 规整度 + 时间一致性) -> 取最高分且通过宽高比门限的。
    返回 MaixPy image.Rect 对象: .rect()=(x,y,w,h), .corners()=真实四角。
    """
    global _prev_rect
    work = img.binary([(BINARY_LO, BINARY_HI)]) if BINARIZE else img
    rects = work.find_rects(threshold=RECT_THRESHOLD)
    best = None
    best_score = -1e9
    cx, cy = PROC_W // 2, PROC_H // 2
    norm = PROC_W + PROC_H
    for r in rects:
        x, y, w, h = r.rect()
        area = w * h
        if area < RECT_MIN_AREA:
            continue
        short, long = min(w, h), max(w, h)
        if short <= 0:
            continue
        aspect = long / short
        if aspect > MAX_ASPECT or aspect < MIN_ASPECT:
            continue  # 宽高比门限: 拒掉太细长/太扁长的误检
        rcx, rcy = x + w // 2, y + h // 2
        center_pen = (abs(rcx - cx) + abs(rcy - cy)) / norm   # 归一化 0~1
        reg_pen = abs(aspect - RECT_TARGET_ASPECT) / RECT_TARGET_ASPECT
        track_pen = 0.0
        if TRACK_WEIGHT > 0 and _prev_rect is not None:
            px, py, pw, ph = _prev_rect.rect()
            pcx, pcy = px + pw // 2, py + ph // 2
            pos_shift = (abs(rcx - pcx) + abs(rcy - pcy)) / norm
            size_shift = abs(area - pw * ph) / (PROC_W * PROC_H)
            track_pen = pos_shift + size_shift
        score = (area / (PROC_W * PROC_H)) \
                - CENTER_WEIGHT * center_pen \
                - REG_WEIGHT * reg_pen \
                - TRACK_WEIGHT * track_pen
        if score > best_score:
            best, best_score = r, score
    if best is not None:
        _prev_rect = best
    return best


def scale_to_frame(pt):
    """proc(320x192) 坐标 -> frame(800x480) 显示坐标(同倍率, 无拉伸错位)。"""
    return (int(round(pt[0] * FRAME_W / PROC_W)),
            int(round(pt[1] * FRAME_H / PROC_H)))


def draw_rect_poly(canvas, rect, color):
    """把矩形(支持倾斜)按真实四角画成闭合多边形 + 角点打叉。"""
    if rect is None:
        return
    pts = [scale_to_frame(c) for c in rect.corners()]
    n = len(pts)
    for i in range(n):
        a = pts[i]
        b = pts[(i + 1) % n]
        canvas.draw_line(a[0], a[1], b[0], b[1], color=color, thickness=3)
    for p in pts:
        canvas.draw_circle(p[0], p[1], 5, color=color, thickness=2)


def run():
    sensor = Sensor(id=SENSOR_ID, width=SENSOR_INPUT_W, height=SENSOR_INPUT_H, fps=SENSOR_FPS)
    sensor.reset()
    try:
        sensor.set_hmirror(HMIRROR)
        sensor.set_vflip(VFLIP)
    except BaseException:
        pass

    # CHN0: 视频直通 LCD (800x480)
    sensor.set_framesize(width=FRAME_W, height=FRAME_H, chn=CAM_CHN_ID_0)
    sensor.set_pixformat(Sensor.YUV420SP, chn=CAM_CHN_ID_0)
    bind_info = sensor.bind_info()
    Display.bind_layer(**bind_info, layer=Display.LAYER_VIDEO1)

    # CHN1: 灰度 320x192 用于矩形检测(唯一检测通道, 砍掉 RGB 通道=省带宽=提帧率)
    sensor.set_framesize(width=PROC_W, height=PROC_H, chn=CAM_CHN_ID_1)
    sensor.set_pixformat(Sensor.GRAYSCALE, chn=CAM_CHN_ID_1)

    Display.init(Display.ST7701, width=FRAME_W, height=FRAME_H,
                 to_ide=True, quality=IDE_JPEG_QUALITY, osd_num=2)
    osd_img = image.Image(FRAME_W, FRAME_H, image.ARGB8888)
    MediaManager.init()
    sensor.run()
    os.exitpoint(os.EXITPOINT_ENABLE)

    clock = time.clock()
    while True:
        os.exitpoint()                 # 每帧调用, 允许 IDE 中断 / 任务调度
        clock.tick()

        gray = sensor.snapshot(chn=CAM_CHN_ID_1)   # 只取检测通道, 不取 RGB
        rect = detect_rect(gray)

        osd_img.clear()
        # 准星
        osd_img.draw_cross(FRAME_W // 2, FRAME_H // 2, color=(255, 255, 0), size=14, thickness=2)
        # 目标矩形(绿)
        draw_rect_poly(osd_img, rect, (0, 200, 0))

        osd_img.draw_string_advanced(
            10, 3, 28, "FPS:{:.1f}".format(clock.fps()), color=(255, 255, 255))

        Display.show_image(osd_img, layer=Display.LAYER_OSD1)
        gc.collect()                   # ★ 每帧回收, 防 K230 小堆碎卡死
    # IDE 中断会跳出循环到这里
    sensor.stop()
    Display.deinit()
    os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)


if __name__ == "__main__":
    run()
