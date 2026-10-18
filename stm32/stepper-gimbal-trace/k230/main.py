# K230 / CanMV vision code for TI Cup 2025 E style RECTANGLE TRACING.
#
# Refactored from the auto-aiming version: NO YOLO. The black rectangle border
# is no longer a "target to hit"; it is a ROUTE. The laser pen traces the closed
# rectangle perimeter, starting from the point on the border nearest to the
# reference (laser dot or frame center), and draws a closed figure.
#
# How it drives the gimbal:
#   The on-board STM32 gimbal firmware (firmware.hex) runs an error-nulling
#   loop: it receives err = target - laser and drives the gimbal so the red
#   laser dot lands on `target`. We set `target` = a moving point that walks
#   along the rectangle perimeter. As the point advances, the laser follows it
#   and traces the border. No motor-side change is required.
#
# Detection:
#   - Rectangle border: CanMV OpenCV (cv2) on the 320x192 grayscale stream:
#     binary threshold -> Gaussian blur -> Canny -> findContours -> approxPolyDP
#     (4-point) -> score pick (anti false-detection). This is the "二值化+Canny
#     +取最大最规整四边形" path you asked for, and now runs on the official cv2
#     binding (no more cv_lite strided-buffer C-level hard fault). img.find_rects
#     is kept only as an emergency fallback if cv2 import/run fails on-board.
#   - Laser dot: LAB red-blob detection on the 320x192 RGB stream (needed as
#     the control reference so the gimbal can close the loop on the real pen).
#
# Protocol:
#   OUTPUT_PROTOCOL = "binary"  -> 0x3C 0x3B [XH][XL][YH][YL][FLAG] CRC8 0x01 0x01
#                                (matches the STM32 gimbal firmware)
#   OUTPUT_PROTOCOL = "ascii"   -> "$cx,cy#" (TI-Cup style: a motor that aims
#                                the laser AT the given point instead of
#                                nulling an error). Set TRACE_REFERENCE="center"
#                                when using ascii mode.
#
# Keep this file as ASCII for CanMV compatibility.

import time
import os
import gc
import sys
import math
import image

from machine import UART
from machine import FPIOA
from media.sensor import *
from media.display import *
from media.media import *

try:
    import cv2
    HAS_CV2 = True
except BaseException:
    cv2 = None
    HAS_CV2 = False


# =========================
# User settings
# =========================

FRAME_W = 800
FRAME_H = 480
PROC_W = 320
PROC_H = 192
# OSD 校准偏移(显示像素): 若 OSD 框与目标错开, 上板量出偏移填入(可由 .conf 调)。
# 成因通常是传感器多通道(视频 800x480 vs 灰度 320x192)缩放/裁剪中心未对齐, 非拉伸
# (proc 与显示宽高比同为 1.667, scale_point_to_frame 用同倍率 2.5, 不会拉伸错位)。
OSD_X_OFFSET = 0
OSD_Y_OFFSET = 0

# Camera parameters from the known-working program for this LCKFB K230 board.
SENSOR_ID = 2
SENSOR_INPUT_W = 1280
SENSOR_INPUT_H = 720
SENSOR_FPS = 90
SENSOR_ANALOG_GAIN = 20
IDE_JPEG_QUALITY = 50

# UART wiring for the STM32 gimbal: K230 GPIO5/6 <-> STM32 PB10/PB11.
UART_TX_PIN = 5
UART_RX_PIN = 6
UART_BAUDRATE = 115200

# Flip these if the image is upside down or mirrored.
HMIRROR = False
VFLIP = False

# ----- Rectangle border detection (grayscale 320x192, CanMV OpenCV) -----
# 主路径 = 官方 cv2 绑定(新固件 daily build 已支持): 二值化(THRESH_BINARY_INV)
#   → 高斯模糊 → Canny(30/90) → findContours → approxPolyDP 取 4 点 → 抗误检评分。
# 这正是你要的"二值化+Canny+取最大最规整四边形", 且跑在官方 cv2 上,
# 不再有旧 cv_lite 灰度 strided-buffer 的 C 层 hard fault(静默卡死)。
# 兜底: 若 cv2 导入/运行失败, 自动回退到 img.find_rects(MaixPy C 实现, 官方同款,
#   抗旋转/仿射、能吃倾斜矩形、直接吃 image 对象无缓冲越界风险)。
USE_CV2 = True
# 检测流水线(朴素边缘法, 与用户确认一致):
#   灰度(320x192) → [二值化去噪] → 高斯模糊 → Canny 边缘 → 轮廓 → 近似4边形
#   → 取四角 → 抗误检评分选最优 → 经串口输出 XY 给 STM32。
# Canny 阈值(已上板实测定档): 低=30 / 高=90。暗光下边缘淡, 比旧默认 50/150 更松、
#   多抓边, 实测矩形拾取稳定。仍可用 /data/ti_vision_tune.conf 现场微调
#   (canny1/canny2); 噪点/误检多 → 上调收紧; 边缘更淡 → 再下调。
CANNY_THRESH1 = 30
CANNY_THRESH2 = 90
APPROX_EPSILON = 0.04     # approxPolyDP 精度 = APPROX_EPSILON * 周长; 调大更容忍畸变边
GAUSSIAN_BLUR_SIZE = 3    # 高斯模糊核大小(奇数); 用户草稿写 5×5, 本实现用 3×3 已实测稳定
# 兜底 find_rects 灰度阈值(仅 cv2 不可用时走)
RECT_THRESHOLD = 6000
RECT_MIN_AREA = 500
RECT_MAX_AREA = PROC_W * PROC_H * 7 // 10
RECT_MAX_ASPECT = 4.0
# “最标准”偏好: 面积相近的候选里, 宽高比越接近 RECT_TARGET_ASPECT 得分越高。
# 调大 RECT_REGULARITY_WEIGHT 更偏好方正矩形; 若目标矩形非方形, 把
# RECT_TARGET_ASPECT 改成实际宽高比; 设 RECT_REGULARITY_WEIGHT=0 则退化为
# 纯“最大+居中”选择(与旧逻辑一致)。权重很小, 仅影响势均力敌时的取舍。
RECT_TARGET_ASPECT = 1.0
RECT_REGULARITY_WEIGHT = 0.02
# 二值化预处理: 开启后 find_rects 跑在二值图上(边更干净/更快/抗弱光误检)。
# 与 reference_official_rect_detect.py 同款思路。"矩形循迹"目标多为黑框白纸,
# 取低灰度段; 光照变化大就调 BINARY_LO/HI, 或设 RECT_BINARIZE=False 退回原灰度。
RECT_BINARIZE = True
BINARY_LO = 0
BINARY_HI = 110
# 时间一致性权重(抗误切换/抗丢帧): 在上一帧目标附近、尺寸相近的候选得分更高。
# 0=关闭(纯空间选最大最规整); 过大→死黏旧目标, 真目标移动时反而跟丢。可由 .conf 调。
RECT_TRACK_WEIGHT = 0.2
# 矩形框边缘留白(像素): 矩形触碰画面边缘时视为不完整 → 拒识, 防止跟踪半截框。
# 仅当矩形四周距离画面边界均 ≥ 此值时才算有效。设 2 可容忍 1px 边界伪影。
RECT_EDGE_MARGIN = 0     # 临时=0, 排障: 防边缘拒识阻断轨迹锁定

# ----- 边框吸附速度 / 高速稳定性(本优化新增) -----
# 锁定后, 每帧成功检测都把"锁定角点"用 EMA 朝真实角点贴合, 并重建轨迹保持设点在
# 边框上的位置(不跳变)。这样边框框(OSD)与循迹路径能近实时跟住真实边框 -> 吸附快。
#   RECT_SNAP_ALPHA: 贴合速率(0~1)。越大越跟手(吸附越快), 过小则框滞后; 过大则抖动。
#     实测 0.4 在"跟手"与"抗单帧噪声"间平衡好。
#   大跳变快贴合: 若本次检测中心相对锁定框位移 > TRACE_REBUILD_DRIFT*框边长(原"重建阈值"
#     复用), 直接整框贴合(alpha=1.0)而非缓慢 EMA -> 边框突然移动/出现时瞬间吸住。
# 高速稳定:
#   RECT_MOVE_PX_PER_FRAME: 边框中心帧间位移超过此值(proc px/帧)即判"在动" ->
#     强制每帧检测(不被 DETECT_INTERVAL 降频), 高速扫动时不漏掉检测帧。
#   RECT_HOLD_FRAMES: 检测丢失(运动模糊/瞬时遮挡)时, 保持上一框并外推最多这么多帧,
#     识别"不掉"; 轨迹继续沿保持路径走。超过则正常判定丢失(走激光回退逻辑)。
RECT_SNAP_ALPHA = 0.4
RECT_MOVE_PX_PER_FRAME = 1.5
RECT_HOLD_FRAMES = 6
RECT_HOLD_EXTRAPOLATE = True

# ----- Trajectory (rectangle tracing) -----
TRACE_ENABLE = True

# Which physical pin the cloud of points is walked on.
#   "laser"  -> start from the border point nearest the detected laser dot,
#               and close the loop using err = desired - laser (needs laser).
#   "center" -> start from the border point nearest the frame center, and use
#               err = desired - center (co-axial laser assumption, no laser
#               detection needed; set LASER_ENABLE=False then).
TRACE_REFERENCE = "laser"

# Perimeter walking speed, in processing pixels per second (320x192 space).
# Tune this together with the STM32 PID gains: too fast -> the gimbal lags
# behind the moving set-point; too slow -> sluggish trace.
TRACE_SPEED = 40.0

# What to do after one full loop:
#   "once"     -> park at the last point and stop advancing
#   "repeat"   -> keep looping forever (default for a visible demo)
#   "pingpong" -> go back and forth along the border
TRACE_LOOP = "repeat"

# Walking direction: +1 or -1 (depends on corner winding / gimbal invert).
TRACE_DIR = 1

# Spacing between dense perimeter samples, in processing pixels.
TRACE_SAMPLE_SPACING = 3.0

# Lock the trajectory after the first stable detection so detection jitter
# does not make the set-point jump. Rebuild only if the rectangle drifts.
TRACE_LOCK = True
TRACE_REBUILD_DRIFT = 0.12   # rebuild if center moves > 12% of box size (0=never)
TRACE_HOLD_FRAMES = 60       # keep following the locked path if rect is lost

# Light smoothing of the moving set-point (keeps the trace fluid).
TRACE_SMOOTH = True
TRACE_ALPHA_NEAR = 0.5
TRACE_ALPHA_MIDDLE = 0.7
TRACE_ALPHA_FAR = 0.9
TRACE_NEAR_DISTANCE = 2.0
TRACE_FAR_DISTANCE = 10.0

# ----- Corner handling (v2.2): keep the trace sharp at 90-degree turns -----
# The set-point walks the perimeter at constant speed; at a corner its
# direction flips 90 deg instantly and a velocity-controlled gimbal lags,
# cutting the corner. Three mitigations:
#   1) slow down near corners so the gimbal has time to turn;
#   2) briefly DWELL (hold) on the exact corner vertex so the laser settles
#      onto it before continuing -> crisp corner;
#   3) disable the set-point EMA near corners so the turn is not rounded off.
CORNER_SLOWDOWN_ARC = 25.0        # arc window (proc px) around each corner where speed ramps down
CORNER_SLOW_FACTOR = 0.30         # min fraction of TRACE_SPEED at the corner (1.0 far away)
CORNER_DWELL_MS = 200             # hold exactly on the corner vertex this long (0 = off)
CORNER_DWELL_ENTER = 6.0          # engage dwell when within this arc distance of a corner
CORNER_SMOOTH_DISABLE_ARC = 10.0  # within this arc of a corner, use raw set-point (no EMA)

# ----- Laser dot detection (RGB 320x192, LAB red) -----
# Needed when TRACE_REFERENCE = "laser". Ported from the YOLO vision code:
# multi-threshold + local ROI + nearest-neighbour tracking for stability.
LASER_ENABLE = True
# 红激光多级 LAB 阈值。
# LASER_LAB_NARROW = 窄阈值, 仅匹配高纯色激光核 —— 用于全局扫描/锁定前捕获(防误检)。
# LASER_LAB_CENTER = 合并宽阈值, 窄+宽泛兜底 —— 用于锁定后 ROI 追踪(耐激光外观变化)。
RED_LASER_LAB_MERGED = [(38, 80, 22, 72, -30, 8)]    # 原主: 饱和正红
RED_LASER_LAB_WIDE   = [(40, 82, 18, 78, -34, 5)]    # 原弱光/偏色(仍收紧 B)
RED_LASER_LAB_HOT    = [(55, 95, 35, 88, -25, 25)]   # 原过曝兜底
RED_LASER_LAB_USER   = [(66, 87, 37, 58, 7, 33)]     # 你实测的激光 LAB
RED_LASER_LAB_USER2  = [(39, 60, 58, 79, 19, 38)]    # 你实测的激光 LAB 2 (2026-07-19 追加)
LASER_LAB_NARROW     = ([(90, 100, -7, 23, -12, 25)] +  # 窄中心核(2026-07-19 实测)
                        RED_LASER_LAB_USER2)       # 拼接实测窄值
LASER_LAB_CENTER     = (LASER_LAB_NARROW +            # 合并: 窄核优先 + 宽泛兜底
                        RED_LASER_LAB_MERGED +
                        RED_LASER_LAB_WIDE +
                        RED_LASER_LAB_HOT +
                        RED_LASER_LAB_USER)
LASER_MIN_PX = 3             # 最小像素从 6 降到 3, 远距/弱激光点不被砍掉
LASER_WASHOUT_PX = 1500      # 整屏泛红/反光拒识(>1500px 必为反光/红墙)
LASER_ACQUIRE_MAX_PX = 1000  # 从 260 放宽到 1000: 激光+辉光在 320x192 下可达数百px
LASER_LOCK_FRAMES = 2
# 锁定更"粘": 连续丢失帧数从 5 提到 8, 配合帧间运动预测, 短暂遮挡/运动模糊
# 不再轻易掉锁(短暂丢失由预测外推补齐, 只有真正长失锁才触发回退)。
LASER_LOST_FRAMES = 8
LASER_POS_ALPHA = 0.5         # 增大 EMA 系数, 激光位置跟手更快
LASER_ROI_SIZE = 120            # 从70放大到120: 激光快速移动时不易跑出ROI
LASER_TRACK_MAX_D = 100         # 从60放宽到100: 帧间许可移动距离放宽, 减少误丢

# ----- 帧间运动预测 (dead-reckoning), 根治"短暂丢失" -----
# 激光红点移动时, 若某帧检测掉链(运动模糊/瞬时遮挡/边缘), 不再直接宣告丢失,
# 而是用上一帧速度外推预测当前位置, 在预测点附近 ROI 搜索; 预测位置同时作为
# 闭环参考使控制连续。这样绝大多数"短暂丢失"被无缝补上, 不会触发回退/掉锁。
LASER_PREDICT_ENABLE = True
LASER_PREDICT_GAIN = 1.0      # 预测步长系数(1.0=按实测速度外推)
LASER_VEL_ALPHA = 0.4         # 速度 EMA 系数(越大越跟手, 越小越平滑)
LASER_ROI_MISS_EXPAND = 2.5   # 落空时 ROI 放大到 2.5 倍(175x175), 找回概率大增
# 锁定后每帧只做小 ROI 检测(极快); 仅每 LASER_GLOBAL_SWEEP_INTERVAL 帧做一次
# 全局兜底扫描, 用于激光偶然飘出 ROI 时重新找回。避免每帧全局 find_blobs 拖慢帧率。
LASER_GLOBAL_SWEEP_INTERVAL = 2   # 从4减到2: 激光飘出后全局找回更快

# ----- Laser-loss recovery (回退机制) -----
# 激光离开视野后, 若什么都不做: STM32 在 valid=0 时保持当前运动(不急停),
# 而 K230 的轨迹点仍沿矩形周长持续前进 -> 云台与靶点同时"跑飞", 越偏越多,
# 且无任何回退/重新捕获手段。修复策略(全部在视觉端完成, 无需改固件):
#   1) 激光丢失瞬间, 反转轨迹行走方向 -> 云台主动"回退"到激光最后已知点附近;
#   2) 丢失期间继续以 valid=True 发帧(用冻结的最后激光位置作参考), 让云台跟
#      随反转的轨迹点回到视野内, 而不是停在原地越飘越远;
#   3) 激光重新捕获时, 把轨迹点重新对齐到激光当前位置(最近周长点), 消除误差
#      跳变, 平滑续描;
#   4) 超时仍未捕获 -> 放弃回退, 改发 valid=False, 由 STM32 超时安全停(兜底,
#      避免云台永远空转)。
LASER_RECOVER_ENABLE = True
LASER_RECOVER_GIVEUP_MS = 4000   # 回退尝试超过此时长仍无激光 -> 放弃并安全停

# ----- Output protocol -----
# "binary" matches the STM32 gimbal firmware (default, runs out of the box).
# "ascii" sends "$cx,cy#" for a TI-Cup style motor that aims AT the point.
OUTPUT_PROTOCOL = "binary"

# ----- Loop timing -----
# 自适应检测频率(边框吸附快 + 高速稳):
#   - 未锁定 / 边框在动(rect_moving) -> 每帧检测(DETECT_INTERVAL 被忽略)
#   - 锁定且静止 -> 每 DETECT_INTERVAL 帧检测(省算力)
# 激光红点每帧检测(LASER_INTERVAL=1) 配帧间运动预测压住"短暂丢失"。激光检测本身
# 是帧率大头:
#   - 三级 LAB 阈值合并为单次 find_blobs (原顺序 3 次 -> 1 次)
#   - 锁定后每帧只做小 ROI 检测; 全局搜索节流为每 LASER_GLOBAL_SWEEP_INTERVAL 帧
#     一次兜底(激光飘出 ROI 时找回), 避免每帧全局 find_blobs 拖慢帧率
#   - GC 降频: 每 30 帧, 减少单帧回收卡顿
DETECT_INTERVAL = 2
UART_SEND_INTERVAL = 1
OSD_REFRESH_INTERVAL = 1
DEBUG_PRINT_INTERVAL = 30
# 垃圾回收周期: 官方例程每帧都 gc.collect() 防止 K230 小堆碎片化后静默卡死,
# 故默认每帧回收(GC_INTERVAL=1)。设大可省一点 CPU, 但堆碎片累积有卡死风险。
GC_INTERVAL = 1
# 检测看门狗: 单次矩形检测耗时超过此值(ms)即判定不可靠, 本次运行永久关闭 cv2
# 路径, 回退到稳定的 find_rects, 避免单帧卡死拖垮整个循环。
DETECT_TIMEOUT_MS = 120
LASER_INTERVAL = 1

# ----- Status files (board verifiable without serial logging) -----
STATUS_FILES_ENABLE = True
STATUS_DIR = "/data"
STATUS_RUNNING_FRAME = 30

# ----- Runtime tuning (上板实测, 免重烧) -----
# 把调参写进 TUNE_CONFIG_FILE(每行 "key=value"), 主循环每 TUNE_RELOAD_INTERVAL
# 帧重读一次并即时生效; 生效值回写到 /data/ti_vision_tune.txt 供上板核对(无需串口)。
# 删掉 .conf 文件 → 所有参数恢复本文件代码默认值。可用 key:
#   canny1=30        低阈值: 边缘淡/暗光下调多抓边
#   canny2=90        高阈值: 噪点多/误检多上调收紧(必须 > canny1)
#   rect_thresh=6000 fallback find_rects 灰度阈值
#   bin=1           二值化预处理开关(0=关, 对原灰度图直接 find_rects; 1=开, 默认)
#   bin_lo=0 bin_hi=110  二值化阈值: 保留灰度∈[bin_lo,bin_hi]像素(暗框亮背景取低段)
#   reg_w=0.02       "最标准"偏好权重(0=纯最大+居中)
#   reg_aspect=1.0   目标宽高比(非方形矩形改成实际值)
#   detect_interval=6 矩形检测降频(1=每帧, 大=省算力)
#   approx_eps=0.04  轮廓多边形近似精度(调大更容忍畸变边)
#   laser_int=1      激光检测降频(>1 提帧率, 代价短暂丢失找回慢)
#   laser_sweep=4    激光全局兜底扫描间隔帧(调大提帧率)
#   osd_x=0 osd_y=0  OSD 框校准偏移(显示像素), 量出偏移填这里消除指示误差
#   track_w=0.3      时间一致性权重(抗误切换/抗丢帧; 0=关, 过大死黏旧目标)
#   use_cv2=1        是否用官方 cv2 检测(1=开, 默认开; 0=强制回退 img.find_rects)
# 数值越界或该行非法 → 忽略该行; .conf 不存在 → 恢复出厂默认。
TUNE_CONFIG_FILE = STATUS_DIR + "/ti_vision_tune.conf"
TUNE_RELOAD_INTERVAL = 15
_TUNE_BOUNDS = {
    "canny1": (1, 254), "canny2": (2, 255),
    "rect_thresh": (500, 60000), "reg_w": (0.0, 1.0),
    "reg_aspect": (0.1, 10.0), "detect_interval": (1, 30),
    "approx_eps": (0.01, 0.20),
    "laser_int": (1, 10), "laser_sweep": (5, 120),
    "osd_x": (-300, 300), "osd_y": (-300, 300),
    "track_w": (0.0, 3.0), "use_cv2": (0, 1),
    "bin": (0, 1), "bin_lo": (0, 254), "bin_hi": (1, 255),
    "snap_a": (0.05, 1.0), "move_px": (0.0, 20.0), "hold_f": (0, 30),
}
# 代码默认值快照: 删掉 .conf 即恢复这些(在常量定义后捕获一次, 不被运行期改写)
_TUNE_DEFAULTS = {
    "canny1": CANNY_THRESH1, "canny2": CANNY_THRESH2,
    "rect_thresh": RECT_THRESHOLD, "reg_w": RECT_REGULARITY_WEIGHT,
    "reg_aspect": RECT_TARGET_ASPECT, "detect_interval": DETECT_INTERVAL,
    "approx_eps": APPROX_EPSILON,
    "laser_int": LASER_INTERVAL, "laser_sweep": LASER_GLOBAL_SWEEP_INTERVAL,
    "osd_x": OSD_X_OFFSET, "osd_y": OSD_Y_OFFSET,
    "track_w": RECT_TRACK_WEIGHT, "use_cv2": USE_CV2,
    "bin": RECT_BINARIZE, "bin_lo": BINARY_LO, "bin_hi": BINARY_HI,
    "snap_a": RECT_SNAP_ALPHA, "move_px": RECT_MOVE_PX_PER_FRAME,
    "hold_f": RECT_HOLD_FRAMES,
}


sensor = None
uart = None
osd_img = None
_CV2_DISABLED = False   # set True if cv2 import/run fails on-board -> use find_rects


def clamp(value, low, high):
    if value < low:
        return low
    if value > high:
        return high
    return value


def status_path(name):
    return STATUS_DIR + "/ti_vision_" + name + ".txt"


def write_status(name, text):
    if not STATUS_FILES_ENABLE:
        return
    status_file = None
    try:
        status_file = open(status_path(name), "w")
        status_file.write(str(text))
    except BaseException:
        pass
    finally:
        if status_file is not None:
            try:
                status_file.close()
            except BaseException:
                pass


def clear_status_files():
    if not STATUS_FILES_ENABLE:
        return
    for name in ("boot", "camera_ok", "running", "error", "stopped"):
        try:
            os.remove(status_path(name))
        except BaseException:
            pass


def load_tune_config():
    """Read TUNE_CONFIG_FILE and apply any valid key=value lines to the live
    detection parameters. Runs every TUNE_RELOAD_INTERVAL frames with zero
    reflash. Missing/illegal lines are ignored; absent file reverts everything
    to the code defaults captured in _TUNE_DEFAULTS.
    """
    global CANNY_THRESH1, CANNY_THRESH2, RECT_THRESHOLD, RECT_REGULARITY_WEIGHT, \
        RECT_TARGET_ASPECT, DETECT_INTERVAL, APPROX_EPSILON, \
        LASER_INTERVAL, LASER_GLOBAL_SWEEP_INTERVAL, OSD_X_OFFSET, OSD_Y_OFFSET, \
        RECT_TRACK_WEIGHT, USE_CV2, RECT_BINARIZE, BINARY_LO, BINARY_HI, \
        RECT_SNAP_ALPHA, RECT_MOVE_PX_PER_FRAME, RECT_HOLD_FRAMES

    if not STATUS_FILES_ENABLE:
        return

    conf = None
    try:
        conf = open(TUNE_CONFIG_FILE, "r")
    except BaseException:
        conf = None

    if conf is None:
        applied = dict(_TUNE_DEFAULTS)  # 文件缺失 → 恢复出厂默认
    else:
        applied = {}  # 仅覆盖 .conf 中显式写出的 key; 其余保持当前值
        try:
            for raw_line in conf:
                line = raw_line.strip()
                if not line or line.startswith("#"):
                    continue
                if "=" not in line:
                    continue
                key, _, val = line.partition("=")
                key = key.strip().lower()
                val = val.strip()
                if key not in _TUNE_BOUNDS:
                    continue
                try:
                    num = float(val)
                    if num.is_integer() and key not in (
                        "reg_w", "reg_aspect", "angle_cos", "approx_eps", "track_w"
                    ):
                        num = int(num)
                except BaseException:
                    continue
                lo, hi = _TUNE_BOUNDS[key]
                if num < lo or num > hi:
                    continue
                applied[key] = num
        finally:
            try:
                conf.close()
            except BaseException:
                pass

    CANNY_THRESH1 = int(applied.get("canny1", CANNY_THRESH1))
    CANNY_THRESH2 = int(applied.get("canny2", CANNY_THRESH2))
    RECT_THRESHOLD = int(applied.get("rect_thresh", RECT_THRESHOLD))
    RECT_REGULARITY_WEIGHT = float(applied.get("reg_w", RECT_REGULARITY_WEIGHT))
    RECT_TARGET_ASPECT = float(applied.get("reg_aspect", RECT_TARGET_ASPECT))
    DETECT_INTERVAL = int(applied.get("detect_interval", DETECT_INTERVAL))
    APPROX_EPSILON = float(applied.get("approx_eps", APPROX_EPSILON))
    LASER_INTERVAL = int(applied.get("laser_int", LASER_INTERVAL))
    LASER_GLOBAL_SWEEP_INTERVAL = int(applied.get("laser_sweep", LASER_GLOBAL_SWEEP_INTERVAL))
    OSD_X_OFFSET = int(applied.get("osd_x", OSD_X_OFFSET))
    OSD_Y_OFFSET = int(applied.get("osd_y", OSD_Y_OFFSET))
    RECT_TRACK_WEIGHT = float(applied.get("track_w", RECT_TRACK_WEIGHT))
    RECT_SNAP_ALPHA = float(applied.get("snap_a", RECT_SNAP_ALPHA))
    RECT_MOVE_PX_PER_FRAME = float(applied.get("move_px", RECT_MOVE_PX_PER_FRAME))
    RECT_HOLD_FRAMES = int(applied.get("hold_f", RECT_HOLD_FRAMES))
    RECT_BINARIZE = bool(int(applied.get("bin", RECT_BINARIZE)))
    BINARY_LO = int(applied.get("bin_lo", BINARY_LO))
    BINARY_HI = int(applied.get("bin_hi", BINARY_HI))
    # 运行时开关官方 cv2 检测(默认开; use_cv2=0 强制回退 img.find_rects)
    if "use_cv2" in applied:
        USE_CV2 = bool(int(applied["use_cv2"]))
    # 保证高阈值严格大于低阈值
    if CANNY_THRESH2 <= CANNY_THRESH1:
        CANNY_THRESH2 = CANNY_THRESH1 + 1

    write_status(
        "tune",
        "canny1={} canny2={} rect_thresh={} reg_w={} reg_aspect={} detect={} "
        "approx_eps={} laser_int={} laser_sweep={} osd=({}, {}) "
        "track_w={} use_cv2={} bin={} bin_lo={} bin_hi={} snap_a={} move_px={} hold_f={}".format(
            CANNY_THRESH1, CANNY_THRESH2, RECT_THRESHOLD, RECT_REGULARITY_WEIGHT,
            RECT_TARGET_ASPECT, DETECT_INTERVAL, APPROX_EPSILON,
            LASER_INTERVAL, LASER_GLOBAL_SWEEP_INTERVAL, OSD_X_OFFSET, OSD_Y_OFFSET,
            RECT_TRACK_WEIGHT, USE_CV2, RECT_BINARIZE, BINARY_LO, BINARY_HI,
            RECT_SNAP_ALPHA, RECT_MOVE_PX_PER_FRAME, RECT_HOLD_FRAMES,
        ),
    )


def elapsed_seconds(now_ms, previous_ms):
    dt = time.ticks_diff(now_ms, previous_ms) / 1000.0
    return clamp(dt, 0.005, 0.25)


# ---------------------------------------------------------------
# Binary protocol (matches STM32 gimbal firmware)
#   0x3C 0x3B [XH][XL][YH][YL][FLAG] [CRC8] 0x01 0x01  (10 bytes)
#   FLAG bit0=valid, bit1=hit(done), bit2=hi_conf
# ---------------------------------------------------------------

def crc8(data):
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
    if not uart:
        return 0
    buf = bytearray(10)
    buf[0] = 0x3C
    buf[1] = 0x3B
    x16 = int(clamp(x, -32768, 32767))
    y16 = int(clamp(y, -32768, 32767))
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
    return uart.write(buf)


def send_ascii(uart, cx, cy, valid):
    if not uart:
        return 0
    if valid and cx is not None and cy is not None:
        uart.write("${},{}#".format(int(cx), int(cy)))
    else:
        uart.write("$-1,-1#")
    return 1


def send_trace(uart, desired_frame, laser_frame, valid, done):
    """Dispatch the current trace point to the gimbal.

    Binary mode sends the pixel error the firmware must null.
    Ascii mode sends the absolute desired point for an aim-at-point motor.
    """
    if OUTPUT_PROTOCOL == "ascii":
        ok = desired_frame is not None
        return send_ascii(uart, desired_frame[0] if desired_frame else None,
                          desired_frame[1] if desired_frame else None, ok)
    # binary: err = desired - reference(laser or center)
    if desired_frame is not None and valid:
        if laser_frame is not None:
            ex = desired_frame[0] - laser_frame[0]
            ey = desired_frame[1] - laser_frame[1]
        else:
            ex = desired_frame[0] - FRAME_W // 2
            ey = desired_frame[1] - FRAME_H // 2
    else:
        ex = ey = 0
    return send_binary(uart, ex, ey, valid, done, valid)


# ---------------------------------------------------------------
# Rectangle geometry / perimeter trajectory
# ---------------------------------------------------------------

def box_corners(rect):
    """Return the 4 ordered corners of a rect. Accepts either a 4-tuple
    (x, y, w, h) or a MaixPy Rect object (which has .corners() giving the
    true *tilted/rotated* quad). Using the real corners lets the trace follow
    a tilted rectangle instead of only its axis-aligned bounding box.
    """
    try:
        c = rect.corners()
        if c and len(c) == 4:
            return order_corners([(int(p[0]), int(p[1])) for p in c])
    except BaseException:
        pass
    x, y, w, h = rect
    return [(x, y), (x + w, y), (x + w, y + h), (x, y + h)]


def order_corners(corners):
    """Sort 4 points by angle around their centroid for a consistent winding."""
    n = len(corners)
    cx = sum(p[0] for p in corners) / n
    cy = sum(p[1] for p in corners) / n
    return sorted(corners, key=lambda p: math.atan2(p[1] - cy, p[0] - cx))


def rect_center_of(corners):
    n = len(corners)
    return (sum(p[0] for p in corners) / n, sum(p[1] for p in corners) / n)


def choose_snap_alpha(jump, ref_size):
    """贴合速率选择: 边框中心相对锁定框位移 > TRACE_REBUILD_DRIFT*ref_size ->
    整框贴合(alpha=1.0, 快贴合防拖影); 否则用 RECT_SNAP_ALPHA 平滑吸附。
    ref_size 取锁定框较长边(proc px)。"""
    if TRACE_REBUILD_DRIFT > 0 and jump > TRACE_REBUILD_DRIFT * ref_size:
        return 1.0
    return RECT_SNAP_ALPHA


def snap_corners_to_target(locked, detected, alpha):
    """角点 EMA 吸附: 把锁定 4 角点朝真实检测角点贴合 alpha 比例。返回新角点列表。
    alpha=RECT_SNAP_ALPHA 时平滑跟手; alpha=1.0 时本帧整框贴合(大跳变快收敛)。"""
    return [
        (locked[i][0] + alpha * (detected[i][0] - locked[i][0]),
         locked[i][1] + alpha * (detected[i][1] - locked[i][1]))
        for i in range(4)
    ]


class RectTrajectory:
    """Closed perimeter of a rectangle, sampled into a dense point list so a
    set-point can walk along it at constant arc-length speed."""

    def __init__(self, corners, spacing):
        self.corners = corners
        self.spacing = spacing
        self.samples = []
        self.total_len = 0.0
        self._build()

    def _build(self):
        self.samples = []
        self.corner_arcs = []      # arc position (proc px) of each original corner vertex
        self.corner_points = []    # (x, y) of each original corner vertex
        n = len(self.corners)
        idx = 0
        for i in range(n):
            x0, y0 = self.corners[i]
            x1, y1 = self.corners[(i + 1) % n]
            # the start of edge i is exactly corner i -> record its arc position
            self.corner_arcs.append(idx * self.spacing)
            self.corner_points.append((x0, y0))
            seg_len = math.sqrt((x1 - x0) * (x1 - x0) + (y1 - y0) * (y1 - y0))
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
    """Circular (wrap-aware) distance in arc from `arc` to the closest corner."""
    best = total
    for ca in corner_arcs:
        d = abs(arc - ca)
        if d > total * 0.5:
            d = total - d
        if d < best:
            best = d
    return best


def next_corner_ahead(arc, direction, corner_arcs, total, enter):
    """Return the arc position of the nearest corner that lies within `enter`
    proc-px AHEAD of `arc` along `direction` (1 or -1), else None. Wrap-aware."""
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


# ---------------------------------------------------------------
# Rectangle detection
# ---------------------------------------------------------------

def _corners_bbox(corners):
    """Axis-aligned bounding box (x, y, w, h) of a 4-point corner list."""
    xs = [p[0] for p in corners]
    ys = [p[1] for p in corners]
    x0 = min(xs)
    y0 = min(ys)
    return x0, y0, max(xs) - x0, max(ys) - y0


def _score_corners(corners, prev_corners):
    """Score one 4-point rectangle (anti false-detection). Mirrors the previous
    choose_target_rect scoring but operates on corner points, so the cv2 path
    and the find_rects fallback share identical selection logic.

    Returns None if the candidate fails the hard size/aspect gates.
    """
    x0, y0, w, h = _corners_bbox(corners)
    if w <= 0 or h <= 0:
        return None
    area = w * h
    if area < RECT_MIN_AREA or area > RECT_MAX_AREA:
        return None
    # 矩形碰画面边缘 → 拒识 (不完整矩形会导致轨迹偏差/误跟踪)
    m = RECT_EDGE_MARGIN
    if x0 < m or y0 < m or (x0 + w) > (PROC_W - m) or (y0 + h) > (PROC_H - m):
        return None
    short = min(w, h)
    long = max(w, h)
    if short <= 0:
        return None
    aspect = long / short
    if aspect > RECT_MAX_ASPECT:
        return None
    cx = x0 + w / 2.0
    cy = y0 + h / 2.0
    frame_cx = PROC_W // 2
    frame_cy = PROC_H // 2
    center_penalty = abs(cx - frame_cx) + abs(cy - frame_cy)
    # “最标准”: 面积相近时, 宽高比越接近目标越优(权重很小, 仅打破平手)
    reg_penalty = abs(aspect - RECT_TARGET_ASPECT) * area * RECT_REGULARITY_WEIGHT
    # 时间一致性: 偏离上一帧目标太远 / 尺寸突变 → 扣分, 抗误切换与抗丢帧
    track_penalty = 0.0
    if prev_corners is not None and RECT_TRACK_WEIGHT > 0:
        px0, py0, pw, ph = _corners_bbox(prev_corners)
        prev_area = pw * ph
        if prev_area > 0:
            prev_cx = px0 + pw / 2.0
            prev_cy = py0 + ph / 2.0
            jump = abs(cx - prev_cx) + abs(cy - prev_cy)
            size_ratio = area / prev_area
            size_pen = abs(math.log(max(size_ratio, 1e-3))) * 3000.0
            track_penalty = (jump + size_pen) * RECT_TRACK_WEIGHT
    return area - center_penalty * 2 - reg_penalty - track_penalty


def choose_target_corners(corner_cands, prev_corners=None):
    """Pick the highest-scoring 4-point rectangle from a candidate list."""
    best = None
    best_score = -1e18
    for corners in corner_cands:
        score = _score_corners(corners, prev_corners)
        if score is None:
            continue
        if score > best_score:
            best_score = score
            best = corners
    return best


def _approx_to_pts(approx):
    """Flatten a cv2.approxPolyDP result into a list of (x, y) int tuples.

    The CanMV cv2 port may return contours/approx as (N,1,2) or (N,2) arrays,
    or plain Python lists, so accept both shapes defensively.
    """
    pts = []
    for p in approx:
        try:
            sub = p[0]
            if isinstance(sub, (list, tuple)) or (
                hasattr(sub, "__len__") and len(sub) == 2
            ):
                pts.append((int(sub[0]), int(sub[1])))
            else:
                pts.append((int(p[0]), int(p[1])))
        except (TypeError, IndexError):
            try:
                pts.append((int(p[0]), int(p[1])))
            except (TypeError, IndexError):
                continue
    return pts


def find_rect_corners_cv2(img, prev_corners=None):
    """Detect the best rectangle with CanMV OpenCV (cv2).

    Pipeline: grayscale snapshot -> binary threshold (dark target -> white) ->
    Gaussian blur -> Canny -> findContours -> approxPolyDP (4-point) -> score pick.
    """
    gray = img.to_numpy_ref()
    # ensure 2D single-channel uint8 (a grayscale snapshot may arrive 3D)
    if hasattr(gray, "ndim") and gray.ndim == 3:
        gray = gray[:, :, 0]
    # --- 二值化: 暗目标(黑框/暗矩形) -> 白(255) ---
    if RECT_BINARIZE:
        _, mask = cv2.threshold(gray, BINARY_HI, 255, cv2.THRESH_BINARY_INV)
    else:
        mask = gray
    # --- 高斯模糊去噪 ---
    if GAUSSIAN_BLUR_SIZE and GAUSSIAN_BLUR_SIZE > 1:
        k = GAUSSIAN_BLUR_SIZE if GAUSSIAN_BLUR_SIZE % 2 == 1 else GAUSSIAN_BLUR_SIZE + 1
        mask = cv2.GaussianBlur(mask, (k, k), 0)
    # --- Canny 边缘 ---
    edges = cv2.Canny(mask, CANNY_THRESH1, CANNY_THRESH2)
    # --- 轮廓 ---
    res = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours = res[1] if len(res) == 3 else res[0]
    cands = []
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < RECT_MIN_AREA or area > RECT_MAX_AREA:
            continue
        peri = cv2.arcLength(cnt, True)
        if peri <= 0:
            continue
        approx = cv2.approxPolyDP(cnt, APPROX_EPSILON * peri, True)
        pts = _approx_to_pts(approx)
        if len(pts) != 4:
            continue
        cands.append(order_corners(pts))
    return choose_target_corners(cands, prev_corners)


def find_rect_corners(img, prev_corners=None):
    """Detect the best rectangle, return its 4 corners (proc space).

    Primary path: CanMV OpenCV (cv2) — matches the "二值化+Canny+取最大最规整
    quadrilateral" ask and runs on the official cv2 binding (no cv_lite hard
    fault). If cv2 is unavailable or throws / is pathologically slow on-board,
    we fall back once to img.find_rects (MaixPy C) for the rest of the run.

    prev_corners feeds the temporal-consistency scoring (anti false-switch /
    anti loss) on both paths.
    """
    global _CV2_DISABLED
    if USE_CV2 and HAS_CV2 and not _CV2_DISABLED:
        try:
            t0 = time.ticks_ms()
            corners = find_rect_corners_cv2(img, prev_corners)
            if time.ticks_diff(time.ticks_ms(), t0) > DETECT_TIMEOUT_MS:
                print("cv2 detect slow ({}ms) -> fallback to find_rects".format(
                    time.ticks_diff(time.ticks_ms(), t0)))
                _CV2_DISABLED = True
            else:
                return corners
        except BaseException as error:
            print("cv2 detect error:", error, "-> fallback to find_rects")
            _CV2_DISABLED = True
    # 兜底: MaixPy img.find_rects (C 实现, 官方矩形例程同款, 抗旋转/仿射)
    detect_img = img.binary([(BINARY_LO, BINARY_HI)]) if RECT_BINARIZE else img
    rects = detect_img.find_rects(threshold=RECT_THRESHOLD)
    corner_cands = [box_corners(r) for r in rects]
    return choose_target_corners(corner_cands, prev_corners)


# ---------------------------------------------------------------
# Laser dot detection (RGB stream, LAB red)
# ---------------------------------------------------------------

def _filter_laser_blobs(blobs, acquire=False):
    """Apply pixel-count / aspect-ratio / fill-ratio rejection.

    acquire=True 用于初始捕获: 额外限制最大像素(LASER_ACQUIRE_MAX_PX),
    防止把一整片红色墙面/反光区域当成激光点。
    """
    cands = []
    max_px = LASER_ACQUIRE_MAX_PX if acquire else LASER_WASHOUT_PX
    for b in blobs:
        px = b.pixels()
        if not (LASER_MIN_PX <= px <= max_px):
            continue
        bw, bh = b.w(), b.h()
        if bw < 1 or bh < 1:
            continue
        ratio = float(max(bw, bh)) / float(min(bw, bh))
        if ratio > 2.2:
            # 激光点近圆; 过长条状红斑(如线状反光)拒掉
            continue
        fill = float(px) / float(bw * bh)
        if fill < 0.45:
            # 实心底红点紧凑度高; 稀疏/环状斑拒掉
            continue
        cands.append((b, px, ratio, fill))
    return cands


def _nearest_in_window(cands, ref_x, ref_y, max_d):
    """Return (cx, cy, True) of the candidate nearest to (ref_x,ref_y) within
    max_d px, else (-1, -1, False). The distance window is the key anti-jump
    guard: when locked, a real laser cannot teleport >max_d in one frame, so any
    far candidate is a false detection and must be rejected (never accepted).
    """
    best = None
    best_d2 = max_d * max_d
    for b, px, ratio, fill in cands:
        dx = b.cx() - ref_x
        dy = b.cy() - ref_y
        d2 = dx * dx + dy * dy
        if d2 <= best_d2:
            best_d2 = d2
            best = (b.cx(), b.cy())
    if best is None:
        return -1, -1, False
    return best[0], best[1], True


def detect_laser(laser_img, prev_x, prev_y, pred_x, pred_y, locked, pred_ok,
                 roi_expand=1.0, do_global=False):
    if not laser_img:
        return -1, -1, False

    # --- 1) ROI 搜索(锚定预测点, 用合并阈值, 宽容) ---
    # 锁定后每帧都做这个: 只在"激光应该在的位置"附近小窗找, 极快且天然抗背景干扰。
    if locked and prev_x >= 0 and prev_y >= 0:
        cx = pred_x if pred_ok else prev_x
        cy = pred_y if pred_ok else prev_y
        size = int(LASER_ROI_SIZE * roi_expand)
        hs = size // 2
        rx = max(0, int(cx - hs))
        ry = max(0, int(cy - hs))
        rw = min(size, PROC_W - rx)
        rh = min(size, PROC_H - ry)
        if rw >= 10 and rh >= 10:
            roi = (rx, ry, rw, rh)
            blobs = laser_img.find_blobs(
                LASER_LAB_CENTER, False, roi=roi,
                x_stride=1, y_stride=1, pixels_threshold=LASER_MIN_PX,
                merge=True, margin=True,
            )
            cands = _filter_laser_blobs(blobs)
            if cands:
                rx2, ry2, ok = _nearest_in_window(
                    cands, cx, cy, LASER_TRACK_MAX_D * roi_expand)
                if ok:
                    return rx2, ry2, True
            # ROI 落空不在此处做全局, 交由下方 do_global 决定(避免每帧全局)

    # --- 2) 全局搜索(用窄阈值, 防误检): 仅初始捕获 或 周期性兜底(do_global) ---
    # 4 组合并为单次 find_blobs。
    # 初始捕获(locked=False)用 stride=1 每像素扫, 不漏小激光;
    # 锁定后的周期性兜底(locked=True)用 stride=2 省开销(快速扫)。
    stride = 2 if locked else 1
    if (not locked) or do_global:
        blobs = laser_img.find_blobs(
            LASER_LAB_NARROW, False,
            x_stride=stride, y_stride=stride, pixels_threshold=LASER_MIN_PX,
            merge=True, margin=True,
        )
        cands = _filter_laser_blobs(blobs, acquire=(not locked))
        if cands:
            if locked and prev_x >= 0 and prev_y >= 0:
                ref_x = pred_x if pred_ok else prev_x
                ref_y = pred_y if pred_ok else prev_y
                gx, gy, ok = _nearest_in_window(
                    cands, ref_x, ref_y, LASER_TRACK_MAX_D * roi_expand)
                if ok:
                    return gx, gy, True
                # 全局找到的候选全离预测点过远 -> 判定丢失, 交由帧间预测保持,
                # 绝不跳到远处 blob(旧逻辑会改取"面积×紧凑度最大"的远处斑 -> 拖偏云台)。
                return -1, -1, False
            else:
                # 初始捕获: 只从 <=ACQUIRE_MAX_PX 的候选中选"紧凑度最高、面积偏小"者,
                # 避免把大片红墙/反光当激光(旧逻辑选面积×紧凑度最大, 易被大红区骗)。
                best = max(cands, key=lambda t: t[3] / (1.0 + t[1] * 0.01))
                return best[0].cx(), best[0].cy(), True

    return -1, -1, False


# ---------------------------------------------------------------
# Scaling helpers (proc 320x192 -> frame 800x480)
# ---------------------------------------------------------------

def scale_point_to_frame(point):
    if point is None:
        return None
    x = point[0] * FRAME_W / PROC_W + OSD_X_OFFSET
    y = point[1] * FRAME_H / PROC_H + OSD_Y_OFFSET
    return (int(round(x)), int(round(y)))


def scale_rect_to_frame(rect):
    if rect is None:
        return None
    x, y, w, h = rect
    sx = x * FRAME_W // PROC_W + OSD_X_OFFSET
    sy = y * FRAME_H // PROC_H + OSD_Y_OFFSET
    sw = w * FRAME_W // PROC_W
    sh = h * FRAME_H // PROC_H
    return (int(sx), int(sy), int(sw), int(sh))


# ---------------------------------------------------------------
# OSD
# ---------------------------------------------------------------

def draw_osd(canvas, corner_frame, start_frame, desired_frame, laser_frame,
             progress, done, fps_value, recovering):
    canvas.clear()
    red = (255, 0, 0)
    yellow = (255, 255, 0)
    white = (255, 255, 255)
    green = (0, 200, 0)
    black = (0, 0, 0)

    # Aiming center reference.
    canvas.draw_cross(FRAME_W // 2, FRAME_H // 2, color=yellow, size=14, thickness=2)

    if corner_frame is not None:
        # The closed figure: 4 edges of the rectangle border.
        n = len(corner_frame)
        for i in range(n):
            a = corner_frame[i]
            b = corner_frame[(i + 1) % n]
            canvas.draw_line(a[0], a[1], b[0], b[1], color=green, thickness=4)
        # Start point marker.
        if start_frame is not None:
            canvas.draw_circle(start_frame[0], start_frame[1], 6, white, 2)

    # 边框上移动的红色设定点十字已取消(用户要求, 省绘制开销提帧率)。

    if laser_frame is not None:
        canvas.draw_cross(laser_frame[0], laser_frame[1], color=yellow, size=14, thickness=2)

    canvas.draw_string_advanced(10, 3, 28, "FPS:{:.1f}".format(fps_value), color=white)
    canvas.draw_string_advanced(10, 39, 22, "TRACE {:.0f}%".format(progress * 100.0), color=green)
    if done:
        canvas.draw_string_advanced(180, 39, 22, "DONE", color=red)

    if recovering:
        amber = (255, 165, 0)
        canvas.draw_string_advanced(330, 39, 24, "RECOVER", color=amber)
        canvas.draw_circle(FRAME_W // 2, FRAME_H // 2, 40, amber, 3)


# ---------------------------------------------------------------
# UART / camera init
# ---------------------------------------------------------------

def init_uart():
    fpioa = FPIOA()
    fpioa.set_function(UART_TX_PIN, FPIOA.UART2_TXD)
    fpioa.set_function(UART_RX_PIN, FPIOA.UART2_RXD)
    return UART(
        UART.UART2,
        baudrate=UART_BAUDRATE,
        bits=UART.EIGHTBITS,
        parity=UART.PARITY_NONE,
        stop=UART.STOPBITS_ONE,
    )


def camera_init():
    global sensor, uart, osd_img

    os.exitpoint(os.EXITPOINT_ENABLE)
    uart = init_uart()

    sensor = Sensor(
        id=SENSOR_ID,
        width=SENSOR_INPUT_W,
        height=SENSOR_INPUT_H,
        fps=SENSOR_FPS,
    )
    sensor.reset()
    try:
        sensor.set_hmirror(HMIRROR)
        sensor.set_vflip(VFLIP)
    except BaseException:
        pass

    # Hardware video pass-through (CHN0) for the LCD.
    sensor.set_framesize(width=FRAME_W, height=FRAME_H, chn=CAM_CHN_ID_0)
    sensor.set_pixformat(Sensor.YUV420SP, chn=CAM_CHN_ID_0)
    bind_info = sensor.bind_info()
    Display.bind_layer(**bind_info, layer=Display.LAYER_VIDEO1)

    # CHN1: grayscale 320x192 for rectangle border detection.
    sensor.set_framesize(width=PROC_W, height=PROC_H, chn=CAM_CHN_ID_1)
    sensor.set_pixformat(Sensor.GRAYSCALE, chn=CAM_CHN_ID_1)

    # CHN2: RGB565 320x192 for laser dot detection (LAB red).
    sensor.set_framesize(width=PROC_W, height=PROC_H, chn=CAM_CHN_ID_2)
    sensor.set_pixformat(Sensor.RGB565, chn=CAM_CHN_ID_2)

    Display.init(
        Display.ST7701,
        width=FRAME_W,
        height=FRAME_H,
        to_ide=True,
        quality=IDE_JPEG_QUALITY,
        osd_num=2,
    )
    osd_img = image.Image(FRAME_W, FRAME_H, image.ARGB8888)
    MediaManager.init()
    sensor.run()

    try:
        gain = k_sensor_gain()
        gain.gain[0] = SENSOR_ANALOG_GAIN
        sensor.again(gain)
    except BaseException as error:
        print("sensor gain skipped:", error)


def camera_deinit():
    global sensor
    try:
        if isinstance(sensor, Sensor):
            sensor.stop()
    except BaseException as error:
        print("sensor stop error:", error)
    try:
        Display.deinit()
    except BaseException as error:
        print("display deinit error:", error)
    os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
    time.sleep_ms(100)
    try:
        MediaManager.deinit()
    except BaseException as error:
        print("media deinit error:", error)


# ---------------------------------------------------------------
# Main
# ---------------------------------------------------------------

def main():
    global sensor, uart, osd_img

    camera_is_init = False
    frame_id = 0

    # Trajectory state.
    traj = None                 # RectTrajectory
    traj_corners_proc = None    # locked corners (proc space)
    traj_rect_proc = None       # locked (x,y,w,h)
    traj_start_frame = None     # start point (frame space)
    traj_corner_frame = None    # 4 corners (frame space) for OSD
    traj_arc = 0.0              # current arc length (proc px)
    traj_dir = TRACE_DIR
    traj_done = False
    traj_locked = False
    traj_dwell_until = 0        # ms timestamp until which we hold on a corner (0 = not dwelling)
    traj_dwell_arc = 0.0        # arc position of the corner we are dwelling on
    missed_rect = 0
    last_traj_ms = time.ticks_ms()

    # 边框运动跟踪(自适应检测频率 + 丢失外推)
    rect_moving = False        # 边框是否在快速移动(强制每帧检测)
    rect_speed = 0.0           # 边框中心帧间位移 EMA (proc px/帧)
    rect_last_center = None    # 上次成功检测的中心
    rect_last_frame = 0        # 上次成功检测的帧号
    rect_prev_corners = None   # 上次成功检测的原始角点(算角点速度)
    rect_corner_vel = [(0.0, 0.0), (0.0, 0.0), (0.0, 0.0), (0.0, 0.0)]  # 角点速度 EMA

    # Set-point smoothing.
    smooth_x = 0.0
    smooth_y = 0.0
    smooth_init = False

    # Laser temporal tracking.
    l_lock = False
    l_hit_cnt = 0
    l_miss_cnt = 0
    lx_s = 0
    ly_s = 0

    # Laser-loss recovery state (回退机制).
    recovering = False
    recover_start_ms = 0
    last_good_laser_proc = None   # (px, py) in proc space, frozen last locked laser pos

    # Laser frame-to-frame motion prediction (dead-reckoning).
    l_vx = 0.0                    # smoothed velocity (proc px/s)
    l_vy = 0.0
    l_last_det_ms = 0            # timestamp of last detection attempt
    l_prev_miss = False          # whether the previous detection frame missed
    l_global_cd = 0              # 全局兜底扫描冷却帧计数(锁定后节流用)

    clock = time.clock()

    try:
        clear_status_files()
        write_status("boot", "main entered")
        camera_init()
        camera_is_init = True
        write_status("camera_ok", "camera initialized")
        print("ti_cup_e_rect_trace start")

        while True:
            os.exitpoint()
            clock.tick()
            frame_id += 1

            # --- 运行时调参重读(上板免重烧; 默认每 15 帧) ---
            if TUNE_RELOAD_INTERVAL and frame_id % TUNE_RELOAD_INTERVAL == 0:
                load_tune_config()

            img = sensor.snapshot(chn=CAM_CHN_ID_1)

            # --- 激光检测(降频): 仅每 LASER_INTERVAL 帧读取 RGB 通道并做 find_blobs,
            #     其余帧复用上一帧的平滑位置 lx_s/ly_s, 省去一次 RGB snapshot + 检测开销 ---
            run_laser = LASER_ENABLE and (frame_id == 1 or frame_id % LASER_INTERVAL == 0)
            rgb = sensor.snapshot(chn=CAM_CHN_ID_2) if run_laser else None

            # --- laser detection (每帧) + 帧间运动预测 + 丢失外推 ---
            laser_det = False
            lx = ly = -1
            if run_laser and rgb:
                now_ms = time.ticks_ms()
                dt = elapsed_seconds(now_ms, l_last_det_ms) if l_last_det_ms else 0.02
                # 用上一帧速度外推激光"当前应在的位置", 作为本帧 ROI/参考中心
                if l_lock and LASER_PREDICT_ENABLE:
                    px_pred = clamp(lx_s + l_vx * dt * LASER_PREDICT_GAIN, 0, PROC_W - 1)
                    py_pred = clamp(ly_s + l_vy * dt * LASER_PREDICT_GAIN, 0, PROC_H - 1)
                    pred_ok = True
                else:
                    px_pred, py_pred, pred_ok = lx_s, ly_s, False
                roi_expand = LASER_ROI_MISS_EXPAND if l_prev_miss else 1.0
                # 全局兜底扫描节流: 锁定后每帧只做 ROI(极快); 仅初始捕获 或 每
                # LASER_GLOBAL_SWEEP_INTERVAL 帧做一次全局, 用于激光飘出 ROI 时找回。
                if not l_lock:
                    do_global = True
                    l_global_cd = LASER_GLOBAL_SWEEP_INTERVAL
                elif l_global_cd <= 0:
                    do_global = True
                    l_global_cd = LASER_GLOBAL_SWEEP_INTERVAL
                else:
                    do_global = False
                    l_global_cd -= 1
                lx, ly, laser_det = detect_laser(
                    rgb, lx_s if l_lock else -1, ly_s if l_lock else -1,
                    px_pred, py_pred, l_lock, pred_ok, roi_expand, do_global
                )
                if laser_det:
                    # 速度 EMA 更新(仅在已锁定时有意义)
                    if l_lock:
                        inst_vx = (lx - lx_s) / dt if dt > 0 else 0.0
                        inst_vy = (ly - ly_s) / dt if dt > 0 else 0.0
                        l_vx = l_vx * (1 - LASER_VEL_ALPHA) + inst_vx * LASER_VEL_ALPHA
                        l_vy = l_vy * (1 - LASER_VEL_ALPHA) + inst_vy * LASER_VEL_ALPHA
                    l_hit_cnt = min(l_hit_cnt + 1, 32)
                    l_miss_cnt = 0
                    l_prev_miss = False
                    if not l_lock and l_hit_cnt == 1:
                        lx_s, ly_s = lx, ly
                        l_vx = l_vy = 0.0
                    else:
                        dx = lx - lx_s
                        dy = ly - ly_s
                        d2 = dx * dx + dy * dy
                        if d2 > LASER_TRACK_MAX_D * LASER_TRACK_MAX_D:
                            scale = LASER_TRACK_MAX_D / math.sqrt(d2)
                            lx = int(lx_s + dx * scale)
                            ly = int(ly_s + dy * scale)
                        lx_s = int(lx_s * (1 - LASER_POS_ALPHA) + lx * LASER_POS_ALPHA)
                        ly_s = int(ly_s * (1 - LASER_POS_ALPHA) + ly * LASER_POS_ALPHA)
                    if l_hit_cnt >= LASER_LOCK_FRAMES and not l_lock:
                        l_lock = True
                        # 激光重新捕获: 把轨迹点重新对齐到激光当前位置
                        # (最近周长点), 消除误差跳变并平滑续描; 同时结束回退。
                        if traj is not None:
                            start_idx = traj.nearest_index((lx_s, ly_s))
                            traj_arc = start_idx * TRACE_SAMPLE_SPACING
                            traj_dir = TRACE_DIR
                            traj_dwell_until = 0
                            smooth_init = False
                        recovering = False
                        last_good_laser_proc = None
                        print("laser: re-acquired, re-synced to arc={:.1f}".format(traj_arc))
                else:
                    l_miss_cnt = min(l_miss_cnt + 1, 32)
                    l_hit_cnt = 0
                    l_prev_miss = True
                    # 短暂丢失: 用预测速度把"保持位置"外推前进, 控制参考连续,
                    # 云台不突跳; 锁仍在, 不触发回退。只有连续 LASER_LOST_FRAMES
                    # 帧都丢才真正掉锁并进入回退。
                    if l_lock:
                        lx_s = int(clamp(lx_s + l_vx * dt * LASER_PREDICT_GAIN, 0, PROC_W - 1))
                        ly_s = int(clamp(ly_s + l_vy * dt * LASER_PREDICT_GAIN, 0, PROC_H - 1))
                    if l_lock and l_miss_cnt >= LASER_LOST_FRAMES:
                        l_lock = False
                        if LASER_RECOVER_ENABLE and traj_locked and traj is not None:
                            # 激光刚离开视野: 反转轨迹方向, 让云台主动回退到
                            # 激光最后已知点附近(不再与靶点一起跑飞)。
                            recovering = True
                            recover_start_ms = time.ticks_ms()
                            last_good_laser_proc = (lx_s, ly_s)
                            traj_dir = -traj_dir
                            print("laser: LOST -> entering recovery (rewind)")
                l_last_det_ms = now_ms

            # --- rectangle detection (自适应频率 + 角点 EMA 吸附 + 丢失保持) ---
            # 自适应频率: 未锁定 / 边框在动 -> 每帧检测; 锁定且静止 -> 每 DETECT_INTERVAL 帧。
            # 角点 EMA 吸附: 锁定后每帧把锁定角点朝真实角点贴合并重建轨迹(保持设点位置),
            #   边框框(OSD)与循迹路径近实时跟住真实边框 -> 吸附快且平滑。
            # 丢失保持: 检测掉链(运动模糊)时保持上一框并外推, 识别"不掉", 轨迹继续走。
            run_detection = (frame_id == 1) or (not traj_locked) or rect_moving \
                or (frame_id % DETECT_INTERVAL == 0)
            if run_detection and TRACE_ENABLE:
                # 检测看门狗(超时/异常)在 find_rect_corners 内部处理: 超时或抛错时
                # 本次运行自动回退到 img.find_rects, 不会单帧卡死拖垮整个循环。
                corners = find_rect_corners(img, traj_corners_proc)
                if corners is not None:
                    missed_rect = 0
                    # --- 边框运动估计: 驱动自适应频率 + 角点速度(丢失外推用) ---
                    rcx, rcy = rect_center_of(corners)
                    if rect_last_center is not None and rect_prev_corners is not None:
                        dframes = max(1, frame_id - rect_last_frame)
                        ddx = rcx - rect_last_center[0]
                        ddy = rcy - rect_last_center[1]
                        disp = math.sqrt(ddx * ddx + ddy * ddy) / dframes
                        rect_speed = rect_speed * 0.7 + disp * 0.3
                        rect_moving = rect_speed > RECT_MOVE_PX_PER_FRAME
                        for i in range(4):
                            vx = (corners[i][0] - rect_prev_corners[i][0]) / dframes
                            vy = (corners[i][1] - rect_prev_corners[i][1]) / dframes
                            rect_corner_vel[i] = (
                                rect_corner_vel[i][0] * 0.6 + vx * 0.4,
                                rect_corner_vel[i][1] * 0.6 + vy * 0.4,
                            )
                    rect_last_center = (rcx, rcy)
                    rect_last_frame = frame_id
                    rect_prev_corners = [tuple(c) for c in corners]

                    if not traj_locked:
                        # ---- 首次锁定: 建立轨迹, 从参考点最近边出发 ----
                        traj = RectTrajectory(corners, TRACE_SAMPLE_SPACING)
                        traj_corners_proc = [tuple(c) for c in corners]
                        x0 = min(p[0] for p in corners)
                        y0 = min(p[1] for p in corners)
                        x1 = max(p[0] for p in corners)
                        y1 = max(p[1] for p in corners)
                        traj_rect_proc = (x0, y0, x1 - x0, y1 - y0)
                        traj_corner_frame = [scale_point_to_frame(p) for p in corners]
                        # start from the border point nearest to the reference
                        if TRACE_REFERENCE == "laser" and l_lock:
                            ref = (lx_s, ly_s)
                        elif recovering and last_good_laser_proc is not None:
                            ref = last_good_laser_proc
                        else:
                            ref = (PROC_W // 2, PROC_H // 2)
                        start_idx = traj.nearest_index(ref)
                        traj_arc = start_idx * TRACE_SAMPLE_SPACING
                        if not recovering:
                            traj_dir = TRACE_DIR
                        traj_dwell_until = 0
                        traj_dwell_arc = 0.0
                        traj_done = False
                        traj_start_frame = scale_point_to_frame(traj.point_at_index(start_idx))
                        traj_locked = True
                        if not TRACE_LOCK:
                            # keep rebuilding every detection (no lock)
                            traj_locked = False
                    else:
                        # ---- 已锁定: 角点 EMA 贴合真实边框(吸附快), 重建轨迹并保持
                        #      当前设点在真实边框上的位置(不跳变)。大跳变直接整框贴合。 ----
                        old_c = rect_center_of(traj_corners_proc)
                        old_w = abs(traj_corners_proc[1][0] - traj_corners_proc[0][0]) or 1
                        old_h = abs(traj_corners_proc[2][1] - traj_corners_proc[1][1]) or 1
                        jdx = rcx - old_c[0]
                        jdy = rcy - old_c[1]
                        jump = math.sqrt(jdx * jdx + jdy * jdy)
                        alpha = choose_snap_alpha(jump, max(old_w, old_h))
                        # 取重建前当前设点位置(用旧 traj), 重建后用最近点对齐, 保持连续不跳变
                        cur = traj.point_at_arc(traj_arc) if traj is not None else None
                        traj_corners_proc = snap_corners_to_target(traj_corners_proc, corners, alpha)
                        traj = RectTrajectory(traj_corners_proc, TRACE_SAMPLE_SPACING)
                        if cur is not None:
                            traj_arc = traj.nearest_index(cur) * TRACE_SAMPLE_SPACING
                        traj_corner_frame = [scale_point_to_frame(p) for p in traj_corners_proc]
                else:
                    # --- 检测丢失(高速运动模糊/瞬时遮挡): 保持上一框并外推,
                    #     识别"不掉", 轨迹继续沿保持的路径走(高速稳)。 ---
                    missed_rect += 1
                    if (missed_rect <= RECT_HOLD_FRAMES and traj_corners_proc is not None
                            and RECT_HOLD_EXTRAPOLATE):
                        for i in range(4):
                            traj_corners_proc[i] = (
                                traj_corners_proc[i][0] + rect_corner_vel[i][0],
                                traj_corners_proc[i][1] + rect_corner_vel[i][1],
                            )
                            traj_corner_frame[i] = scale_point_to_frame(traj_corners_proc[i])

            # --- advance the set-point along the perimeter (with corner handling) ---
            desired_frame = None
            progress = 0.0
            if traj_locked and traj is not None:
                now_ms = time.ticks_ms()
                dt = elapsed_seconds(now_ms, last_traj_ms)
                last_traj_ms = now_ms

                total = traj.total_len
                if total > 0 and not traj_done:
                    if traj_dwell_until > 0 and now_ms < traj_dwell_until:
                        # --- dwelling on a corner: freeze exactly on the vertex
                        #     so the velocity-controlled laser settles onto it ---
                        traj_arc = traj_dwell_arc
                    else:
                        if traj_dwell_until > 0:
                            # dwell just ended: nudge just past the corner so we
                            # do not immediately re-trigger the same corner
                            traj_arc = traj_dwell_arc + traj_dir * TRACE_SAMPLE_SPACING
                            traj_dwell_until = 0

                        # --- corner slowdown: ramp speed down near corners so the
                        #     gimbal has time to make the 90 deg turn (no cut) ---
                        d_corner = arc_dist_to_nearest_corner(traj_arc, traj.corner_arcs, total)
                        if d_corner >= CORNER_SLOWDOWN_ARC:
                            speed_scale = 1.0
                        else:
                            t = d_corner / CORNER_SLOWDOWN_ARC
                            speed_scale = CORNER_SLOW_FACTOR + (1.0 - CORNER_SLOW_FACTOR) * t
                        traj_arc += traj_dir * TRACE_SPEED * speed_scale * dt

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

                        # --- engage dwell when reaching a corner vertex ---
                        if CORNER_DWELL_MS > 0 and not traj_done:
                            ca = next_corner_ahead(traj_arc, traj_dir,
                                                  traj.corner_arcs, total,
                                                  CORNER_DWELL_ENTER)
                            if ca is not None:
                                traj_dwell_arc = ca
                                traj_dwell_until = now_ms + CORNER_DWELL_MS
                                traj_arc = ca            # snap exactly onto the corner

                desired_proc = traj.point_at_arc(traj_arc)
                if desired_proc is not None:
                    df = scale_point_to_frame(desired_proc)
                    if TRACE_SMOOTH:
                        d_corner = arc_dist_to_nearest_corner(traj_arc, traj.corner_arcs, total)
                        if d_corner < CORNER_SMOOTH_DISABLE_ARC:
                            # --- near a corner: use the RAW set-point so the EMA
                            #     does not round off the 90 deg turn ---
                            smooth_x, smooth_y = df
                            smooth_init = True
                            desired_frame = df
                        elif not smooth_init:
                            smooth_x, smooth_y = df
                            smooth_init = True
                            desired_frame = df
                        else:
                            d = max(abs(df[0] - smooth_x), abs(df[1] - smooth_y))
                            if d <= TRACE_NEAR_DISTANCE:
                                a = TRACE_ALPHA_NEAR
                            elif d <= TRACE_FAR_DISTANCE:
                                a = TRACE_ALPHA_MIDDLE
                            else:
                                a = TRACE_ALPHA_FAR
                            smooth_x += (df[0] - smooth_x) * a
                            smooth_y += (df[1] - smooth_y) * a
                            desired_frame = (int(round(smooth_x)), int(round(smooth_y)))
                    else:
                        desired_frame = df
                    progress = (traj_arc % total) / total if total > 0 else 0.0

            # --- laser-loss recovery give-up (safety net) ---
            # 回退超时仍未捕获到激光: 停止回退, 改发 valid=0, 让 STM32 超时停。
            if recovering and LASER_RECOVER_ENABLE:
                if time.ticks_diff(time.ticks_ms(), recover_start_ms) > LASER_RECOVER_GIVEUP_MS:
                    recovering = False
                    last_good_laser_proc = None
                    print("laser: recovery timeout -> give up (will safe-stop)")

            # --- decide validity / reference for the gimbal ---
            if desired_frame is not None:
                if TRACE_REFERENCE == "laser":
                    if l_lock:
                        laser_frame = scale_point_to_frame((lx_s, ly_s))
                        valid_out = True
                    else:
                        # 激光丢失: 若处于回退中, 用"冻结的最后激光位置"作参考
                        # 并继续发 valid=True, 让云台跟随反转的轨迹点回到视野;
                        # 否则(valid=0)由 STM32 超时安全停。
                        if recovering and last_good_laser_proc is not None:
                            laser_frame = scale_point_to_frame(last_good_laser_proc)
                            valid_out = True
                        else:
                            laser_frame = None
                            valid_out = False
                else:
                    laser_frame = scale_point_to_frame((lx_s, ly_s)) if l_lock else None
                    valid_out = True
            else:
                laser_frame = scale_point_to_frame((lx_s, ly_s)) if l_lock else None
                valid_out = False

            # --- send ---
            if frame_id % UART_SEND_INTERVAL == 0:
                send_trace(uart, desired_frame, laser_frame, valid_out, traj_done)

            # --- debug ---
            if DEBUG_PRINT_INTERVAL and frame_id % DEBUG_PRINT_INTERVAL == 0:
                if desired_frame is not None:
                    print("trace:", desired_frame, "laser:", laser_frame,
                          "prog:{:.2f}".format(progress), "done:", traj_done,
                          "fps:", clock.fps())
                else:
                    print("trace: none", "fps:", clock.fps())

            # --- OSD ---
            if OSD_REFRESH_INTERVAL and frame_id % OSD_REFRESH_INTERVAL == 0:
                draw_osd(osd_img, traj_corner_frame, traj_start_frame,
                         desired_frame, laser_frame, progress, traj_done,
                         clock.fps(), recovering)
                Display.show_image(osd_img, layer=Display.LAYER_OSD1)

            if frame_id == STATUS_RUNNING_FRAME:
                write_status("running", "frame={} fps={:.1f}".format(frame_id, clock.fps()))

            if GC_INTERVAL and frame_id % GC_INTERVAL == 0:
                gc.collect()

    except KeyboardInterrupt as error:
        write_status("stopped", repr(error))
        print("user stop:", error)
    except BaseException as error:
        error_text = repr(error)
        if "IDE interrupt" in error_text:
            write_status("stopped", error_text)
            print("ide stop:", error_text)
        else:
            write_status("error", error_text)
            print("exception:", error_text)
            raise
    finally:
        if camera_is_init:
            camera_deinit()


if __name__ == "__main__":
    main()
