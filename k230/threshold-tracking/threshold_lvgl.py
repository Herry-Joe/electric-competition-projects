# threshold_lvgl.py  v4.0 — 电赛专用双模式版 (优化 + 特别调试版)
# 庐山派 K230 + 官方可触摸屏 (ST7701 800x480)
#
# 模式一 TRACK: 色块追踪(find_blobs) + 脱机阈值调节(MASK编辑器) + USART2发坐标(比赛主功能)
# 模式二 SHAPE: 常见几何形状识别 + 描轮廓 + USART2发中心坐标(赛前准备)
#
# 本版新增(面向电赛现场"特别调试"):
#   * 配置持久化: 阈值/ROI 存 SD 卡, 断电不丢 (Save 按钮落盘 + 开机自动加载)
#   * 调试 HUD: 实时 FPS / 剩余内存 / 串口 TX·ERR 计数 / 最近发送坐标 / 阈值 —— DBG 按钮开关
#   * 串口链路自检: TEST 按钮发一帧自检帧(FLAG.bit3), 配合 PC 端 tools/uart_link_test.py 验证链路
#   * 启动/运行兜底: 摄像头初始化失败时屏上提示原因, 不再黑屏死机
#
# 接线(与 main.py 一致):
#   GPIO05(UART2_TXD) -> STM32 PB11
#   GPIO06(UART2_RXD) <- STM32 PB10
#
# 协议(复用 main.py): 坐标帧 0x3C 0x3B [XH][XL][YH][YL][FLAG][CRC] 0x01 0x01 (10字节)
#   FLAG bit0=valid  bit1=hit  bit2=shape_mode(0=追踪 1=形状)  bit3=test(自检帧)
#
# 固件兼容: 核心检测纯内置 API; cv2 形状轮廓为可选增强(无则用几何近似描边)
# LVGL 版本限制: 无 set_hidden, 用 set_pos 移出屏幕外; 无 checkbox, 用按钮颜色切换

from media.sensor import *
from media.display import *
from media.media import *
import time, os, sys, gc, math
import lvgl as lv
from machine import TOUCH, Pin, UART, FPIOA
import image
import uctypes

try:
    import ujson
except Exception:
    ujson = None

DISPLAY_WIDTH = ALIGN_UP(800, 16)
DISPLAY_HEIGHT = 480

# 脱机/调试切换: 连 IDE 联调时 True; 纯脱机比赛可改 False(不向 IDE 推流, 略省资源)
TO_IDE = True

# ==================== 全局状态 ====================
# 颜色阈值 (LAB): [Lmin, Lmax, Amin, Amax, Bmin, Bmax]
# 默认从全白开始(覆盖整个 LAB 空间), 进 MASK 后拖滑块逐步收紧到只命中目标
thresholds = [0, 100, -128, 127, -128, 127]
roi = None              # ROI 区域 (x,y,w_h), None = 全屏
mask_on = False         # MASK 阈值编辑器模式
track_on = False        # TRACK 追踪模式(发坐标到主控)
shape_on = False        # SHAPE 形状识别模式(描轮廓+发中心)

sensor = None
uart = None              # UART2 实例
cam_ok = False           # 摄像头是否就绪(失败则进错误态而非黑屏)
thr_panel = None         # 阈值滑块面板
val_labels = [None] * 6

# ---------- 调试 HUD 状态 ----------
debug_on = False
fps_smooth = 0.0
last_loop_ms = 0
uart_tx_cnt = 0
uart_err_cnt = 0
last_tx = (0, 0)
last_save_ms = 0
err_label = None

# ---------- 配置持久化 ----------
CONFIG_PATH = "/sdcard/threshold_config.json"   # 运行时校准落盘位置

# ---------- 形状多选 ----------
SHAPE_OPTIONS = [
    ("circle",    "Circle"),     # 圆
    ("rectangle", "Rectangle"),  # 矩形
    ("triangle",  "Triangle"),   # 三角形
    ("ellipse",   "Ellipse"),    # 椭圆
    ("bar",       "Bar"),        # 条形
]
shape_panel = None
shape_btns = []
shape_enabled = {}       # {key: bool}

# ---------- 滑块配置 ----------
SL_NAMES  = ["Lmin", "Lmax", "Amin", "Amax", "Bmin", "Bmax"]
SL_RANGES = [(0, 100), (0, 100), (-128, 127), (-128, 127), (-128, 127), (-128, 127)]
SL_INIT   = [0, 100, 0, 127, 0, 127]

# ---------- ROI 预设 ----------
ROI_PRESETS = [None, (0, 0, 400, 480), (400, 0, 400, 480), (200, 90, 400, 300)]

# ---------- 追踪时序(防抖+平滑, 参考 main.py) ----------
TRACK_LOCK_FRAMES = 3      # 连续检测到才算锁定
TRACK_LOST_FRAMES  = 8      # 连续丢失才算跟丢
TRACK_ALPHA         = 0.35   # 位置平滑系数(越小越稳)
track_locked = False
track_hit_cnt = 0
track_miss_cnt = 0
track_x = 0                 # 平滑后坐标(显示空间)
track_y = 0
last_track_ms = 0

# ---------- UART 发送时序 ----------
COORD_INTERVAL_MS = 25      # 40Hz 发送频率(与 main.py 一致)
last_send_ms = 0

# ---------- 可选 cv2(用于精确形状轮廓) ----------
try:
    import cv2
    HAVE_CV2 = True
except Exception:
    HAVE_CV2 = False


# ==================== 触摸输入 ====================
class touch_screen():
    def __init__(self):
        self.x = 0
        self.y = 0
        self.state = lv.INDEV_STATE.RELEASED
        self.indev_drv = lv.indev_create()
        self.indev_drv.set_type(lv.INDEV_TYPE.POINTER)
        self.indev_drv.set_read_cb(self.callback)
        self.touch = TOUCH(0)

    def callback(self, driver, data):
        tp = self.touch.read(1)
        if tp:
            p = tp[0]
            self.x, self.y = p.x, p.y
            if p.event in (2, 3):
                self.state = lv.INDEV_STATE.PRESSED
            else:
                self.state = lv.INDEV_STATE.RELEASED
        else:
            self.state = lv.INDEV_STATE.RELEASED
        data.point = lv.point_t({"x": self.x, "y": self.y})
        data.state = self.state


# ==================== LVGL 显示驱动 ====================
disp_img1 = None
disp_img2 = None

def disp_drv_flush_cb(disp_drv, area, color):
    global disp_img1, disp_img2
    buf = color.__dereference__()
    addr = uctypes.addressof(buf)
    if addr == disp_img1.virtaddr():
        Display.show_image(disp_img1, layer=Display.LAYER_OSD2)
    else:
        Display.show_image(disp_img2, layer=Display.LAYER_OSD2)
    disp_drv.flush_ready()


def lvgl_init():
    global disp_img1, disp_img2
    lv.init()
    disp_drv = lv.disp_create(DISPLAY_WIDTH, DISPLAY_HEIGHT)
    disp_drv.set_flush_cb(disp_drv_flush_cb)
    disp_drv.set_color_format(lv.COLOR_FORMAT.ARGB8888)
    disp_img1 = image.Image(DISPLAY_WIDTH, DISPLAY_HEIGHT, image.BGRA8888)
    disp_img2 = image.Image(DISPLAY_WIDTH, DISPLAY_HEIGHT, image.BGRA8888)
    disp_img1.clear()
    disp_img2.clear()
    disp_drv.set_draw_buffers(disp_img1.bytearray(), disp_img2.bytearray(), disp_img1.size(), lv.DISP_RENDER_MODE.FULL)
    touch_screen()


# ==================== 滑块回调 ====================
def _sl_event(idx, e):
    global thresholds, val_labels
    sl = lv.slider.__cast__(e.get_target())
    thresholds[idx] = sl.get_value()
    if val_labels[idx] is not None:
        val_labels[idx].set_text(SL_NAMES[idx] + ":" + str(thresholds[idx]))


def make_slider(parent, idx, x, y, w, rng, init):
    lbl = lv.label(parent)
    lbl.set_text(SL_NAMES[idx] + ":" + str(init))
    lbl.set_pos(x, y - 16)
    lbl.set_style_text_color(lv.color_hex(0xFFFFFF), lv.PART.MAIN)
    sl = lv.slider(parent)
    sl.set_size(w, 18)
    sl.set_pos(x, y)
    sl.set_range(rng[0], rng[1])
    sl.set_value(init, lv.ANIM.OFF)
    sl.add_event(lambda e, i=idx: _sl_event(i, e), lv.EVENT.VALUE_CHANGED, None)
    return lbl


# ==================== 按钮回调 ====================
def next_roi():
    try:
        idx = ROI_PRESETS.index(roi)
    except ValueError:
        idx = 0
    return ROI_PRESETS[(idx + 1) % len(ROI_PRESETS)]


def btn_roi_cb(e):
    global roi
    roi = next_roi()


def btn_mask_cb(e):
    global mask_on, thr_panel
    mask_on = not mask_on
    if thr_panel is not None:
        thr_panel.set_pos(5, 5) if mask_on else thr_panel.set_pos(-400, 0)


def btn_shape_cb(e):
    global shape_on, shape_panel
    shape_on = not shape_on
    if shape_panel is not None:
        cur_x = shape_panel.get_x()
        shape_panel.set_pos(480, 60) if cur_x < 0 else shape_panel.set_pos(-400, 0)


def btn_track_cb(e):
    global track_on
    track_on = not track_on


def btn_dbg_cb(e):
    """切换调试 HUD (FPS/内存/串口计数/阈值)"""
    global debug_on
    debug_on = not debug_on


def btn_test_cb(e):
    """串口链路自检: 发一帧特殊坐标(0x7FFF,0x7FFF) + FLAG.bit3, 供 PC/STM32 验证链路"""
    send_coord(0x7FFF, 0x7FFF, valid=True, hit=True, is_shape=False, test=True)


def btn_save_cb(e):
    """把当前阈值/ROI 落盘到 SD 卡, 开机自动加载"""
    global last_save_ms
    save_config()
    last_save_ms = time.ticks_ms()


# ==================== 形状选项按钮回调 ====================
def _shape_toggle(idx, e):
    global shape_enabled, shape_btns
    key = SHAPE_OPTIONS[idx][0]
    shape_enabled[key] = not shape_enabled.get(key, False)
    btn = shape_btns[idx]
    if shape_enabled[key]:
        btn.set_style_bg_color(lv.color_hex(0x00CC00), lv.PART.MAIN)
    else:
        btn.set_style_bg_color(lv.color_hex(0x666666), lv.PART.MAIN)


# ==================== UART2 通信 (复用 main.py 协议) ====================
def crc8(data):
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if (crc & 0x80) else (crc << 1) & 0xFF
    return crc & 0xFF


def send_coord(x, y, valid=True, hit=False, is_shape=False, test=False):
    """发送坐标帧到主控。协议: 0x3C 0x3B [XH][XL][YH][YL][FLAG][CRC] 0x01 0x01"""
    global uart, uart_tx_cnt, uart_err_cnt, last_tx
    if uart is None:
        uart_err_cnt += 1
        return
    try:
        buf = bytearray(10)
        buf[0] = 0x3C; buf[1] = 0x3B
        buf[2] = (x >> 8) & 0xFF; buf[3] = x & 0xFF
        buf[4] = (y >> 8) & 0xFF; buf[5] = y & 0xFF
        f = 0
        if valid: f |= 1
        if hit:   f |= 2
        if is_shape: f |= 4          # bit2 区分追踪/形状模式
        if test: f |= 8              # bit3 自检帧
        buf[6] = f
        buf[7] = crc8(bytes([buf[2], buf[3], buf[4], buf[5], buf[6]]))
        buf[8] = 1; buf[9] = 1
        uart.write(buf)
        uart_tx_cnt += 1
        if not test:
            last_tx = (x & 0xFFFF, y & 0xFFFF)
    except Exception:
        uart_err_cnt += 1


def init_uart():
    """初始化 UART2 (GPIO05 TX / GPIO06 RX, 115200)。失败仅置 None(不发数据, 不阻塞)。"""
    global uart
    try:
        fpioa = FPIOA()
        fpioa.set_function(5, fpioa.UART2_TXD)
        fpioa.set_function(6, fpioa.UART2_RXD)
        uart = UART(UART.UART2, 115200, bits=UART.EIGHTBITS,
                    parity=UART.PARITY_NONE, stop=UART.STOPBITS_ONE)
        print("[UART2] OK @115200")
        return True
    except Exception as e:
        print("[UART2] FAIL:", e)
        uart = None
        return False


# ==================== 配置持久化 (SD 卡) ====================
def load_config():
    """开机自动加载上次校准的阈值/ROI。文件不存在则用出厂默认。"""
    global thresholds, roi, SL_INIT
    try:
        if ujson is None:
            raise Exception("ujson 不可用")
        with open(CONFIG_PATH) as f:
            cfg = ujson.load(f)
        t = cfg.get("thresholds")
        if isinstance(t, (list, tuple)) and len(t) == 6:
            for i in range(6):
                thresholds[i] = int(t[i])
        r = cfg.get("roi", None)
        if r is None:
            roi = None
        elif isinstance(r, (list, tuple)) and len(r) == 4:
            roi = (int(r[0]), int(r[1]), int(r[2]), int(r[3]))
        SL_INIT = list(thresholds)   # 让滑块初始位置反映已加载的阈值
        print("[CFG] loaded", thresholds, roi)
    except Exception as e:
        print("[CFG] 使用出厂默认(无已保存配置):", e)


def save_config():
    """把当前阈值/ROI 写入 SD 卡。"""
    try:
        if ujson is None:
            raise Exception("ujson 不可用")
        try:
            os.mkdir("/sdcard")
        except Exception:
            pass
        with open(CONFIG_PATH, "w") as f:
            ujson.dump({"thresholds": list(thresholds), "roi": roi}, f)
        print("[CFG] saved", thresholds, roi)
    except Exception as e:
        print("[CFG] 保存失败:", e)


# ==================== 错误兜底显示 ====================
def show_fatal(msg):
    """摄像头等致命错误时, 在屏上(OSD2 覆盖层)显示原因, 避免黑屏。"""
    global err_label
    if err_label is not None:
        err_label.set_text(msg)
        err_label.set_pos(40, 180)


# ==================== 几何形状分类 ====================
def classify_shape(b):
    w, h = b[2], b[3]
    ar = (w / h) if h != 0 else 0
    rnd = b.roundness() if hasattr(b, "roundness") else 0
    if rnd > 0.85:
        return "circle"
    if rnd > 0.65 and (ar > 1.5 or ar < 0.67):
        return "ellipse"
    if 0.55 <= ar <= 1.8 and rnd <= 0.65:
        return "rectangle"
    if 0.45 <= rnd <= 0.7 and 0.7 <= ar <= 1.4:
        return "triangle"
    if ar > 1.8 or ar < 0.55:
        return "bar"
    return "rectangle"


# ==================== 形状轮廓绘制 (无 cv2 时用几何近似) ====================
def draw_shape_outline(img, b, shape_key):
    """根据分类结果在 img 上描绘形状轮廓。
    有 cv2 时用 approxPolyDP 精确描边; 无 cv2 时用几何近似。"""
    x, y, w, h = int(b[0]), int(b[1]), int(b[2]), int(b[3])
    cx, cy = int(b[5]), int(b[6])

    if shape_key == "circle":
        r = max(w, h) // 2
        img.draw_circle(cx, cy, r, color=(255, 0, 255), thickness=2)

    elif shape_key == "ellipse":
        # K230 无 draw_ellipse, 用矩形包围框 + 内切圆近似
        img.draw_rectangle(x, y, w, h, color=(255, 0, 255), thickness=2)
        r = min(w, h) // 2
        img.draw_circle(cx, cy, r, color=(255, 0, 255), thickness=1)

    elif shape_key == "rectangle":
        img.draw_rectangle(x, y, w, h, color=(255, 165, 0), thickness=2)

    elif shape_key == "triangle":
        # 近似三角: blob 包围框的顶部中点 + 左下角 + 右下角
        tx_top = x + w // 2
        tx_left = x + 3
        tx_right = x + w - 3
        ty_top = y + 3
        ty_bot = y + h - 3
        img.draw_line(tx_top, ty_top, tx_left, ty_bot, color=(0, 255, 255), thickness=2)
        img.draw_line(tx_left, ty_bot, tx_right, ty_bot, color=(0, 255, 255), thickness=2)
        img.draw_line(tx_right, ty_bot, tx_top, ty_top, color=(0, 255, 255), thickness=2)

    elif shape_key == "bar":
        if w > h:   # 横条
            img.draw_rectangle(x, y, w, h, color=(0, 255, 255), thickness=2)
        else:       # 竖条
            img.draw_rectangle(x, y, w, h, color=(0, 255, 255), thickness=2)

    else:
        img.draw_rectangle(x, y, w, h, color=(255, 0, 255), thickness=2)


# ==================== cv2 精确形状轮廓 (可选增强) ====================
def detect_shapes_cv(img):
    """用 cv2 轮廓逼近精确识别并返回 [(key, cx, cy, contour_pts), ...]。
    返回 None 表示 cv2 不可用或失败。"""
    if not HAVE_CV2:
        return None
    try:
        arr = img.to_numpy_ref()
        if arr is None:
            return None
        # 转 BGR (cv2 需要)
        if arr.dtype == "uint8" and arr.ndim == 3 and arr.shape[2] == 3:
            bgr = cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
        elif arr.dtype == "uint16" and arr.ndim == 2:
            bgr = cv2.cvtColor(arr, cv2.COLOR_RGB5652BGR)
        else:
            return None

        lab = cv2.cvtColor(bgr, cv2.COLOR_BGR2LAB)
        Lmin, Lmax, Amin, Amax, Bmin, Bmax = thresholds
        lower = (int(Lmin * 255 / 100), Amin + 128, Bmin + 128)
        upper = (int(Lmax * 255 / 100), Amax + 128, Bmax + 128)
        mask = cv2.inRange(lab, lower, upper)

        cs = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnts = cs[0] if len(cs) == 2 else cs[1]
        results = []
        for c in cnts:
            area = cv2.contourArea(c)
            if area < 400:
                continue
            peri = cv2.arcLength(c, True)
            approx = cv2.approxPolyDP(c, 0.04 * peri, True)
            verts = len(approx)
            M = cv2.moments(c)
            if M['m00'] == 0:
                continue
            cx_f = M['m10'] / M['m00']
            cy_f = M['m01'] / M['m00']

            if verts >= 8:
                circ = 4 * math.pi * area / (peri * peri + 1e-9)
                key = "circle" if circ > 0.75 else "ellipse"
            elif verts == 4:
                key = "rectangle"
            elif verts == 3:
                key = "triangle"
            else:
                key = "bar"

            if shape_enabled.get(key, False):
                results.append((key, int(cx_f), int(cy_f), approx))
        return results
    except Exception:
        return None


# ==================== 色块检测封装 ====================
def run_find_blobs(img):
    kw = dict(area_threshold=400, pixels_threshold=150, merge=True,
              x_stride=2, y_stride=2)           # 降采样加速(4倍像素跳过)
    if roi is not None:
        kw["roi"] = roi
    return img.find_blobs([tuple(thresholds)], **kw)


# ==================== 形状是否有任一启用 ====================
def any_shape_enabled():
    return any(shape_enabled.get(k, False) for k, _ in SHAPE_OPTIONS)


# ==================== LVGL 界面搭建 ====================
def user_gui_init():
    global val_labels, thr_panel, shape_panel, shape_btns, shape_enabled, err_label
    res_path = "/sdcard/examples/15-LVGL/data/"
    try:
        font = lv.font_load("A:" + res_path + "font/lv_font_simsun_16_cjk.fnt")
    except Exception:
        font = None

    scr = lv.scr_act()
    lv.scr_act().set_style_bg_opa(lv.OPA.TRANSP, lv.PART.MAIN)

    # ===== 致命错误提示层(默认移出屏幕, 出错时移入) =====
    err_label = lv.label(scr)
    err_label.set_text("")
    err_label.set_pos(-600, -600)
    err_label.set_style_text_color(lv.color_hex(0xFF4444), lv.PART.MAIN)
    try:
        err_label.set_style_text_line_space(6, lv.PART.MAIN)
    except Exception:
        pass

    # ===== 左侧: 阈值滑块面板 (MASK 模式显示, 双列布局) =====
    thr_panel = lv.obj(lv.layer_sys())
    thr_panel.set_size(275, 240)
    thr_panel.set_pos(5, 5)
    thr_panel.set_style_bg_color(lv.color_hex(0x000000), lv.PART.MAIN)
    thr_panel.set_style_bg_opa(150, lv.PART.MAIN)
    thr_panel.set_style_border_width(0, lv.PART.MAIN)
    thr_panel.set_pos(-400, 0)

    title = lv.label(thr_panel)
    title.set_text("LAB Threshold")
    title.set_pos(10, 6)
    title.set_style_text_color(lv.color_hex(0xFFFFFF), lv.PART.MAIN)
    if font:
        title.set_style_text_font(font, 0)

    left_indices  = [0, 2, 4]
    right_indices = [1, 3, 5]
    col_w = 120
    for row, (li, ri) in enumerate(zip(left_indices, right_indices)):
        y = 40 + row * 63
        lbl_l = make_slider(thr_panel, li, 10, y, col_w, SL_RANGES[li], SL_INIT[li])
        val_labels[li] = lbl_l
        lbl_r = make_slider(thr_panel, ri, 145, y, col_w, SL_RANGES[ri], SL_INIT[ri])
        val_labels[ri] = lbl_r
        if font:
            val_labels[li].set_style_text_font(font, 0)
            val_labels[ri].set_style_text_font(font, 0)

    # ===== 形状选择面板 =====
    shape_panel = lv.obj(lv.layer_sys())
    shape_panel.set_size(200, 290)
    shape_panel.set_pos(-400, 0)
    shape_panel.set_style_bg_color(lv.color_hex(0x000000), lv.PART.MAIN)
    shape_panel.set_style_bg_opa(160, lv.PART.MAIN)
    shape_panel.set_style_border_width(2, lv.PART.MAIN)
    shape_panel.set_style_border_color(lv.color_hex(0x3366FF), lv.PART.MAIN)

    sptitle = lv.label(shape_panel)
    sptitle.set_text("Shape Select")
    sptitle.set_pos(15, 8)
    sptitle.set_style_text_color(lv.color_hex(0xFFFFFF), lv.PART.MAIN)
    if font:
        sptitle.set_style_text_font(font, 0)

    # CV2 状态提示
    cv2_lbl = lv.label(shape_panel)
    cv2_lbl.set_text("cv2:%s" % ("Y" if HAVE_CV2 else "N"))
    cv2_lbl.set_pos(15, 28)
    cv2_lbl.set_style_text_color(lv.color_hex(0xAAAAAA), lv.PART.MAIN)

    shape_btns = []
    for i, (key, name) in enumerate(SHAPE_OPTIONS):
        shape_enabled[key] = False
        y = 50 + i * 46
        b = lv.btn(shape_panel)
        b.set_size(170, 38)
        b.set_pos(12, y)
        b.set_style_radius(12, lv.PART.MAIN)
        b.set_style_bg_color(lv.color_hex(0x666666), lv.PART.MAIN)
        b.set_style_bg_opa(255, lv.PART.MAIN)
        b.add_event(lambda e, idx=i: _shape_toggle(idx, e), lv.EVENT.CLICKED, None)
        t = lv.label(b)
        t.set_text(name)
        if font:
            t.set_style_text_font(font, 0)
        shape_btns.append(b)

    # ===== 右侧: 功能按钮栏 =====
    sidebar = lv.obj(lv.layer_sys())
    sidebar.set_size(95, 470)
    sidebar.set_pos(700, 5)
    sidebar.set_style_bg_color(lv.color_hex(0x000000), lv.PART.MAIN)
    sidebar.set_style_bg_opa(130, lv.PART.MAIN)
    sidebar.set_style_border_width(0, lv.PART.MAIN)

    def add_btn(y, text, cb, color=None):
        b = lv.btn(sidebar)
        b.set_size(85, 42)
        b.set_pos(5, y)
        b.set_style_radius(20, lv.PART.MAIN)
        b.set_style_bg_color(color or lv.color_hex(0x3366FF), lv.PART.MAIN)
        b.set_style_bg_opa(255, lv.PART.MAIN)
        b.add_event(cb, lv.EVENT.CLICKED, None)
        t = lv.label(b)
        t.set_text(text)
        if font:
            t.set_style_text_font(font, 0)
        return b

    add_btn(15,  "ROI",   btn_roi_cb)
    add_btn(62,  "Shape", btn_shape_cb)
    add_btn(109, "MASK",  btn_mask_cb)
    add_btn(156, "Track", btn_track_cb, lv.color_hex(0xCC0000))   # 红色强调
    add_btn(203, "DBG",   btn_dbg_cb, lv.color_hex(0x00AAAA))     # 调试 HUD
    add_btn(250, "TEST",  btn_test_cb, lv.color_hex(0xAA00AA))    # 串口自检
    add_btn(297, "Save",  btn_save_cb)

    lv.scr_load(scr)


# ==================== 每帧检测 + 绘制 + 通信 ====================
def draw_frame():
    global sensor, track_locked, track_hit_cnt, track_miss_cnt, track_x, track_y, last_track_ms
    global last_send_ms

    # 摄像头未就绪: 直接返回(LVGL 错误层已在屏上显示)
    if sensor is None:
        return

    img = sensor.snapshot(chn=CAM_CHN_ID_0)
    if img is None:
        return

    now = time.ticks_ms()

    # ===== MASK 阈值编辑器模式 (最高优先级) =====
    if mask_on:
        try:
            mask = img.binary([tuple(thresholds)])
            if roi:
                mask.draw_rectangle(roi, color=(255, 255, 0), thickness=2)
            txt = "L[%d~%d] A[%d~%d] B[%d~%d]" % (
                thresholds[0], thresholds[1], thresholds[2], thresholds[3],
                thresholds[4], thresholds[5])
            try:
                mask.draw_string_advanced(280, 10, 20, txt, color=(255, 255, 255))
            except Exception:
                pass
            Display.show_image(mask, 0, 0, Display.LAYER_OSD1)
            return
        except Exception as e:
            sys.print_exception(e)

    # ===== ROI 框 =====
    if roi:
        img.draw_rectangle(roi, color=(255, 255, 0), thickness=2)

    n = 0
    has_shapes = any_shape_enabled()

    try:
        blobs = run_find_blobs(img)

        # ========== SHAPE 形状识别模式 ==========
        if shape_on and has_shapes:
            # 优先走 cv2 精确路径
            cv_results = detect_shapes_cv(img) if HAVE_CV2 else None
            if cv_results:
                for (key, scx, scy, pts) in cv_results:
                    n += 1
                    # 画轮廓点连线
                    for i in range(len(pts)):
                        p1 = tuple(pts[i][0])
                        p2 = tuple(pts[(i + 1) % len(pts)][0])
                        img.draw_line(int(p1[0]), int(p1[1]),
                                      int(p2[0]), int(p2[1]),
                                      color=(255, 0, 255), thickness=2)
                    img.draw_cross(scx, scy, color=(255, 0, 255), thickness=2)
                    img.draw_string_advanced(scx, scy - 24, 20, key,
                                            color=(255, 0, 255))
                    # 定时发送中心坐标
                    if time.ticks_diff(now, last_send_ms) >= COORD_INTERVAL_MS:
                        send_coord(scx, scy, valid=True, is_shape=True)
                        last_send_ms = now
            else:
                # 回退: find_blobs + 几何近似描轮廓
                for b in blobs:
                    n += 1
                    key = classify_shape(b)
                    if shape_enabled.get(key, False):
                        draw_shape_outline(img, b, key)
                        bx, by = int(b[5]), int(b[6])
                        img.draw_cross(bx, by, color=(255, 0, 255), thickness=2)
                        img.draw_string_advanced(bx, by - 24, 20, key,
                                                color=(255, 0, 255))
                        if time.ticks_diff(now, last_send_ms) >= COORD_INTERVAL_MS:
                            send_coord(bx, by, valid=True, is_shape=True)
                            last_send_ms = now

        # ========== TRACK 色块追踪模式 (默认) ==========
        else:
            best_blob = None
            best_px = 0
            for b in blobs:
                n += 1
                px = b.pixels() if hasattr(b, 'pixels') else (b[2] * b[3])
                if px > best_px:
                    best_px = px
                    best_blob = b

            if best_blob is not None:
                raw_cx = int(best_blob[5])
                raw_cy = int(best_blob[6])

                # ---- 平滑跟踪 (参考 main.py 激光时序) ----
                track_hit_cnt = min(track_hit_cnt + 1, 32)
                track_miss_cnt = 0

                if not track_locked and track_hit_cnt >= TRACK_LOCK_FRAMES:
                    track_locked = True
                    track_x, track_y = raw_cx, raw_cy
                elif track_locked:
                    # 最大跳跃限制
                    max_jump = 60
                    dx = raw_cx - track_x
                    dy = raw_cy - track_y
                    d2 = dx * dx + dy * dy
                    if d2 > max_jump * max_jump and d2 > 0:
                        scale = max_jump / math.sqrt(d2)
                        raw_cx = int(track_x + dx * scale)
                        raw_cy = int(track_y + dy * scale)
                    # 指数平滑
                    track_x = track_x * (1 - TRACK_ALPHA) + raw_cx * TRACK_ALPHA
                    track_y = track_y * (1 - TRACK_ALPHA) + raw_cy * TRACK_ALPHA

                last_track_ms = now
            else:
                track_miss_cnt = min(track_miss_cnt + 1, 32)
                track_hit_cnt = 0
                if track_locked and track_miss_cnt >= TRACK_LOST_FRAMES:
                    track_locked = False

            # ---- 绘制所有检测框 ----
            for b in blobs:
                bx, by, bw, bh = int(b[0]), int(b[1]), int(b[2]), int(b[3])
                bcx, bcy = int(b[5]), int(b[6])
                img.draw_rectangle(bx, by, bw, bh,
                                  color=(0, 255, 0), thickness=3)
                img.draw_cross(bcx, bcy, color=(0, 255, 0), thickness=2)

                if track_on and track_locked:
                    # 锁定目标用红色高亮
                    if abs(bcx - track_x) < 15 and abs(bcy - track_y) < 15:
                        img.draw_rectangle(bx, by, bw, bh,
                                          color=(255, 0, 0), thickness=3)
                        img.draw_string_advanced(bx, by - 30, 22,
                                                "LOCK", color=(255, 0, 0))

            # ---- 追踪状态指示 ----
            if track_on:
                if track_locked:
                    tx, ty = int(track_x), int(track_y)
                    img.draw_cross(tx, ty, sz=18, color=(255, 0, 0))
                    img.draw_string_advanced(tx + 20, ty - 10, 18,
                                            "(%d,%d)" % (tx, ty),
                                            color=(255, 0, 0))

                    # 定时发送坐标
                    if time.ticks_diff(now, last_send_ms) >= COORD_INTERVAL_MS:
                        send_coord(tx, ty, valid=True, hit=False, is_shape=False)
                        last_send_ms = now
                else:
                    try:
                        img.draw_string_advanced(280, 10, 22,
                                                "SEARCHING...", color=(255, 100, 100))
                    except Exception:
                        pass

            # 默认标签(非追踪模式或未锁定时显示坐标)
            if not (track_on and track_locked):
                for b in blobs[:5]:   # 最多标 5 个避免遮挡
                    bx, by = int(b[5]), int(b[6])
                    img.draw_string_advanced(bx, by - 24, 20,
                                            "(%d,%d)" % (bx, by),
                                            color=(0, 255, 0))

    except Exception as e:
        sys.print_exception(e)

    # ===== 状态栏 =====
    mode_str = "SHAPE" if (shape_on and any_shape_enabled()) else \
               ("TRACK" if track_on else "VIEW")
    lock_str = "LK" if (track_on and track_locked) else \
              ("--" if track_on else "")
    status = "%s%s %s blobs:%d ROI:%s" % (
        mode_str, (" D" if debug_on else ""), lock_str, n, "on" if roi else "off")
    try:
        img.draw_string_advanced(270, 8, 20, status, color=(255, 255, 255))
    except Exception:
        pass

    # ===== 调试 HUD(底部, DBG 开关) =====
    if debug_on:
        try:
            mem_kb = gc.mem_free() // 1024
            uart_str = "OK" if uart is not None else "NO"
            lines = [
                "FPS:%.1f  MEM:%dkB" % (fps_smooth, mem_kb),
                "TX:%d  ERR:%d  UART:%s" % (uart_tx_cnt, uart_err_cnt, uart_str),
                "LST:(%d,%d)" % (last_tx[0], last_tx[1]),
                "thr L[%d~%d] A[%d~%d] B[%d~%d]" % (
                    thresholds[0], thresholds[1], thresholds[2], thresholds[3],
                    thresholds[4], thresholds[5]),
            ]
            if time.ticks_diff(now, last_save_ms) < 1500:
                lines.append("CFG SAVED OK")
            ys = [398, 414, 430, 446, 462]
            for i, line in enumerate(lines):
                if i < len(ys):
                    img.draw_string_advanced(8, ys[i], 16, line, color=(0, 255, 255))
        except Exception:
            pass

    Display.show_image(img, 0, 0, Display.LAYER_OSD1)


# ==================== 媒体初始化(拆分为显示与摄像头, 便于兜底) ====================
def display_init():
    Display.init(Display.ST7701, width=DISPLAY_WIDTH, height=DISPLAY_HEIGHT,
                 to_ide=TO_IDE, osd_num=2)
    print("[DISP] OK ST7701 %dx%d to_ide=%s" % (DISPLAY_WIDTH, DISPLAY_HEIGHT, TO_IDE))


def sensor_init():
    """初始化摄像头。失败返回 False(由 show_fatal 屏上提示), 不阻塞整程序。"""
    global sensor
    try:
        sensor = Sensor(fps=30)
        sensor.reset()
        sensor.set_framesize(w=DISPLAY_WIDTH, h=DISPLAY_HEIGHT, chn=CAM_CHN_ID_0)
        sensor.set_pixformat(Sensor.RGB565, chn=CAM_CHN_ID_0)
        sensor.run()
        print("[CAM] OK %dx%d RGB565" % (DISPLAY_WIDTH, DISPLAY_HEIGHT))
        return True
    except Exception as e:
        sys.print_exception(e)
        return False


def media_init():
    display_init()
    sensor_init()


def media_deinit():
    global sensor
    os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
    try:
        if sensor is not None:
            sensor.stop()
    except Exception:
        pass
    try:
        Display.deinit()
    except Exception:
        pass
    time.sleep_ms(50)


# ==================== 入口 ====================
init_uart()                      # 先起串口, 失败仅置 None(不发数据)
lvgl_init()
user_gui_init()
load_config()                    # 开机自动加载上次校准(若 SD 卡有配置)
cam_ok = sensor_init()           # 摄像头初始化(失败在屏上提示, 不死机)

print("[SYS] MODES: VIEW(default) | Track(UART2) | Shape(outline+UART2) | Mask(editor)")
print("[SYS] cv2=%s | UART=%s | cam=%s" % (
    "Y" if HAVE_CV2 else "N", "ON" if uart else "NO", "OK" if cam_ok else "FAIL"))

try:
    if not cam_ok:
        # 摄像头未就绪: 保持 LVGL 存活以显示错误, 不进相机循环
        show_fatal("CAMERA INIT FAILED\ncheck FPC / power / lens\n[UART=%s]" %
                   ("ON" if uart else "NO"))
        while True:
            lv.task_handler()
            time.sleep_ms(20)

    frame_cnt = 0
    last_loop_ms = time.ticks_ms()
    while True:
        draw_frame()
        lv.task_handler()
        # FPS 估算(平滑)
        now = time.ticks_ms()
        dt = time.ticks_diff(now, last_loop_ms)
        last_loop_ms = now
        if dt > 0:
            inst = 1000.0 / dt
            fps_smooth = fps_smooth * 0.9 + inst * 0.1
        time.sleep_ms(1)
        # 周期性 GC: 通信模式每 50 帧, 普通模式每 15 帧(该固件无 gc.collect_interval)
        frame_cnt = (frame_cnt + 1) % 50
        if frame_cnt == 0:
            gc.collect()
        elif not (track_on or (shape_on and any_shape_enabled())) and frame_cnt % 15 == 0:
            gc.collect()
except BaseException as e:
    sys.print_exception(e)
    # 异常退出时通知主控
    if uart:
        try:
            send_coord(0, 0, valid=False)
        except Exception:
            pass
    time.sleep_ms(100)

lv.deinit()
media_deinit()
gc.collect()
print("[SYS] Done.")
