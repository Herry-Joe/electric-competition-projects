from media.display import *
from media.media import *
import machine
from machine import TOUCH
from machine import Pin
from machine import FPIOA
from machine import RTC
from machine import PWM
import os
import gc
import time
import lvgl as lv
import math

DISPLAY_WIDTH = 640
DISPLAY_HEIGHT = 480

RESOURCES_PATH = "/sdcard/CanMV Sample/"

class LCD():
    def __init__(self, width=640, height=480, to_ide=False, fpioa=None, bl_pinx=5, bl_valid=1):
        self.display = Display()
        self.display.init(Display.ST7701, width, height, to_ide=to_ide, quality=100)
        MediaManager.init()

        fpioa.set_function(bl_pinx, fpioa.GPIO0 + bl_pinx)
        pull = Pin.PULL_UP if bl_valid == 0 else Pin.PULL_DOWN
        self.bl = Pin(bl_pinx, Pin.OUT, pull=pull, drive=7)
        self.bl_valid = bl_valid
        self.on()

    def __del__(self):
        del self.bl
        os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
        time.sleep_ms(50)
        self.display.deinit()
        MediaManager.deinit()

    def on(self):
        self.bl.value(self.bl_valid)

    def off(self):
        self.bl.value(1 - self.bl_valid)

    def lvgl_flush_cb(self, disp, area, px_map):
        if disp.flush_is_last() == True:
            if self.draw_buf_1.virtaddr() == uctypes.addressof(px_map.__dereference__()):
                self.display.show_image(self.draw_buf_1)
            else:
                self.display.show_image(self.draw_buf_2)

        disp.flush_ready()

    def lvgl_init(self, width, height):
        self.draw_buf_1 = image.Image(width, height, image.BGRA8888)
        self.draw_buf_2 = image.Image(width, height, image.BGRA8888)

        self.disp = lv.disp_create(width, height)
        self.disp.set_flush_cb(self.lvgl_flush_cb)
        self.disp.set_draw_buffers(self.draw_buf_1.bytearray(), self.draw_buf_2.bytearray(), self.draw_buf_1.size(), lv.DISP_RENDER_MODE.DIRECT)

    def lvgl_deinit(self):
        del self.disp
        del self.draw_buf_1
        del self.draw_buf_2

class Touch():
    def __init__(self):
        self.touch = TOUCH(0)

    def __del__(self):
        del self.touch

    def lvgl_read_cb(self, indev, data):
        x, y, state = 0, 0, lv.INDEV_STATE.RELEASED
        tp = self.touch.read(1)
        if len(tp):
            x, y, event = tp[0].x, tp[0].y, tp[0].event
            if event == TOUCH.EVENT_DOWN or event == TOUCH.EVENT_MOVE:
                state = lv.INDEV_STATE.PRESSED
        data.point = lv.point_t({'x': x, 'y': y})
        data.state = state

    def lvgl_init(self):
        self.indev = lv.indev_create()
        self.indev.set_type(lv.INDEV_TYPE.POINTER)
        self.indev.set_read_cb(self.lvgl_read_cb)

    def lvgl_deinit(self):
        del self.indev

def lvgl_init(lcd, touch):
    lv.init()
    lcd.lvgl_init(DISPLAY_WIDTH, DISPLAY_HEIGHT)
    touch.lvgl_init()

def lvgl_deinit(lcd, touch):
    lcd.lvgl_deinit()
    touch.lvgl_deinit()
    lv.deinit()


class BookApp():
    def __init__(self, screen):
        self.screen = screen
        self.chapters = []
        self.page = 0
        self.view = 'list'
        self.current_ch = 0
        self.load_index()
        self.font = lv.font_load("A:" + RESOURCES_PATH + "Fonts/lv_font_simsun_16_cjk.fnt")
        self.show_list()

    def load_index(self):
        try:
            f = open(RESOURCES_PATH + "APP/Books/index.txt", "rb")
            raw = f.read()
            f.close()
            text = raw.decode("utf-8")
            for line in text.strip().split("\n"):
                parts = line.split("|")
                if len(parts) == 3:
                    self.chapters.append((parts[0], int(parts[1]), int(parts[2])))
        except:
            pass

    def show_list(self):
        self.screen.clean()
        self.view = 'list'
        n = len(self.chapters)
        pp = 10
        tp = (n + pp - 1) // pp
        if tp == 0:
            tp = 1

        hdr = lv.label(self.screen)
        hdr.set_text("Page " + str(self.page + 1) + "/" + str(tp) + "  " + str(n) + "ch")
        hdr.set_style_text_font(self.font, 0)
        hdr.set_style_text_color(lv.color_hex(0x8B4513), 0)
        hdr.align(lv.ALIGN.TOP_MID, 0, 5)

        s = self.page * pp
        e = s + pp
        if e > n:
            e = n

        txt = ""
        for i in range(s, e):
            txt += str(i + 1) + ". " + self.chapters[i][0] + "\n"

        lbl = lv.label(self.screen)
        lbl.set_width(lv.pct(90))
        lbl.set_text(txt)
        lbl.set_style_text_font(self.font, 0)
        lbl.set_style_text_color(lv.color_hex(0x333333), 0)
        lbl.align(lv.ALIGN.TOP_MID, 0, 30)

        if self.page > 0:
            pb = lv.btn(self.screen)
            pb.set_size(100, 36)
            pb.align(lv.ALIGN.BOTTOM_LEFT, 20, -5)
            pb.set_style_bg_color(lv.color_hex(0xD2691E), 0)
            pb.set_style_radius(6, 0)
            pb.set_style_shadow_width(0, 0)
            pb.add_event(lambda e: self.prev_page(), lv.EVENT.CLICKED, None)
            pl = lv.label(pb)
            pl.set_text("< Prev")
            pl.set_style_text_font(self.font, 0)
            pl.center()

        if self.page < tp - 1:
            nb = lv.btn(self.screen)
            nb.set_size(100, 36)
            nb.align(lv.ALIGN.BOTTOM_RIGHT, -20, -5)
            nb.set_style_bg_color(lv.color_hex(0xD2691E), 0)
            nb.set_style_radius(6, 0)
            nb.set_style_shadow_width(0, 0)
            nb.add_event(lambda e: self.next_page(), lv.EVENT.CLICKED, None)
            nl = lv.label(nb)
            nl.set_text("Next >")
            nl.set_style_text_font(self.font, 0)
            nl.center()

        # tap handler
        self.screen.add_event(self.on_list_tap, lv.EVENT.CLICKED, None)

    def on_list_tap(self, event):
        indev = lv.indev_get_act()
        if indev is None:
            return
        point = lv.point_t()
        indev.get_point(point)
        y = point.y

        pp = 10
        s = self.page * pp
        e = s + pp
        if e > len(self.chapters):
            e = len(self.chapters)

        mid_start = 30
        mid_end = 440
        cnt = e - s
        if cnt <= 0:
            return
        slot_h = (mid_end - mid_start) / cnt
        idx = int((y - mid_start) / slot_h) + s
        if idx >= s and idx < e:
            self.open_chapter(idx)

    def prev_page(self):
        if self.page > 0:
            self.page -= 1
            self.show_list()

    def next_page(self):
        pp = 10
        tp = (len(self.chapters) + pp - 1) // pp
        if self.page < tp - 1:
            self.page += 1
            self.show_list()

    def open_chapter(self, idx):
        self.screen.clean()
        self.view = 'reader'
        self.current_ch = idx

        ch = self.chapters[idx]
        txt = "Read error"
        try:
            f = open(RESOURCES_PATH + "APP/Books/jianlai.txt", "rb")
            f.seek(ch[1])
            raw = f.read(ch[2])
            f.close()
            text = raw.decode("utf-8", "ignore").strip()
            nl = text.find("\n")
            if nl > 0:
                text = text[nl + 1:].strip()
            if len(text) > 4000:
                text = text[:4000] + "\n\n..."
            txt = text
        except:
            pass

        hdr = lv.label(self.screen)
        hdr.set_text(str(idx + 1) + "/" + str(len(self.chapters)))
        hdr.set_style_text_font(self.font, 0)
        hdr.set_style_text_color(lv.color_hex(0x8B4513), 0)
        hdr.align(lv.ALIGN.TOP_MID, 0, 3)

        bb = lv.btn(self.screen)
        bb.set_size(80, 30)
        bb.align(lv.ALIGN.TOP_LEFT, 10, 3)
        bb.set_style_bg_color(lv.color_hex(0x8B4513), 0)
        bb.set_style_radius(6, 0)
        bb.set_style_shadow_width(0, 0)
        bb.add_event(lambda e: self.show_list(), lv.EVENT.CLICKED, None)
        bl = lv.label(bb)
        bl.set_text("< List")
        bl.set_style_text_font(self.font, 0)
        bl.center()

        lbl = lv.label(self.screen)
        lbl.set_width(lv.pct(90))
        lbl.set_text(txt)
        lbl.set_style_text_font(self.font, 0)
        lbl.set_style_text_color(lv.color_hex(0x333333), 0)
        lbl.align(lv.ALIGN.TOP_MID, 0, 20)

        if idx > 0:
            pb = lv.btn(self.screen)
            pb.set_size(100, 30)
            pb.align(lv.ALIGN.BOTTOM_LEFT, 20, -5)
            pb.set_style_bg_color(lv.color_hex(0xD2691E), 0)
            pb.set_style_radius(6, 0)
            pb.set_style_shadow_width(0, 0)
            pb.add_event(lambda e: self.open_chapter(self.current_ch - 1), lv.EVENT.CLICKED, None)
            pl = lv.label(pb)
            pl.set_text("< Prev")
            pl.set_style_text_font(self.font, 0)
            pl.center()

        if idx < len(self.chapters) - 1:
            nb = lv.btn(self.screen)
            nb.set_size(100, 30)
            nb.align(lv.ALIGN.BOTTOM_RIGHT, -20, -5)
            nb.set_style_bg_color(lv.color_hex(0xD2691E), 0)
            nb.set_style_radius(6, 0)
            nb.set_style_shadow_width(0, 0)
            nb.add_event(lambda e: self.open_chapter(self.current_ch + 1), lv.EVENT.CLICKED, None)
            nl = lv.label(nb)
            nl.set_text("Next >")
            nl.set_style_text_font(self.font, 0)
            nl.center()


def main():
    os.exitpoint(os.EXITPOINT_ENABLE)
    try:
        fpioa = FPIOA()
        lcd = LCD(640, 480, True, fpioa, 5, 1)
        touch = Touch()
        lvgl_init(lcd, touch)

        scr = lv.scr_act()
        scr.set_style_bg_color(lv.color_hex(0xFFF5E6), 0)

        app = BookApp(scr)

        while True:
            lv.task_handler()
            gc.collect()
            time.sleep_ms(10)

    except BaseException as e:
        import sys
        sys.print_exception(e)

    lvgl_deinit(lcd, touch)
    del lcd
    del touch
    gc.collect()

if __name__ == "__main__":
    main()

