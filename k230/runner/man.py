from media.display import *

from media.media import *

import machine

from machine import TOUCH

from machine import Pin

from machine import FPIOA

import os

import gc

import time

import lvgl as lv



DISPLAY_WIDTH = 640

DISPLAY_HEIGHT = 480





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



    def on(self):

        self.bl.value(self.bl_valid)



    def lvgl_flush_cb(self, disp, area, px_map):

        if disp.flush_is_last():

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



    def lvgl_read_cb(self, indev, data):

        x, y, state = 0, 0, lv.INDEV_STATE.RELEASED

        tp = self.touch.read(1)

        if len(tp):

            x, y, event = tp[0].x, tp[0].y, tp[0].event

            if event in (TOUCH.EVENT_DOWN, TOUCH.EVENT_MOVE):

                state = lv.INDEV_STATE.PRESSED

        data.point = lv.point_t({"x": x, "y": y})

        data.state = state



    def lvgl_init(self):

        self.indev = lv.indev_create()

        self.indev.set_type(lv.INDEV_TYPE.POINTER)

        self.indev.set_read_cb(self.lvgl_read_cb)



    def lvgl_deinit(self):

        del self.indev





import urandom


def show_title(lcd, touch):
    screen = lv.scr_act()
    screen.set_style_bg_color(lv.color_hex(0x1A1A2E), 0)

    font = lv.font_load("A:/sdcard/CanMV Sample/Fonts/lv_font_simsun_16_cjk.fnt")

    # ?????
    stripe = lv.obj(screen)
    stripe.set_size(DISPLAY_WIDTH, 120)
    stripe.align(lv.ALIGN.TOP_LEFT, 0, 180)
    stripe.set_style_bg_color(lv.color_hex(0x16213E), 0)
    stripe.set_style_border_width(0, 0)
    stripe.set_style_radius(0, 0)

    stripe2 = lv.obj(screen)
    stripe2.set_size(DISPLAY_WIDTH, 120)
    stripe2.align(lv.ALIGN.TOP_LEFT, 0, 300)
    stripe2.set_style_bg_color(lv.color_hex(0x0F3460), 0)
    stripe2.set_style_border_width(0, 0)
    stripe2.set_style_radius(0, 0)

    # ????(??????)
    demo_runner = lv.obj(screen)
    demo_runner.set_size(34, 46)
    demo_runner.align(lv.ALIGN.TOP_LEFT, 260, 200)
    demo_runner.set_style_bg_color(lv.color_hex(0x3A7BD5), 0)
    demo_runner.set_style_radius(8, 0)
    demo_runner.set_style_border_width(0, 0)

    demo_eye = lv.obj(screen)
    demo_eye.set_size(8, 8)
    demo_eye.align(lv.ALIGN.TOP_LEFT, 282, 208)
    demo_eye.set_style_bg_color(lv.color_hex(0xFFFFFF), 0)
    demo_eye.set_style_border_width(0, 0)
    demo_eye.set_style_radius(4, 0)

    demo_obs1 = lv.obj(screen)
    demo_obs1.set_size(28, 36)
    demo_obs1.align(lv.ALIGN.TOP_LEFT, 370, 210)
    demo_obs1.set_style_bg_color(lv.color_hex(0xD9534F), 0)
    demo_obs1.set_style_radius(4, 0)
    demo_obs1.set_style_border_width(0, 0)

    demo_obs2 = lv.obj(screen)
    demo_obs2.set_size(30, 58)
    demo_obs2.align(lv.ALIGN.TOP_LEFT, 440, 188)
    demo_obs2.set_style_bg_color(lv.color_hex(0xB54A4A), 0)
    demo_obs2.set_style_radius(3, 0)
    demo_obs2.set_style_border_width(0, 0)

    demo_bird = lv.obj(screen)
    demo_bird.set_size(30, 20)
    demo_bird.align(lv.ALIGN.TOP_LEFT, 340, 170)
    demo_bird.set_style_bg_color(lv.color_hex(0xFF8C00), 0)
    demo_bird.set_style_radius(8, 0)
    demo_bird.set_style_border_width(0, 0)

    demo_ground = lv.obj(screen)
    demo_ground.set_size(260, 6)
    demo_ground.align(lv.ALIGN.TOP_LEFT, 230, 246)
    demo_ground.set_style_bg_color(lv.color_hex(0x4A8C6F), 0)
    demo_ground.set_style_border_width(0, 0)
    demo_ground.set_style_radius(0, 0)

    # ??
    title = lv.label(screen)
    title.set_text("PARKOUR RUNNER")
    title.set_style_text_font(font, 0)
    title.set_style_text_color(lv.color_hex(0xE94560), 0)
    title.align(lv.ALIGN.TOP_MID, 0, 60)

    # ???
    sub = lv.label(screen)
    sub.set_text("Tap. Jump. Survive.")
    sub.set_style_text_font(font, 0)
    sub.set_style_text_color(lv.color_hex(0xAAAAAA), 0)
    sub.align(lv.ALIGN.TOP_MID, 0, 95)

    # ????
    info_lines = [
        "- Tap to jump, tap again for double jump",
        "- Dodge red walls, tall walls, and birds",
        "- Grab gold shield, cyan magnet, pink rocket",
    ]
    info = lv.label(screen)
    info.set_text("\n".join(info_lines))
    info.set_style_text_font(font, 0)
    info.set_style_text_color(lv.color_hex(0xCCCCCC), 0)
    info.set_width(400)
    info.align(lv.ALIGN.TOP_MID, 0, 350)

    # ????
    btn = lv.btn(screen)
    btn.set_size(220, 56)
    btn.align(lv.ALIGN.TOP_MID, 0, 130)
    btn.set_style_bg_color(lv.color_hex(0xE94560), 0)
    btn.set_style_radius(28, 0)
    btn.set_style_shadow_width(6, 0)
    btn.set_style_shadow_color(lv.color_hex(0xE94560), 0)

    btn_label = lv.label(btn)
    btn_label.set_text("START GAME")
    btn_label.set_style_text_font(font, 0)
    btn_label.set_style_text_color(lv.color_hex(0xFFFFFF), 0)
    btn_label.center()

    started = [False]

    def on_start(e):
        started[0] = True

    btn.add_event(on_start, lv.EVENT.CLICKED, None)

    # ????
    ver = lv.label(screen)
    ver.set_text("v2.0  CanMV K230")
    ver.set_style_text_font(font, 0)
    ver.set_style_text_color(lv.color_hex(0x555555), 0)
    ver.align(lv.ALIGN.BOTTOM_MID, 0, -8)

    # ??????
    touch_active = False
    while not started[0]:
        tp = touch.touch.read(1)
        pressed = bool(len(tp) and tp[0].event in (TOUCH.EVENT_DOWN, TOUCH.EVENT_MOVE))
        if pressed and not touch_active:
            started[0] = True
        touch_active = pressed
        lv.task_handler()
        gc.collect()
        time.sleep_ms(16)

    # ??????
    screen.clean()


import urandom


def show_title(lcd, touch):
    screen = lv.scr_act()
    screen.set_style_bg_color(lv.color_hex(0x1A1A2E), 0)

    font = lv.font_load("A:/sdcard/CanMV Sample/Fonts/lv_font_simsun_16_cjk.fnt")

    # ????
    stripe = lv.obj(screen)
    stripe.set_size(DISPLAY_WIDTH, 120)
    stripe.align(lv.ALIGN.TOP_LEFT, 0, 180)
    stripe.set_style_bg_color(lv.color_hex(0x16213E), 0)
    stripe.set_style_border_width(0, 0)
    stripe.set_style_radius(0, 0)

    stripe2 = lv.obj(screen)
    stripe2.set_size(DISPLAY_WIDTH, 120)
    stripe2.align(lv.ALIGN.TOP_LEFT, 0, 300)
    stripe2.set_style_bg_color(lv.color_hex(0x0F3460), 0)
    stripe2.set_style_border_width(0, 0)
    stripe2.set_style_radius(0, 0)

    # ????????????
    demo_runner = lv.obj(screen)
    demo_runner.set_size(34, 46)
    demo_runner.align(lv.ALIGN.TOP_LEFT, 260, 200)
    demo_runner.set_style_bg_color(lv.color_hex(0x3A7BD5), 0)
    demo_runner.set_style_radius(8, 0)
    demo_runner.set_style_border_width(0, 0)

    demo_eye = lv.obj(screen)
    demo_eye.set_size(8, 8)
    demo_eye.align(lv.ALIGN.TOP_LEFT, 282, 208)
    demo_eye.set_style_bg_color(lv.color_hex(0xFFFFFF), 0)
    demo_eye.set_style_border_width(0, 0)
    demo_eye.set_style_radius(4, 0)

    demo_obs1 = lv.obj(screen)
    demo_obs1.set_size(28, 36)
    demo_obs1.align(lv.ALIGN.TOP_LEFT, 370, 210)
    demo_obs1.set_style_bg_color(lv.color_hex(0xD9534F), 0)
    demo_obs1.set_style_radius(4, 0)
    demo_obs1.set_style_border_width(0, 0)

    demo_obs2 = lv.obj(screen)
    demo_obs2.set_size(30, 58)
    demo_obs2.align(lv.ALIGN.TOP_LEFT, 440, 188)
    demo_obs2.set_style_bg_color(lv.color_hex(0xB54A4A), 0)
    demo_obs2.set_style_radius(3, 0)
    demo_obs2.set_style_border_width(0, 0)

    demo_bird = lv.obj(screen)
    demo_bird.set_size(30, 20)
    demo_bird.align(lv.ALIGN.TOP_LEFT, 340, 170)
    demo_bird.set_style_bg_color(lv.color_hex(0xFF8C00), 0)
    demo_bird.set_style_radius(8, 0)
    demo_bird.set_style_border_width(0, 0)

    demo_ground = lv.obj(screen)
    demo_ground.set_size(260, 6)
    demo_ground.align(lv.ALIGN.TOP_LEFT, 230, 246)
    demo_ground.set_style_bg_color(lv.color_hex(0x4A8C6F), 0)
    demo_ground.set_style_border_width(0, 0)
    demo_ground.set_style_radius(0, 0)

    # ??
    title = lv.label(screen)
    title.set_text("PARKOUR RUNNER")
    title.set_style_text_font(font, 0)
    title.set_style_text_color(lv.color_hex(0xE94560), 0)
    title.align(lv.ALIGN.TOP_MID, 0, 55)

    # ???
    sub = lv.label(screen)
    sub.set_text("Tap. Jump. Survive.")
    sub.set_style_text_font(font, 0)
    sub.set_style_text_color(lv.color_hex(0xAAAAAA), 0)
    sub.align(lv.ALIGN.TOP_MID, 0, 90)

    # ????
    btn = lv.btn(screen)
    btn.set_size(220, 56)
    btn.align(lv.ALIGN.TOP_MID, 0, 125)
    btn.set_style_bg_color(lv.color_hex(0xE94560), 0)
    btn.set_style_radius(28, 0)
    btn.set_style_shadow_width(6, 0)
    btn.set_style_shadow_color(lv.color_hex(0xE94560), 0)

    btn_label = lv.label(btn)
    btn_label.set_text("START GAME")
    btn_label.set_style_text_font(font, 0)
    btn_label.set_style_text_color(lv.color_hex(0xFFFFFF), 0)
    btn_label.center()

    # ????
    info = lv.label(screen)
    info.set_text("Tap to jump, tap again mid-air\nDodge walls, birds\nGrab shield, magnet, rocket")
    info.set_style_text_font(font, 0)
    info.set_style_text_color(lv.color_hex(0xCCCCCC), 0)
    info.set_width(400)
    info.align(lv.ALIGN.TOP_MID, 0, 350)

    # ??
    ver = lv.label(screen)
    ver.set_text("v2.0  CanMV K230")
    ver.set_style_text_font(font, 0)
    ver.set_style_text_color(lv.color_hex(0x555555), 0)
    ver.align(lv.ALIGN.BOTTOM_MID, 0, -8)

    started = [False]

    def on_start(e):
        started[0] = True

    btn.add_event(on_start, lv.EVENT.CLICKED, None)

    # ????
    touch_active = False
    while not started[0]:
        tp = touch.touch.read(1)
        pressed = bool(len(tp) and tp[0].event in (TOUCH.EVENT_DOWN, TOUCH.EVENT_MOVE))
        if pressed and not touch_active:
            started[0] = True
        touch_active = pressed
        lv.task_handler()
        gc.collect()
        time.sleep_ms(16)

    screen.clean()


def build_gameover_panel(screen, font):
    overlay = lv.obj(screen)
    overlay.set_size(DISPLAY_WIDTH, DISPLAY_HEIGHT)
    overlay.align(lv.ALIGN.TOP_LEFT, 0, 0)
    overlay.set_style_bg_color(lv.color_hex(0x000000), 0)
    overlay.set_style_opa(140, 0)
    overlay.set_style_border_width(0, 0)
    overlay.set_style_radius(0, 0)
    overlay.add_flag(lv.obj.FLAG.CLICKABLE)

    box = lv.obj(screen)
    box.set_size(340, 260)
    box.align(lv.ALIGN.CENTER, 0, 0)
    box.set_style_bg_color(lv.color_hex(0xFFFFFF), 0)
    box.set_style_radius(16, 0)
    box.set_style_border_width(2, 0)
    box.set_style_border_color(lv.color_hex(0x3A7BD5), 0)
    box.set_style_shadow_width(12, 0)
    box.set_style_shadow_color(lv.color_hex(0x888888), 0)

    go_title = lv.label(box)
    go_title.set_text("Game Over")
    go_title.set_style_text_font(font, 0)
    go_title.set_style_text_color(lv.color_hex(0xD9534F), 0)
    go_title.align(lv.ALIGN.TOP_MID, 0, 14)

    go_score = lv.label(box)
    go_score.set_text("Score: 0")
    go_score.set_style_text_font(font, 0)
    go_score.set_style_text_color(lv.color_hex(0x1A1A1A), 0)
    go_score.align(lv.ALIGN.TOP_LEFT, 40, 50)

    go_best = lv.label(box)
    go_best.set_text("Best: 0")
    go_best.set_style_text_font(font, 0)
    go_best.set_style_text_color(lv.color_hex(0x666666), 0)
    go_best.align(lv.ALIGN.TOP_LEFT, 40, 78)

    go_rank = lv.label(box)
    go_rank.set_text("")
    go_rank.set_style_text_font(font, 0)
    go_rank.set_style_text_color(lv.color_hex(0xFF6600), 0)
    go_rank.align(lv.ALIGN.TOP_MID, 0, 112)

    go_stat = lv.label(box)
    go_stat.set_text("")
    go_stat.set_style_text_font(font, 0)
    go_stat.set_style_text_color(lv.color_hex(0x888888), 0)
    go_stat.align(lv.ALIGN.TOP_MID, 0, 140)

    go_hint = lv.label(box)
    go_hint.set_text("Tap to retry")
    go_hint.set_style_text_font(font, 0)
    go_hint.set_style_text_color(lv.color_hex(0x3A7BD5), 0)
    go_hint.align(lv.ALIGN.BOTTOM_MID, 0, -14)

    return overlay, box, go_score, go_best, go_rank, go_stat


def create_hud(screen, font):
    title = lv.label(screen)
    title.set_text("Parkour Runner")
    title.set_style_text_font(font, 0)
    title.set_style_text_color(lv.color_hex(0x333333), 0)
    title.align(lv.ALIGN.TOP_LEFT, 12, 8)

    score = lv.label(screen)
    score.set_text("Score: 0")
    score.set_style_text_font(font, 0)
    score.set_style_text_color(lv.color_hex(0x1A1A1A), 0)
    score.align(lv.ALIGN.TOP_RIGHT, -12, 8)

    power_label = lv.label(screen)
    power_label.set_text("")
    power_label.set_style_text_font(font, 0)
    power_label.set_style_text_color(lv.color_hex(0xFF6600), 0)
    power_label.align(lv.ALIGN.TOP_MID, 0, 8)

    hint = lv.label(screen)
    hint.set_text("Tap to jump, tap again mid-air")
    hint.set_style_text_font(font, 0)
    hint.set_style_text_color(lv.color_hex(0x666666), 0)
    hint.align(lv.ALIGN.BOTTOM_MID, 0, -10)

    overlay, go_box, go_score, go_best, go_rank, go_stat = build_gameover_panel(screen, font)
    overlay.add_flag(lv.obj.FLAG.HIDDEN)
    go_box.add_flag(lv.obj.FLAG.HIDDEN)

    return score, hint, power_label, overlay, go_box, go_score, go_best, go_rank, go_stat


def rand_range(lo, hi):
    return lo + (urandom.getrandbits(16) % (hi - lo + 1))


def run_game(lcd, touch):
    screen = lv.scr_act()
    screen.set_style_bg_color(lv.color_hex(0xE8F0FE), 0)

    font = lv.font_load("A:/sdcard/CanMV Sample/Fonts/lv_font_simsun_16_cjk.fnt")

    ground_y = DISPLAY_HEIGHT - 80

    ground = lv.obj(screen)
    ground.set_size(DISPLAY_WIDTH, 6)
    ground.align(lv.ALIGN.TOP_LEFT, 0, ground_y)
    ground.set_style_bg_color(lv.color_hex(0x4A8C6F), 0)
    ground.set_style_border_width(0, 0)
    ground.set_style_radius(0, 0)

    grass = lv.obj(screen)
    grass.set_size(DISPLAY_WIDTH, 14)
    grass.align(lv.ALIGN.TOP_LEFT, 0, ground_y + 6)
    grass.set_style_bg_color(lv.color_hex(0x6BBF59), 0)
    grass.set_style_border_width(0, 0)
    grass.set_style_radius(0, 0)

    runner = lv.obj(screen)
    runner.set_size(34, 46)
    runner.align(lv.ALIGN.TOP_LEFT, 120, ground_y - 46)
    runner.set_style_bg_color(lv.color_hex(0x3A7BD5), 0)
    runner.set_style_radius(8, 0)
    runner.set_style_border_width(0, 0)

    eye = lv.obj(screen)
    eye.set_size(8, 8)
    eye.align(lv.ALIGN.TOP_LEFT, 142, ground_y - 38)
    eye.set_style_bg_color(lv.color_hex(0xFFFFFF), 0)
    eye.set_style_border_width(0, 0)
    eye.set_style_radius(4, 0)

    score_label, hint_label, power_label, go_overlay, go_box, go_score_lbl, go_best_lbl, go_rank_lbl, go_stat_lbl = create_hud(screen, font)

    clouds = []
    for ci in range(4):
        c = lv.obj(screen)
        cw = 40 + (ci * 13) % 30
        ch = 18 + (ci * 7) % 12
        c.set_size(cw, ch)
        c.set_style_bg_color(lv.color_hex(0xFFFFFF), 0)
        c.set_style_border_width(0, 0)
        c.set_style_radius(ch // 2, 0)
        c.set_style_opa(180, 0)
        cx = ci * 170 + 30
        cy = 30 + (ci * 37) % 80
        c.align(lv.ALIGN.TOP_LEFT, cx, cy)
        clouds.append([c, cx, cy, cw])

    RUNNER_X = 120
    RUNNER_W = 34
    RUNNER_H = 46
    GRAVITY = 1
    JUMP_POWER = -16
    AIR_JUMPS = 2

    jump_vy = 0
    jumping = False
    air_jumps_left = AIR_JUMPS
    score = 0
    best = 0
    alive = True
    speed = 5
    max_speed = 12
    touch_active = False
    flash_tick = 0
    total_obstacles = 0
    total_powerups = 0
    total_shield_uses = 0
    total_rocket_uses = 0

    obstacles = []
    spawn_cd = 0

    powerups = []
    powerup_cd = 0

    shield_timer = 0
    magnet_timer = 0
    rocket_timer = 0

    last_ms = time.ticks_ms()

    try:
        while True:
            now = time.ticks_ms()
            dt = now - last_ms
            if dt < 16:
                lv.task_handler()
                gc.collect()
                time.sleep_ms(1)
                continue
            last_ms = now

            if alive:
                tp = touch.touch.read(1)
                pressed = bool(len(tp) and tp[0].event in (TOUCH.EVENT_DOWN, TOUCH.EVENT_MOVE))
                if pressed and not touch_active:
                    if not jumping:
                        jumping = True
                        jump_vy = JUMP_POWER
                    elif air_jumps_left > 0:
                        air_jumps_left -= 1
                        jump_vy = JUMP_POWER
                touch_active = pressed

                if rocket_timer > 0:
                    jumping = True
                    jump_vy = -8

                if jumping:
                    jump_vy += GRAVITY
                    new_y = runner.get_y() + jump_vy
                    if new_y >= ground_y - RUNNER_H:
                        new_y = ground_y - RUNNER_H
                        if rocket_timer <= 0:
                            jumping = False
                        jump_vy = 0
                        air_jumps_left = AIR_JUMPS
                    runner.align(lv.ALIGN.TOP_LEFT, RUNNER_X, new_y)
                    eye.align(lv.ALIGN.TOP_LEFT, RUNNER_X + 22, new_y + 8)

                for ci in range(len(clouds)):
                    c, cx, cy, cw = clouds[ci]
                    cx -= 1
                    if cx < -cw - 10:
                        cx = DISPLAY_WIDTH + rand_range(10, 60)
                    clouds[ci][1] = cx
                    c.align(lv.ALIGN.TOP_LEFT, cx, cy)

                # ????
                spawn_cd -= 1
                if spawn_cd <= 0:
                    r = urandom.getrandbits(8) % 10
                    if r < 4:
                        ow, oh = 28, 36
                        o = lv.obj(screen)
                        o.set_size(ow, oh)
                        o.set_style_bg_color(lv.color_hex(0xD9534F), 0)
                        o.set_style_radius(4, 0)
                        o.set_style_border_width(0, 0)
                        oy = ground_y - oh
                        o.align(lv.ALIGN.TOP_LEFT, DISPLAY_WIDTH + 20, oy)
                        obstacles.append([o, DISPLAY_WIDTH + 20, oy, 0, False])
                    elif r < 7:
                        ow, oh = 30, 58
                        o = lv.obj(screen)
                        o.set_size(ow, oh)
                        o.set_style_bg_color(lv.color_hex(0xB54A4A), 0)
                        o.set_style_radius(3, 0)
                        o.set_style_border_width(0, 0)
                        oy = ground_y - oh
                        o.align(lv.ALIGN.TOP_LEFT, DISPLAY_WIDTH + 20, oy)
                        obstacles.append([o, DISPLAY_WIDTH + 20, oy, 1, False])
                    else:
                        ow, oh = 30, 20
                        o = lv.obj(screen)
                        o.set_size(ow, oh)
                        o.set_style_bg_color(lv.color_hex(0xFF8C00), 0)
                        o.set_style_radius(8, 0)
                        o.set_style_border_width(0, 0)
                        oy = ground_y - rand_range(80, 130)
                        o.align(lv.ALIGN.TOP_LEFT, DISPLAY_WIDTH + 20, oy)
                        obstacles.append([o, DISPLAY_WIDTH + 20, oy, 2, False])
                    gap = max(40, 90 - score * 2)
                    spawn_cd = gap + rand_range(10, 40)

                # ????
                powerup_cd -= 1
                if powerup_cd <= 0 and len(powerups) == 0:
                    r = urandom.getrandbits(8) % 3
                    pw, ph = 26, 26
                    pu = lv.obj(screen)
                    pu.set_size(pw, ph)
                    pu.set_style_radius(13, 0)
                    pu.set_style_border_width(2, 0)
                    if r == 0:
                        pu.set_style_bg_color(lv.color_hex(0xFFD700), 0)
                        pu.set_style_border_color(lv.color_hex(0xFFA500), 0)
                    elif r == 1:
                        pu.set_style_bg_color(lv.color_hex(0x00CED1), 0)
                        pu.set_style_border_color(lv.color_hex(0x008B8B), 0)
                    else:
                        pu.set_style_bg_color(lv.color_hex(0xFF69B4), 0)
                        pu.set_style_border_color(lv.color_hex(0xFF1493), 0)
                    pu_y = ground_y - rand_range(55, 120)
                    pu.align(lv.ALIGN.TOP_LEFT, DISPLAY_WIDTH + 20, pu_y)
                    powerups.append([pu, DISPLAY_WIDTH + 20, pu_y, r])
                    powerup_cd = rand_range(180, 320)

                # ????
                rx, ry = runner.get_x(), runner.get_y()
                remaining = []
                for ob in obstacles:
                    o, ox, oy, otype, passed = ob
                    ox -= speed
                    if otype == 2:
                        oy += (1 if (now // 300) % 2 == 0 else -1)
                    o.align(lv.ALIGN.TOP_LEFT, ox, oy)

                    if not passed and ox + o.get_width() < RUNNER_X:
                        ob[4] = True
                        score += 1
                        total_obstacles += 1
                        if score % 6 == 0 and speed < max_speed:
                            speed += 1

                    if ox < -80:
                        o.delete()
                        continue

                    ow2 = o.get_width()
                    oh2 = o.get_height()
                    rtop = runner.get_y()
                    if (RUNNER_X + RUNNER_W - 6 > ox + 2 and
                        RUNNER_X + 6 < ox + ow2 - 2 and
                        rtop + RUNNER_H - 4 > oy + 2 and
                        rtop + 4 < oy + oh2 - 2):
                        if rocket_timer > 0:
                            o.delete()
                            continue
                        if shield_timer > 0:
                            shield_timer = 0
                            o.delete()
                            continue
                        alive = False
                        if score > best:
                            best = score
                        o.delete()
                        break

                    ob[1] = ox
                    ob[2] = oy
                    remaining.append(ob)

                if not alive:
                    for ob2 in remaining:
                        ob2[0].delete()
                    obstacles.clear()
                else:
                    obstacles = remaining

                # ????
                pu_remaining = []
                for pu in powerups:
                    o, px, py, ptype = pu
                    px -= speed
                    o.align(lv.ALIGN.TOP_LEFT, px, py)
                    if px < -40:
                        o.delete()
                        continue
                    pw2 = o.get_width()
                    ph2 = o.get_height()
                    rtop = runner.get_y()
                    if (RUNNER_X + RUNNER_W - 6 > px and
                        RUNNER_X + 6 < px + pw2 and
                        rtop + RUNNER_H - 4 > py and
                        rtop + 4 < py + ph2):
                        o.delete()
                        if ptype == 0:
                            shield_timer = 400
                            total_shield_uses += 1
                        elif ptype == 1:
                            magnet_timer = 350
                            total_powerups += 1
                        else:
                            rocket_timer = 250
                            total_rocket_uses += 1
                        score += 3
                        continue
                    pu[1] = px
                    pu_remaining.append(pu)
                powerups = pu_remaining

                # ????
                if magnet_timer > 0:
                    magnet_timer -= 1
                    if magnet_timer % 8 == 0:
                        score += 1
                if shield_timer > 0:
                    shield_timer -= 1
                if rocket_timer > 0:
                    rocket_timer -= 1
                    if rocket_timer <= 0:
                        jumping = False
                        jump_vy = 0
                        air_jumps_left = AIR_JUMPS
                        runner.align(lv.ALIGN.TOP_LEFT, RUNNER_X, ground_y - RUNNER_H)
                        eye.align(lv.ALIGN.TOP_LEFT, RUNNER_X + 22, ground_y - 38)

                # HUD
                flash_tick += 1
                parts = []
                if shield_timer > 0:
                    if shield_timer >= 60 or flash_tick % 4 < 2:
                        parts.append("[shield]")
                if magnet_timer > 0:
                    if magnet_timer >= 60 or flash_tick % 4 < 2:
                        parts.append("[magnet]")
                if rocket_timer > 0:
                    if rocket_timer >= 60 or flash_tick % 4 < 2:
                        parts.append("[rocket]")
                power_label.set_text(" ".join(parts))

                if rocket_timer > 0:
                    runner.set_style_bg_color(lv.color_hex(0xFF69B4), 0)
                elif shield_timer > 0 and flash_tick % 6 < 3:
                    runner.set_style_bg_color(lv.color_hex(0xFFD700), 0)
                else:
                    runner.set_style_bg_color(lv.color_hex(0x3A7BD5), 0)

                score_label.set_text("Score: " + str(score))

                # ????
                if not alive:
                    for pu in powerups:
                        pu[0].delete()
                    powerups.clear()

                    if score >= 50:
                        rank = "S  LEGENDARY!"
                    elif score >= 30:
                        rank = "A  EXCELLENT"
                    elif score >= 15:
                        rank = "B  GOOD JOB"
                    elif score >= 5:
                        rank = "C  NOT BAD"
                    else:
                        rank = "D  TRY HARDER"

                    stat_text = "obstacles:" + str(total_obstacles) + "  items:" + str(total_powerups + total_shield_uses + total_rocket_uses)

                    go_score_lbl.set_text("Score: " + str(score))
                    go_best_lbl.set_text("Best: " + str(best))
                    go_rank_lbl.set_text(rank)
                    go_stat_lbl.set_text(stat_text)

                    go_overlay.clear_flag(lv.obj.FLAG.HIDDEN)
                    go_box.clear_flag(lv.obj.FLAG.HIDDEN)

                    score_label.set_text("Best: " + str(best))
                    hint_label.set_text("")

            else:
                tp = touch.touch.read(1)
                pressed = bool(len(tp) and tp[0].event in (TOUCH.EVENT_DOWN, TOUCH.EVENT_MOVE))
                if pressed and not touch_active:
                    alive = True
                    go_overlay.add_flag(lv.obj.FLAG.HIDDEN)
                    go_box.add_flag(lv.obj.FLAG.HIDDEN)
                    score = 0
                    speed = 5
                    total_obstacles = 0
                    total_powerups = 0
                    total_shield_uses = 0
                    total_rocket_uses = 0
                    jumping = False
                    jump_vy = 0
                    air_jumps_left = AIR_JUMPS
                    shield_timer = 0
                    magnet_timer = 0
                    rocket_timer = 0
                    spawn_cd = 60
                    powerup_cd = 120
                    runner.align(lv.ALIGN.TOP_LEFT, RUNNER_X, ground_y - RUNNER_H)
                    eye.align(lv.ALIGN.TOP_LEFT, RUNNER_X + 22, ground_y - 38)
                    runner.set_style_bg_color(lv.color_hex(0x3A7BD5), 0)
                    score_label.set_text("Score: 0")
                    hint_label.set_text("Tap to jump, tap again mid-air")
                    power_label.set_text("")
                touch_active = pressed

            lv.task_handler()
            gc.collect()

    except BaseException as e:
        import sys
        sys.print_exception(e)


def main():
    os.exitpoint(os.EXITPOINT_ENABLE)
    try:
        fpioa = FPIOA()
        lcd = LCD(DISPLAY_WIDTH, DISPLAY_HEIGHT, True, fpioa, 5, 1)
        touch = Touch()

        lv.init()
        lcd.lvgl_init(DISPLAY_WIDTH, DISPLAY_HEIGHT)
        touch.lvgl_init()

        show_title(lcd, touch)
        run_game(lcd, touch)

    except BaseException as e:
        import sys
        sys.print_exception(e)

    finally:
        try:
            lcd.lvgl_deinit()
            touch.lvgl_deinit()
            lv.deinit()
        except Exception:
            pass
        del lcd
        del touch
        gc.collect()


if __name__ == "__main__":
    main()
