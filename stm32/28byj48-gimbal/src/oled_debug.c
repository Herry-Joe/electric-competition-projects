/**
 * @file    oled_debug.c
 * @brief   云台调试信息 OLED 显示界面实现
 *
 * 屏幕布局 (128x64, 8 行 x 8px):
 *   L0: == 28BYJ-48 GIMBAL ==
 *   L1: MODE: SERIAL
 *   L2: PAN :RUN CW MAN 2.0ms
 *   L3: TILT:SWP CCW 2.0ms
 *   L4: SEL:PAN
 *   L5: RX:c
 *   L6: ACK:OK
 *   L7: UPT:12.3s
 */
#include "oled_debug.h"
#include "ssd1306.h"

uint8_t OLED_Debug_Init(void)
{
    return SSD1306_Init();
}

uint8_t OLED_Debug_IsReady(void)
{
    return SSD1306_IsReady();
}

void OLED_Debug_Update(const OledDebugInfo *info)
{
    if (!SSD1306_IsReady() || !info) return;

    SSD1306_Clear();

    SSD1306_DrawString(0,  0, "= 28BYJ-48 GIMBAL =", 1);
    SSD1306_DrawString(0,  8, "MODE:", 1);
    SSD1306_DrawString(36, 8, info->mode, 1);
    SSD1306_DrawString(0, 16, info->pan_line, 1);
    SSD1306_DrawString(0, 24, info->tilt_line, 1);
    SSD1306_DrawString(0, 32, info->serial_line, 1);
    SSD1306_DrawString(0, 40, info->cmd_line, 1);
    SSD1306_DrawString(0, 48, info->ack_line, 1);
    SSD1306_DrawString(0, 56, info->time_line, 1);

    SSD1306_UpdateScreen();
}
