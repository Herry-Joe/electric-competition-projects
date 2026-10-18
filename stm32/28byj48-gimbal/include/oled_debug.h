/**
 * @file    oled_debug.h
 * @brief   云台调试信息 OLED 显示界面 (辅助串口调试)
 * @note    周期性调用 OLED_Debug_Update() 把当前云台状态 / 串口命令 / 回执
 *          显示到 SSD1306 屏幕, 无需打开串口助手即可观察系统状态
 */
#ifndef __OLED_DEBUG_H
#define __OLED_DEBUG_H

#include "main.h"

/* ========================== 调试信息结构体 ========================== */
typedef struct {
    const char *mode;            /* 当前云台模式: PAN/TILT/SWEEP/SERIAL/DANCE */
    char pan_line[26];           /* Pan 轴状态行 */
    char tilt_line[26];          /* Tilt 轴状态行 */
    char serial_line[26];        /* 串口选中电机 */
    char cmd_line[26];           /* 最近收到的串口命令 (RX) */
    char ack_line[26];           /* 最近回执 (ACK: OK / ERR) */
    char time_line[26];          /* 运行时间 */
} OledDebugInfo;

/* ========================== 公共接口 ========================== */
uint8_t OLED_Debug_Init(void);      /* 返回 1=OLED 就绪, 0=未检测到 */
void OLED_Debug_Update(const OledDebugInfo *info);
uint8_t OLED_Debug_IsReady(void);

#endif /* __OLED_DEBUG_H */
