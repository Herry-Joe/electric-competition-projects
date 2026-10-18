#ifndef __OLED_DRAW_H
#define __OLED_DRAW_H

#include "stm32f1xx_hal.h"
#include <stdint.h>

/*========================================
 * 128x64 SSD1306 OLED 图形显示模块
 * 基于帧缓冲，支持像素级绘图
 * 引脚: PB8=SCL, PB9=SDA
 *========================================*/

/*========================================
 * 工作模式枚举
 *========================================*/
typedef enum {
    MODE_DETECT = 0,   /* 声音检测模式: 频率+幅值+LED */
    MODE_LOCATE,       /* 雷达定位模式: 方向+距离+舵机 */
    MODE_FREQ_SET,     /* 频率选择模式: 选择目标频率 */
    MODE_COUNT         /* 模式总数 */
} App_Mode_t;

/*========================================
 * 显示数据结构
 *========================================*/
typedef struct {
    uint16_t freq_hz;
    uint8_t  freq_valid;
    uint16_t amp_mv;
    uint8_t  amp_valid;
    uint8_t  direction;      /* 0=无, 1~8=N/NE/E/SE/S/SW/W/NW */
    uint16_t angle_deg;
    uint8_t  confidence;
    uint16_t rms[4];
    uint16_t max_channel;
    uint8_t  equal_signal;
    uint16_t distance_cm;    /* 估算距离(cm) */
    uint16_t target_freq_hz; /* 目标频率(Hz), 用于FREQ_SET模式 */
    uint8_t  freq_option_idx; /* 当前频率选项索引 */
    uint8_t  freq_option_cnt; /* 频率选项总数 */
    App_Mode_t mode;         /* 当前工作模式 */
    uint16_t sweep_angle;    /* 雷达扫描角度(0~359), 用于LOCATE模式 */
} OLED_Screen_Data_t;

/*========================================
 * 初始化 & 刷新
 *========================================*/
void OLED_Draw_Init(void);
void OLED_Draw_Refresh(void);

/*========================================
 * 基础绘图 (写入帧缓冲)
 *========================================*/
void OLED_Draw_Clear(void);
void OLED_Draw_Pixel(uint8_t x, uint8_t y);
void OLED_Draw_Line(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1);
void OLED_Draw_Circle(uint8_t cx, uint8_t cy, uint8_t r);
void OLED_Draw_FillCircle(uint8_t cx, uint8_t cy, uint8_t r);
void OLED_Draw_Rect(uint8_t x, uint8_t y, uint8_t w, uint8_t h);
void OLED_Draw_Char(uint8_t x, uint8_t y, char c);
void OLED_Draw_String(uint8_t x, uint8_t y, const char* s);

/*========================================
 * 显示界面
 *========================================*/

/** @brief 欢迎界面 */
void OLED_Draw_Welcome(void);

/** @brief 主界面: 左侧罗盘 + 右侧数据 */
void OLED_Draw_MainScreen(const OLED_Screen_Data_t* data);

/** @brief MPU6050姿态显示界面 */
typedef struct {
    float roll;
    float pitch;
    float yaw;
    float temperature;
} Attitude_Display_t;

void OLED_Draw_Attitude(const Attitude_Display_t* attitude);

#endif /* __OLED_DRAW_H */
