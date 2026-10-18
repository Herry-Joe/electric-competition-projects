#include "oled_draw.h"

/* 引用OLED.c中已定义的字体，避免重复定义 */
extern const uint8_t OLED_F8x16[][16];

/*========================================
 * 硬件引脚配置
 *========================================*/
#define SCL_PORT GPIOB
#define SCL_PIN  GPIO_PIN_8
#define SDA_PORT GPIOB
#define SDA_PIN  GPIO_PIN_9

#define W_SCL(x) HAL_GPIO_WritePin(SCL_PORT, SCL_PIN, (x) ? GPIO_PIN_SET : GPIO_PIN_RESET)
#define W_SDA(x) HAL_GPIO_WritePin(SDA_PORT, SDA_PIN, (x) ? GPIO_PIN_SET : GPIO_PIN_RESET)

#define ADDR 0x78

/*========================================
 * 帧缓冲: 128列 x 8页 = 1024字节
 * 每字节控制8个垂直像素
 *========================================*/
static uint8_t fb[1024];

/*========================================
 * 屏幕布局参数 (从屏幕尺寸自动推算)
 * 修改这5个常量即可适配不同屏幕/字体
 *========================================*/
#define SCR_W     128          /* 屏幕宽度(像素) */
#define SCR_H     64           /* 屏幕高度(像素) */
#define FONT_W    8            /* 字体宽度(像素) */
#define FONT_H    16           /* 字体高度(像素) */
#define MARGIN    2            /* 边距(像素) */
#define ROW_H     (FONT_H - 1) /* 行高: 字体高度-1, 64px屏可排4行 */

/*========================================
 * 软件I2C
 * 说明: STM32@72MHz 下裸 GPIO 翻转极快, SCL 周期可能只有 ~100ns(≈5MHz),
 *       远超 SSD1306 的 400kHz 上限 -> 芯片采样跟不上、字节错位 -> 屏幕乱码。
 *       下面 i2c_delay() 把每位拉宽到 ~5us, 把总线降到 ~100~200kHz, 稳定可靠。
 *========================================*/
static void i2c_delay(void)
{
    volatile uint32_t t = 50;   /* 约 3~5us @ 72MHz, 可按实际微调 */
    while (t--) __NOP();
}

static void i2c_start(void)
{
    W_SDA(1); W_SCL(1); i2c_delay();
    W_SDA(0); i2c_delay();
    W_SCL(0);
}

static void i2c_stop(void)
{
    W_SDA(0); W_SCL(1); i2c_delay();
    W_SDA(1); i2c_delay();
}

static void i2c_write(uint8_t d)
{
    uint8_t i;
    for (i = 0; i < 8; i++) {
        W_SDA(!!(d & (0x80 >> i)));
        i2c_delay();
        W_SCL(1); i2c_delay();
        W_SCL(0);
    }
    W_SDA(0);  /* ACK */
    i2c_delay();
    W_SCL(1); i2c_delay();
    W_SCL(0);
}

static void wr_cmd(uint8_t cmd)
{
    i2c_start(); i2c_write(ADDR); i2c_write(0x00); i2c_write(cmd); i2c_stop();
}


/*========================================
 * 初始化
 *========================================*/
void OLED_Draw_Init(void)
{
    GPIO_InitTypeDef g;
    uint32_t i, j;

    /* 上电延时 */
    for (i = 0; i < 1000; i++)
        for (j = 0; j < 1000; j++);

    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* PB6=OLED_VCC (推挽输出高), PB7=OLED_GND (推挽输出低) */
    g.Mode = GPIO_MODE_OUTPUT_PP;
    g.Speed = GPIO_SPEED_FREQ_HIGH;
    g.Pin = GPIO_PIN_6;
    HAL_GPIO_Init(GPIOB, &g);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);   /* VCC=高 */

    g.Pin = GPIO_PIN_7;
    HAL_GPIO_Init(GPIOB, &g);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET); /* GND=低 */

    /* PB8=SCL, PB9=SDA (开漏) */
    g.Mode = GPIO_MODE_OUTPUT_OD;
    g.Pin = SCL_PIN; HAL_GPIO_Init(SCL_PORT, &g);
    g.Pin = SDA_PIN; HAL_GPIO_Init(SDA_PORT, &g);
    W_SCL(1); W_SDA(1);

    HAL_Delay(120);  /* 上电后等 VCC 稳定再发命令(SSD1306 要求) */

    wr_cmd(0xAE);        /* 关显示 */
    wr_cmd(0xD5); wr_cmd(0x80);
    wr_cmd(0xA8); wr_cmd(0x3F);
    wr_cmd(0xD3); wr_cmd(0x00);
    wr_cmd(0x40);
    wr_cmd(0xA1);        /* 水平翻转 */
    wr_cmd(0xC8);        /* 垂直翻转 */
    wr_cmd(0xDA); wr_cmd(0x12);
    wr_cmd(0x81); wr_cmd(0xCF);
    wr_cmd(0xD9); wr_cmd(0xF1);
    wr_cmd(0xDB); wr_cmd(0x30);
    wr_cmd(0xA4);        /* 全屏显示关 */
    wr_cmd(0xA6);        /* 正常显示(非反色) */
    wr_cmd(0x20); wr_cmd(0x02);  /* 显存寻址: 页地址模式(最稳) */
    wr_cmd(0x8D); wr_cmd(0x14);  /* 电荷泵 */
    HAL_Delay(100);              /* 电荷泵建立 VCC, 必须等 */
    wr_cmd(0xAF);        /* 开显示 */

    OLED_Draw_Clear();
    OLED_Draw_Refresh();
    HAL_Delay(20);
    OLED_Draw_Refresh();  /* 再刷一次, 确保首帧稳定 */
}

/*========================================
 * 设置光标 (与参考代码一致)
 *========================================*/
static void set_cursor(uint8_t page, uint8_t col)
{
    wr_cmd(0xB0 | page);
    wr_cmd(0x10 | ((col & 0xF0) >> 4));
    wr_cmd(0x00 | (col & 0x0F));
}

/*========================================
 * 帧缓冲 → 屏幕 (整页连续发送, 速度快)
 *========================================*/
void OLED_Draw_Refresh(void)
{
    uint8_t page, col;
    for (page = 0; page < 8; page++) {
        set_cursor(page, 0);
        i2c_start();
        i2c_write(ADDR);
        i2c_write(0x40);  /* 数据模式 */
        for (col = 0; col < 128; col++) {
            i2c_write(fb[page * 128 + col]);
        }
        i2c_stop();
    }
}

/*========================================
 * 清屏 (帧缓冲)
 *========================================*/
void OLED_Draw_Clear(void)
{
    uint16_t i;
    for (i = 0; i < 1024; i++) fb[i] = 0;
}

/*========================================
 * 画点 (带严格边界检查)
 *========================================*/
void OLED_Draw_Pixel(uint8_t x, uint8_t y)
{
    if (x > 127 || y > 63) return;
    fb[x + (y >> 3) * 128] |= (1 << (y & 7));
}

/*========================================
 * Bresenham直线
 *========================================*/
void OLED_Draw_Line(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1)
{
    int16_t dx = (x1 > x0) ? (x1 - x0) : (x0 - x1);
    int16_t dy = (y1 > y0) ? (y1 - y0) : (y0 - y1);
    int16_t sx = (x0 < x1) ? 1 : -1;
    int16_t sy = (y0 < y1) ? 1 : -1;
    int16_t err = dx - dy;
    int16_t e2;

    for (;;) {
        OLED_Draw_Pixel((uint8_t)x0, (uint8_t)y0);
        if (x0 == x1 && y0 == y1) break;
        e2 = 2 * err;
        if (e2 > -dy) { err -= dy; x0 = (uint8_t)(x0 + sx); }
        if (e2 <  dx) { err += dx; y0 = (uint8_t)(y0 + sy); }
    }
}

/*========================================
 * 画圆 (中点圆算法)
 *========================================*/
void OLED_Draw_Circle(uint8_t cx, uint8_t cy, uint8_t r)
{
    int16_t x = 0, y = (int16_t)r;
    int16_t d = 1 - (int16_t)r;

    while (x <= y) {
        OLED_Draw_Pixel((uint8_t)(cx + x), (uint8_t)(cy + y));
        OLED_Draw_Pixel((uint8_t)(cx - x), (uint8_t)(cy + y));
        OLED_Draw_Pixel((uint8_t)(cx + x), (uint8_t)(cy - y));
        OLED_Draw_Pixel((uint8_t)(cx - x), (uint8_t)(cy - y));
        OLED_Draw_Pixel((uint8_t)(cx + y), (uint8_t)(cy + x));
        OLED_Draw_Pixel((uint8_t)(cx - y), (uint8_t)(cy + x));
        OLED_Draw_Pixel((uint8_t)(cx + y), (uint8_t)(cy - x));
        OLED_Draw_Pixel((uint8_t)(cx - y), (uint8_t)(cy - x));
        if (d < 0) {
            d += 2 * x + 3;
        } else {
            d += 2 * (x - y) + 5;
            y--;
        }
        x++;
    }
}

/*========================================
 * 填充圆
 *========================================*/
void OLED_Draw_FillCircle(uint8_t cx, uint8_t cy, uint8_t r)
{
    int16_t x = 0, y = (int16_t)r;
    int16_t d = 1 - (int16_t)r;

    while (x <= y) {
        OLED_Draw_Line((uint8_t)(cx - x), (uint8_t)(cy + y),
                       (uint8_t)(cx + x), (uint8_t)(cy + y));
        OLED_Draw_Line((uint8_t)(cx - x), (uint8_t)(cy - y),
                       (uint8_t)(cx + x), (uint8_t)(cy - y));
        OLED_Draw_Line((uint8_t)(cx - y), (uint8_t)(cy + x),
                       (uint8_t)(cx + y), (uint8_t)(cy + x));
        OLED_Draw_Line((uint8_t)(cx - y), (uint8_t)(cy - x),
                       (uint8_t)(cx + y), (uint8_t)(cy - x));
        if (d < 0) {
            d += 2 * x + 3;
        } else {
            d += 2 * (x - y) + 5;
            y--;
        }
        x++;
    }
}

/*========================================
 * 画矩形
 *========================================*/
void OLED_Draw_Rect(uint8_t x, uint8_t y, uint8_t w, uint8_t h)
{
    OLED_Draw_Line(x, y, x + w - 1, y);
    OLED_Draw_Line(x, y + h - 1, x + w - 1, y + h - 1);
    OLED_Draw_Line(x, y, x, y + h - 1);
    OLED_Draw_Line(x + w - 1, y, x + w - 1, y + h - 1);
}

/*========================================
 * 清除矩形区域 (写0)
 *========================================*/
static void clear_area(uint8_t x, uint8_t y, uint8_t w, uint8_t h)
{
    uint16_t col;
    uint8_t py;
    uint8_t page_start = y >> 3;
    uint8_t page_end = (y + h - 1) >> 3;
    uint16_t col_end = (uint16_t)x + (uint16_t)w;
    for (py = page_start; py <= page_end; py++)
        for (col = x; col < 128 && col < col_end; col++)
            fb[col + py * 128] = 0;
}

/*========================================
 * 反色字符 (白底黑字, 8x16)
 *========================================*/
static void OLED_Draw_Char_Inv(uint8_t x, uint8_t y, char c)
{
    uint8_t col, row;
    if (c < ' ' || c > '~') return;
    /* 先填充白色背景 */
    clear_area(x, y, 8, 16);
    for (col = 0; col < 8; col++) {
        uint8_t d = ~OLED_F8x16[c - ' '][col];
        for (row = 0; row < 8; row++) {
            if (d & (1 << row)) OLED_Draw_Pixel(x + col, y + row);
        }
        d = ~OLED_F8x16[c - ' '][col + 8];
        for (row = 0; row < 8; row++) {
            if (d & (1 << row)) OLED_Draw_Pixel(x + col, y + 8 + row);
        }
    }
}

/*========================================
 * 反色字符串
 *========================================*/
static void OLED_Draw_String_Inv(uint8_t x, uint8_t y, const char* s)
{
    while (*s) {
        OLED_Draw_Char_Inv(x, y, *s);
        x += 8;
        s++;
    }
}

/*========================================
 * 水平虚线
 *========================================*/
static void draw_dashed_hline(uint8_t y)
{
    uint8_t x;
    for (x = 0; x < 128; x += 4)
        OLED_Draw_Pixel(x, y);
}

/*========================================
 * 画字符 (8x16字体)
 *========================================*/
void OLED_Draw_Char(uint8_t x, uint8_t y, char c)
{
    uint8_t col;
    if (c < ' ' || c > '~') return;
    for (col = 0; col < 8; col++) {
        uint8_t d = OLED_F8x16[c - ' '][col];
        uint8_t row;
        for (row = 0; row < 8; row++) {
            if (d & (1 << row)) OLED_Draw_Pixel(x + col, y + row);
        }
        d = OLED_F8x16[c - ' '][col + 8];
        for (row = 0; row < 8; row++) {
            if (d & (1 << row)) OLED_Draw_Pixel(x + col, y + 8 + row);
        }
    }
}

/*========================================
 * 画字符串
 *========================================*/
void OLED_Draw_String(uint8_t x, uint8_t y, const char* s)
{
    while (*s) {
        OLED_Draw_Char(x, y, *s);
        x += 8;
        s++;
    }
}

/*========================================
 * 数字转字符串 (局部)
 *========================================*/
static uint8_t u16_to_str(uint16_t v, char* buf)
{
    char tmp[6];
    uint8_t i = 0, len = 0;
    if (v == 0) { buf[0] = '0'; buf[1] = '\0'; return 1; }
    while (v > 0) { tmp[i++] = '0' + (v % 10); v /= 10; }
    while (i > 0) buf[len++] = tmp[--i];
    buf[len] = '\0';
    return len;
}

static void u16_right_align(uint16_t v, char* buf, uint8_t width)
{
    char tmp[6];
    uint8_t len = u16_to_str(v, tmp);
    uint8_t i;
    for (i = 0; i < width; i++) buf[i] = ' ';
    buf[width] = '\0';
    for (i = 0; i < len && i < width; i++)
        buf[width - len + i] = tmp[i];
}

/*========================================
 * 模式名称表
 *========================================*/
static const char* mode_names[] = {
    "DETECT", "LOCATE", "FREQ"
};

/*========================================
 * 方向名称表
 *========================================*/
static const char* dir_names[] = {
    "---", "N", "NE", "E", "SE", "S", "SW", "W", "NW"
};

/*========================================
 * 绘制模式指示栏 (反色背景, 右侧可选信息)
 *========================================*/
static void draw_mode_bar(App_Mode_t mode, const char* info)
{
    uint8_t x, len;
    /* 反色背景: y=0~15, 全宽128像素 */
    for (x = 0; x < 128; x++) {
        fb[x] = 0xFF;
        fb[x + 128] = 0xFF;
    }
    /* 模式名 (反色) */
    OLED_Draw_String_Inv(0, 0, mode_names[mode]);
    /* 右侧信息 (反色, 右对齐) */
    if (info) {
        len = 0;
        while (info[len]) len++;
        OLED_Draw_String_Inv(128 - len * 8, 0, info);
    }
    /* y=16 分隔虚线 */
    draw_dashed_hline(16);
}

/*========================================
 * 欢迎界面
 *========================================*/
void OLED_Draw_Welcome(void)
{
    OLED_Draw_Clear();
    OLED_Draw_String((SCR_W - FONT_W * 14) / 2, SCR_H / 2 - FONT_H,
                     "Sound Detector");
    OLED_Draw_String((SCR_W - FONT_W * 15) / 2, SCR_H / 2 + MARGIN,
                     "Initializing...");
    OLED_Draw_Refresh();
}

/*========================================
 * 模式1: 声音检测模式
 *
 *  DETECT      1000Hz   ← 反色模式栏+目标频率
 *  ─ ─ ─ ─ ─ ─ ─ ─ ─  ← 虚线分隔
 *  F: 1234 Hz
 *  A:  250 mV
 *  D:  120 cm
 *           DETECTED    ← 右下角状态
 *========================================*/
static void draw_detect_mode(const OLED_Screen_Data_t* data)
{
    char buf[12];
    char info[8];
    uint8_t y;

    /* 模式栏右侧: 目标频率 + "Hz" */
    u16_to_str(data->target_freq_hz, info);
    {
        uint8_t i = 0;
        while (info[i]) i++;     /* 找到末尾 */
        info[i++] = 'H';
        info[i++] = 'z';
        info[i] = '\0';
    }
    draw_mode_bar(MODE_DETECT, info);

    /* 频率 (始终显示, 便于调试; LED仅在valid时亮) */
    y = FONT_H + MARGIN;
    OLED_Draw_String(MARGIN, y, "F:");
    u16_right_align(data->freq_hz, buf, 5);
    OLED_Draw_String(FONT_W * 2 + MARGIN, y, buf);
    OLED_Draw_String(FONT_W * 7 + MARGIN, y, "Hz");
    y += ROW_H;

    /* 幅值 */
    OLED_Draw_String(MARGIN, y, "A:");
    if (data->amp_valid) {
        u16_right_align(data->amp_mv, buf, 4);
        OLED_Draw_String(FONT_W * 2 + MARGIN, y, buf);
        OLED_Draw_String(FONT_W * 6 + MARGIN, y, "mV");
    } else {
        OLED_Draw_String(FONT_W * 2 + MARGIN, y, "----");
    }
    y += ROW_H;

    /* 距离 */
    OLED_Draw_String(MARGIN, y, "D:");
    if (data->distance_cm > 0 && data->distance_cm < 9999) {
        u16_right_align(data->distance_cm, buf, 4);
        OLED_Draw_String(FONT_W * 2 + MARGIN, y, buf);
        OLED_Draw_String(FONT_W * 6 + MARGIN, y, "cm");
    } else {
        OLED_Draw_String(FONT_W * 2 + MARGIN, y, "----");
    }

    /* 右下角状态 */
    if (data->freq_valid && data->amp_valid) {
        OLED_Draw_String(SCR_W - FONT_W * 8 - MARGIN, SCR_H - FONT_H, "DETECTED");
    }
}

/*========================================
 * 模式2: 雷达图定位模式 (无模式栏, 全屏雷达)
 *
 *      N                 ┃ Dir: NE
 *    W + E               ┃ Ang:  45
 *      S                 ┃ Dst: 120
 *   · · · ·              ┃ Cf:  85%
 *      ↑                 ┃
 *   左: 雷达(r=25)   ┃ 右: 数据面板
 *========================================*/
static void draw_locate_mode(const OLED_Screen_Data_t* data)
{
    char buf[10];
    uint8_t y, i;
    uint8_t show_dir;   /* 方向有效 */

    /* 雷达参数 */
    const uint8_t cx = 30;
    const uint8_t cy = 30;
    const uint8_t r  = 22;
    const uint8_t rx = 58;   /* 右侧数据x起始 */

    /* 方向有效: direction 1~8 即可, 不要求距离 */
    show_dir = (data->direction >= 1 && data->direction <= 8);

    /*=== 左侧: 雷达图 ===*/

    /* 外圈 */
    OLED_Draw_Circle(cx, cy, r);
    OLED_Draw_Circle(cx, cy, r - 1);

    /* 内圈 */
    OLED_Draw_Circle(cx, cy, r / 2);

    /* 十字线 */
    OLED_Draw_Line(cx, cy - r + 2, cx, cy + r - 2);
    OLED_Draw_Line(cx - r + 2, cy, cx + r - 2, cy);

    /* 8方向刻度线 */
    {
        static const int16_t sin8[] = {0, 383, 707, 924, 1000, 924, 707, 383};
        static const int16_t cos8[] = {-1000, -924, -707, -383, 0, 383, 707, 924};
        for (i = 0; i < 8; i++) {
            uint8_t ix = (uint8_t)(cx + (int8_t)((int32_t)(r - 1) * sin8[i] / 1000));
            uint8_t iy = (uint8_t)(cy - (int8_t)((int32_t)(r - 1) * cos8[i] / 1000));
            uint8_t ox = (uint8_t)(cx + (int8_t)((int32_t)(r + 1) * sin8[i] / 1000));
            uint8_t oy = (uint8_t)(cy - (int8_t)((int32_t)(r + 1) * cos8[i] / 1000));
            OLED_Draw_Line(ix, iy, ox, oy);
        }
    }

    /* N/E/S/W 标签 */
    OLED_Draw_Char(cx - FONT_W / 2, cy - r - FONT_H + 2, 'N');
    OLED_Draw_Char(cx + r + 2, cy - FONT_H / 2, 'E');
    OLED_Draw_Char(cx - FONT_W / 2, cy + r + 2, 'S');
    OLED_Draw_Char(cx - r - FONT_W - 1, cy - FONT_H / 2, 'W');

    /* 扫描线: 360度连续旋转 */
    {
        static const int16_t sin_lut[] = {
               0,  17,  35,  52,  70,  87, 105, 122, 139, 156,
             174, 191, 208, 225, 242, 259, 276, 292, 309, 326,
             342, 358, 375, 391, 407, 423, 438, 454, 469, 485,
             500, 515, 530, 545, 559, 574, 588, 602, 616, 629,
             643, 656, 669, 682, 695, 707, 719, 731, 743, 755,
             766, 777, 788, 799, 809, 819, 829, 839, 848, 857,
             866, 875, 883, 891, 899, 906, 914, 921, 927, 934,
             940, 946, 951, 956, 961, 966, 970, 974, 978, 982,
             985, 988, 990, 993, 995, 996, 998, 999, 999,1000
        };
        int16_t sx_v, sy_v;
        uint16_t a = data->sweep_angle % 360;
        uint16_t quadrant = a / 90;
        uint16_t idx = a % 90;
        int16_t s = sin_lut[idx];
        int16_t c = sin_lut[90 - idx];

        switch (quadrant) {
            case 0: sx_v =  s; sy_v = -c; break;
            case 1: sx_v =  c; sy_v =  s; break;
            case 2: sx_v = -s; sy_v =  c; break;
            default: sx_v = -c; sy_v = -s; break;
        }
        {
            uint8_t ex = (uint8_t)(cx + (int8_t)((int32_t)r * sx_v / 1000));
            uint8_t ey = (uint8_t)(cy + (int8_t)((int32_t)r * sy_v / 1000));
            OLED_Draw_Line(cx, cy, ex, ey);
        }
    }

    /* 中心标记 */
    OLED_Draw_Pixel(cx - 1, cy);
    OLED_Draw_Pixel(cx + 1, cy);
    OLED_Draw_Pixel(cx, cy - 1);
    OLED_Draw_Pixel(cx, cy + 1);

    /* 声源方向标记: 用连续角度 (0~359°) */
    if (show_dir) {
        /* 复用扫描线的 sin_lut, 角度映射: 0°=N(上), 90°=E(右), 180°=S(下), 270°=W(左) */
        static const int16_t sin_lut[] = {
               0,  17,  35,  52,  70,  87, 105, 122, 139, 156,
             174, 191, 208, 225, 242, 259, 276, 292, 309, 326,
             342, 358, 375, 391, 407, 423, 438, 454, 469, 485,
             500, 515, 530, 545, 559, 574, 588, 602, 616, 629,
             643, 656, 669, 682, 695, 707, 719, 731, 743, 755,
             766, 777, 788, 799, 809, 819, 829, 839, 848, 857,
             866, 875, 883, 891, 899, 906, 914, 921, 927, 934,
             940, 946, 951, 956, 961, 966, 970, 974, 978, 982,
             985, 988, 990, 993, 995, 996, 998, 999, 999,1000
        };
        uint16_t a = data->angle_deg % 360;
        uint16_t quadrant = a / 90;
        uint16_t idx = a % 90;
        int16_t s = sin_lut[idx];
        int16_t c = sin_lut[90 - idx];
        int16_t sx_v, sy_v;

        switch (quadrant) {
            case 0: sx_v =  s; sy_v = -c; break;
            case 1: sx_v =  c; sy_v =  s; break;
            case 2: sx_v = -s; sy_v =  c; break;
            default: sx_v = -c; sy_v = -s; break;
        }
        {
            uint8_t dx = (uint8_t)(cx + (int8_t)((int32_t)(r - 2) * sx_v / 1000));
            uint8_t dy = (uint8_t)(cy + (int8_t)((int32_t)(r - 2) * sy_v / 1000));
            OLED_Draw_Line(cx, cy, dx, dy);
            OLED_Draw_FillCircle(dx, dy, 2);
        }
    }

    /*=== 右侧: 数据面板 ===*/
    OLED_Draw_Line(rx - 2, 0, rx - 2, SCR_H - 1);

    y = 0;

    /* 频率 (始终显示) */
    OLED_Draw_String(rx, y, "F:");
    u16_right_align(data->freq_hz, buf, 5);
    OLED_Draw_String(rx + FONT_W * 2, y, buf);
    OLED_Draw_String(rx + FONT_W * 7, y, "H");
    y += ROW_H;

    /* 方向 + 角度 */
    OLED_Draw_String(rx, y, show_dir ? dir_names[data->direction] : "---");
    if (show_dir) {
        u16_right_align(data->angle_deg, buf, 3);
        OLED_Draw_String(rx + FONT_W * 4, y, buf);
        OLED_Draw_Char(rx + FONT_W * 7, y, 'd');
    }
    y += ROW_H;

    /* 距离 */
    OLED_Draw_String(rx, y, "D:");
    if (data->distance_cm > 0 && data->distance_cm < 9999) {
        u16_right_align(data->distance_cm, buf, 4);
        OLED_Draw_String(rx + FONT_W * 2, y, buf);
        OLED_Draw_String(rx + FONT_W * 6, y, "cm");
    } else {
        OLED_Draw_String(rx + FONT_W * 2, y, "----");
    }
    y += ROW_H;

    /* 幅值 + 置信度 */
    OLED_Draw_String(rx, y, "A:");
    if (data->amp_valid) {
        u16_right_align(data->amp_mv, buf, 3);
        OLED_Draw_String(rx + FONT_W * 2, y, buf);
    } else {
        OLED_Draw_String(rx + FONT_W * 2, y, "---");
    }
    OLED_Draw_String(rx + FONT_W * 6, y, "C");
    u16_right_align(data->confidence, buf, 2);
    OLED_Draw_String(rx + FONT_W * 7, y, buf);
}

/*========================================
 * 模式3: 频率选择模式 (简洁居中)
 *
 *  FREQ SET              ← 反色模式栏
 *  ─ ─ ─ ─ ─ ─ ─ ─ ─
 *
 *     >> 1000 Hz <<      ← 当前频率居中,反色
 *
 *   1x:Next  2x:OK       ← 操作提示
 *========================================*/
static void draw_freq_set_mode(const OLED_Screen_Data_t* data)
{
    char buf[16];
    uint8_t len, x;

    draw_mode_bar(MODE_FREQ_SET, NULL);

    /* 当前频率居中显示 (反色) */
    len = 0;
    buf[len++] = '>'; buf[len++] = '>'; buf[len++] = ' ';
    len += u16_to_str(data->target_freq_hz, buf + len);
    buf[len++] = ' '; buf[len++] = 'H'; buf[len++] = 'z';
    buf[len++] = ' '; buf[len++] = '<'; buf[len++] = '<';
    buf[len++] = '\0';

    x = (SCR_W - len * FONT_W) / 2;
    OLED_Draw_String_Inv(x, (SCR_H - FONT_H) / 2, buf);

    /* 操作提示 (底部) */
    OLED_Draw_String(MARGIN, SCR_H - FONT_H, "1x:Next  2x:OK");
}

/*========================================
 * 主界面分发
 *========================================*/
void OLED_Draw_MainScreen(const OLED_Screen_Data_t* data)
{
    OLED_Draw_Clear();

    switch (data->mode) {
        case MODE_DETECT:   draw_detect_mode(data); break;
        case MODE_LOCATE:   draw_locate_mode(data); break;
        case MODE_FREQ_SET: draw_freq_set_mode(data); break;
        default:            draw_detect_mode(data); break;
    }
}

/*========================================
 * 浮点数转字符串 (带1位小数, 有符号)
 *========================================*/
static void f16_to_str(float v, char* buf)
{
    int16_t iv;
    uint8_t neg = 0;
    uint8_t len = 0;

    if (v < 0.0f) { neg = 1; v = -v; }

    iv = (int16_t)(v + 0.5f);
    /* 符号 */
    if (neg) buf[len++] = '-';
    /* 整数部分 */
    len += u16_to_str((uint16_t)iv, buf + len);
    /* 小数点 + 1位小数 */
    {
        uint8_t frac = (uint8_t)((int16_t)((v + 0.05f) * 10) % 10);
        buf[len++] = '.';
        buf[len++] = '0' + frac;
    }
    buf[len] = '\0';
}

/*========================================
 * 姿态显示界面
 *
 *  MPU6050 ATTITUDE      ← 标题
 *  ─ ─ ─ ─ ─ ─ ─ ─ ─
 *   R: +12.3   P: -45.6  ← Roll + Pitch
 *   Y: +180.2            ← Yaw
 *  T: 25.3C              ← 温度
 *========================================*/
void OLED_Draw_Attitude(const Attitude_Display_t* att)
{
    char buf[12];

    /* 标题 (反色) */
    {
        uint8_t x;
        for (x = 0; x < 128; x++) {
            fb[x] = 0xFF;
            fb[x + 128] = 0xFF;
        }
    }
    OLED_Draw_String_Inv(0, 0, "MPU6050");
    {
        char time_buf[8];
        uint16_t ms = HAL_GetTick() / 1000;
        time_buf[0] = (ms / 60) / 10 + '0';
        time_buf[1] = (ms / 60) % 10 + '0';
        time_buf[2] = ':';
        time_buf[3] = (ms % 60) / 10 + '0';
        time_buf[4] = (ms % 60) % 10 + '0';
        time_buf[5] = '\0';
        OLED_Draw_String_Inv(128 - 5 * 8, 0, time_buf);
    }
    draw_dashed_hline(16);

    /* Roll + Pitch */
    {
        uint8_t y = FONT_H + MARGIN;
        OLED_Draw_String(MARGIN, y, "R:");
        f16_to_str(att->roll, buf);
        OLED_Draw_String(FONT_W * 2 + MARGIN, y, buf);

        OLED_Draw_String(FONT_W * 8 + MARGIN, y, "P:");
        f16_to_str(att->pitch, buf);
        OLED_Draw_String(FONT_W * 10 + MARGIN, y, buf);
    }

    /* Yaw */
    {
        uint8_t y = FONT_H + MARGIN + ROW_H;
        OLED_Draw_String(MARGIN, y, "Y:");
        f16_to_str(att->yaw, buf);
        OLED_Draw_String(FONT_W * 2 + MARGIN, y, buf);
    }

    /* 温度 */
    {
        uint8_t y = SCR_H - FONT_H;
        OLED_Draw_String(MARGIN, y, "T:");
        f16_to_str(att->temperature, buf);
        {
            uint8_t i = 0;
            while (buf[i]) i++;
            buf[i++] = 'C';
            buf[i] = '\0';
        }
        OLED_Draw_String(FONT_W * 2 + MARGIN, y, buf);
    }
}
