/**
 * @file    main.c
 * @brief   STM32F103C8T6 + TB6612 + 28BYJ-48 云台 Demo — 主程序 (双轴)
 *
 * 功能概述:
 *   1. 双轴云台控制 (Pan 水平 + Tilt 垂直)
 *   2. 按键控制: 通过 MODE 切换活动轴
 *   3. 串口控制: 上位机发送指令控制指定电机
 *   4. 扫描模式: 自动来回摆动 (双轴)
 *   5. 步进模式: 支持全步进 (大力矩) 和 半步进 (平滑)
 *
 * 硬件连线:
 *   TB6612 #1 (Pan) : PWMA→PA0, AIN1→PA4, AIN2→PA5
 *                      PWMB→PA1, BIN1→PA6, BIN2→PA7
 *   TB6612 #2 (Tilt): AIN1→PB3,  AIN2→PB4
 *                      BIN1→PB5,  BIN2→PB6
 *                      PWMA→PB1,  PWMB→PB7
 *   STBY → PB0 (两个 TB6612 共用)
 *   28BYJ-48: 红(COM)→5V, 橙→AOUT1, 粉→AOUT2, 黄→BOUT1, 蓝→BOUT2
 *
 *   按键: PB12(UP加速) PB13(DOWN减速) PB14(DIR换向) PB15(MODE切换)
 *   MODE 循环: Pan手动 → Tilt手动 → 双轴扫描 → 串口模式
 *
 *   串口: PA9 TX / PA10 RX, 115200bps
 *   命令: p(选Pan) t(选Tilt) c(正转) a(反转)
 *         f(全步) h(半步) s(停) ?(状态)
 *         +N(前进步数) -N(后退步数) rN(旋转N°) wN(扫描±N°)
 *
 *   OLED (SSD1306, 辅助串口调试): I2C1 重映射到 PB8(SCL) / PB9(SDA), 地址 0x3C
 *        屏幕实时显示: 模式 / Pan·Tilt 状态 / 串口选中轴 / 最近命令(RX) / 回执(ACK)
 */

#include "main.h"
#include "stepper.h"
#include "ssd1306.h"
#include "oled_debug.h"
#include <stdlib.h>

/* ========================== 全局外设句柄 ========================== */
TIM_HandleTypeDef  htim2;       /* 步进定时器 (Pan) */
TIM_HandleTypeDef  htim3;       /* 步进定时器 (Tilt) */
UART_HandleTypeDef huart1;      /* 调试串口 */
UART_HandleTypeDef huart2;      /* 蓝牙串口 (JDY-31) */
I2C_HandleTypeDef  hi2c1;       /* OLED I2C 总线 (PB8/PB9) */

/* ========================== 电机实例 ========================== */
static StepperMotor g_motor_pan;    /* 水平电机 (Pan) */
static StepperMotor g_motor_tilt;   /* 垂直电机 (Tilt) */

/* ========================== 引脚配置表 ========================== */
static const StepperPins g_pan_pins = {
    .ain1_port = GPIOA, .ain1_pin = GPIO_PIN_4,
    .ain2_port = GPIOA, .ain2_pin = GPIO_PIN_5,
    .bin1_port = GPIOA, .bin1_pin = GPIO_PIN_6,
    .bin2_port = GPIOA, .bin2_pin = GPIO_PIN_7,
};

static const StepperPins g_tilt_pins = {
    .ain1_port = GPIOB, .ain1_pin = GPIO_PIN_3,
    .ain2_port = GPIOB, .ain2_pin = GPIO_PIN_4,
    .bin1_port = GPIOB, .bin1_pin = GPIO_PIN_5,
    .bin2_port = GPIOB, .bin2_pin = GPIO_PIN_6,
};

/* ========================== 云台控制状态 ========================== */

/** @brief 云台操作模式 */
typedef enum {
    GIMBAL_PAN    = 0,   /* 按键控制水平电机 (Pan) */
    GIMBAL_TILT   = 1,   /* 按键控制垂直电机 (Tilt) */
    GIMBAL_SWEEP  = 2,   /* 双轴自动扫描 */
    GIMBAL_SERIAL = 3,   /* 串口控制 */
    GIMBAL_DANCE  = 4,   /* 舞蹈模式！云台蹦迪 */
} GimbalMode;

static GimbalMode    g_gimbal_mode  = GIMBAL_SERIAL;  /* 上电默认串口模式, 不动电机 */
static StepperMotor *g_active_motor = &g_motor_pan;  /* 按键当前控制的电机 */
static StepperMotor *g_serial_motor = &g_motor_pan;  /* 串口当前选中的电机 */

/* ========================== 舞蹈模式状态 ========================== */
static uint32_t g_dance_phase_start = 0;   /* 当前舞蹈阶段的起始时间 */
static uint8_t  g_dance_phase       = 0;   /* 当前舞蹈阶段序号 */
static uint32_t g_dance_dir_toggle  = 0;   /* 甩头方向切换计时 */

/* ========================== 串口接收缓冲 (ISR → Main loop) ========================== */
#define SERIAL_BUF_SIZE  32
static char            s_serial_buf[SERIAL_BUF_SIZE];
static volatile uint8_t s_serial_len  = 0;
static volatile uint8_t s_cmd_ready   = 0;

/* ========================== 命令反馈 (LED 视觉反馈) ========================== */
static uint32_t g_feedback_until = 0;   /* LED 保持亮到此时间戳 */

/* ========================== OLED 调试显示缓冲 ========================== */
static char g_oled_cmd[20];             /* 最近收到的串口命令 (大写, 供 OLED 显示) */
static char g_oled_ack[20];             /* 最近回执: "OK" / "ERR" */

/* ========================== 系统时钟配置 ========================== */

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState       = RCC_HSE_ON;
    RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    RCC_OscInitStruct.PLL.PLLState   = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL     = RCC_PLL_MUL9;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK  |
                                       RCC_CLOCKTYPE_SYSCLK |
                                       RCC_CLOCKTYPE_PCLK1  |
                                       RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
        Error_Handler();
    }
}

/* ========================== GPIO 初始化 ========================== */

void GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_AFIO_CLK_ENABLE();

    /* [修复] 解除 JTAG 复用，释放 PB3/PB4 作为普通 GPIO (Tilt 电机控制)
     * 用 NOJTAG 而不是 DISABLE: 仅关闭 JTAG, 保留 SWD (PA13/PA14) 调试接口,
     * 否则 DISABLE 会连 SWD 一起禁用, 导致后续无法再烧录/调试。 */
    __HAL_AFIO_REMAP_SWJ_NOJTAG();

    /* ---- PA0, PA1: Pan 电机 PWMA/PWMB (设高 = 全速) ---- */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0 | GPIO_PIN_1, GPIO_PIN_SET);
    GPIO_InitStruct.Pin   = GPIO_PIN_0 | GPIO_PIN_1;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* ---- PA4~PA7: Pan 电机 TB6612 控制信号 ---- */
    GPIO_InitStruct.Pin   = GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* ---- PB0: STBY (两个 TB6612 共用) ---- */
    GPIO_InitStruct.Pin   = GPIO_PIN_0;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);  /* 使能 TB6612 */

    /* ---- PB1: Tilt 电机 PWMA (设高 = 全速) ---- */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
    GPIO_InitStruct.Pin   = GPIO_PIN_1;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* ---- PB3~PB6: Tilt 电机 TB6612 控制信号 ---- */
    GPIO_InitStruct.Pin   = GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* ---- PB7: Tilt 电机 PWMB (设高 = 全速) ---- */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
    GPIO_InitStruct.Pin   = GPIO_PIN_7;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* ---- PB12~PB15: 按键 (上拉输入) ---- */
    GPIO_InitStruct.Pin   = GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
    GPIO_InitStruct.Mode  = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull  = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* ---- PC13: 板载 LED ---- */
    GPIO_InitStruct.Pin   = GPIO_PIN_13;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
    LED_OFF();
}

/* ========================== USART1 初始化 ========================== */

void USART1_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_USART1_CLK_ENABLE();

    GPIO_InitStruct.Pin   = GPIO_PIN_9;
    GPIO_InitStruct.Mode  = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin   = GPIO_PIN_10;
    GPIO_InitStruct.Mode  = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    huart1.Instance          = USART1;
    huart1.Init.BaudRate     = DEBUG_USART_BAUD;
    huart1.Init.WordLength   = UART_WORDLENGTH_8B;
    huart1.Init.StopBits     = UART_STOPBITS_1;
    huart1.Init.Parity       = UART_PARITY_NONE;
    huart1.Init.Mode         = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&huart1);

    /* [修复] 启用 RXNE 中断, 否则 ISR 不会因收到数据而触发 */
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);
}

/* ========================== USART2 初始化 (JDY-31 蓝牙) ========================== */

void USART2_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_USART2_CLK_ENABLE();

    /* PA2: USART2 TX */
    GPIO_InitStruct.Pin   = GPIO_PIN_2;
    GPIO_InitStruct.Mode  = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* PA3: USART2 RX */
    GPIO_InitStruct.Pin   = GPIO_PIN_3;
    GPIO_InitStruct.Mode  = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    huart2.Instance          = USART2;
    huart2.Init.BaudRate     = BT_USART_BAUD;      /* 9600 (JDY-31 默认) */
    huart2.Init.WordLength   = UART_WORDLENGTH_8B;
    huart2.Init.StopBits     = UART_STOPBITS_1;
    huart2.Init.Parity       = UART_PARITY_NONE;
    huart2.Init.Mode         = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&huart2);

    /* [修复] 启用 RXNE 中断, 否则蓝牙发来的命令不会被处理 */
    __HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE);
}

/* ========================== I2C1 初始化 (OLED PB8/PB9) ========================== */

void I2C1_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_I2C1_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_AFIO_CLK_ENABLE();

    /* I2C1 默认在 PB6/PB7, 但本工程 PB6/PB7 已被 Tilt 电机占用,
     * 重映射到 PB8(SCL) / PB9(SDA) */
    __HAL_AFIO_REMAP_I2C1_ENABLE();

    /* PB8/PB9: I2C 开漏复用 + 上拉 (OLED 模块通常自带上拉, 这里再加一层保险) */
    GPIO_InitStruct.Pin   = GPIO_PIN_8 | GPIO_PIN_9;
    GPIO_InitStruct.Mode  = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull  = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    hi2c1.Instance          = I2C1;
    hi2c1.Init.ClockSpeed   = OLED_I2C_SPEED;        /* 400kHz */
    hi2c1.Init.DutyCycle    = I2C_DUTYCYCLE_2;
    hi2c1.Init.OwnAddress1  = 0;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2  = 0;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
        Error_Handler();
    }
}

/* ========================== TIM2 初始化 (Pan 电机步进节拍) ========================== */

void TIM2_Init(void)
{
    __HAL_RCC_TIM2_CLK_ENABLE();

    htim2.Instance               = TIM2;
    htim2.Init.Prescaler         = 72 - 1;
    htim2.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim2.Init.Period            = STEPPER_DEFAULT_DELAY - 1;
    htim2.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    HAL_TIM_Base_Init(&htim2);

    __HAL_TIM_ENABLE_IT(&htim2, TIM_IT_UPDATE);
    HAL_TIM_Base_Start(&htim2);

    HAL_NVIC_SetPriority(TIM2_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(TIM2_IRQn);
}

/* ========================== TIM3 初始化 (Tilt 电机步进节拍) ========================== */

void TIM3_Init(void)
{
    __HAL_RCC_TIM3_CLK_ENABLE();

    htim3.Instance               = TIM3;
    htim3.Init.Prescaler         = 72 - 1;
    htim3.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim3.Init.Period            = STEPPER_DEFAULT_DELAY - 1;
    htim3.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    HAL_TIM_Base_Init(&htim3);

    __HAL_TIM_ENABLE_IT(&htim3, TIM_IT_UPDATE);
    HAL_TIM_Base_Start(&htim3);

    HAL_NVIC_SetPriority(TIM3_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(TIM3_IRQn);
}

/* ========================== NVIC 配置 ========================== */

static void NVIC_Config(void)
{
    /* USART1 中断优先级 (最高) */
    HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);

    /* USART2 中断 (蓝牙) */
    HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);

    /* SysTick 优先级 (最低) */
    HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);
}

/* ========================== printf 重定向 ========================== */

#ifdef __GNUC__
int _write(int fd, char *ptr, int len)
{
    (void)fd;
    /* 同时输出到调试串口(USART1)和蓝牙(USART2) */
    HAL_UART_Transmit(&huart1, (uint8_t *)ptr, (uint16_t)len, 10);
    if (HAL_IS_BIT_SET(huart2.Instance->CR1, USART_CR1_UE)) {
        HAL_UART_Transmit(&huart2, (uint8_t *)ptr, (uint16_t)len, 10);
    }
    return len;
}

/* stub: newlib-nano 不会实际调用，仅避免链接警告 */
int _read(int fd, char *ptr, int len)   { (void)fd; (void)ptr; (void)len; return 0; }
int _lseek(int fd, int ptr, int dir)    { (void)fd; (void)ptr; (void)dir; return 0; }
int _close(int fd)                      { (void)fd; return -1; }
int _fstat(int fd, void *buf)           { (void)fd; (void)buf; return -1; }
int _isatty(int fd)                     { (void)fd; return 1; }
#else
int fputc(int ch, FILE *f)
{
    (void)f;
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    return ch;
}
#endif

/* ========================== 串口命令执行 (在主循环调用, 非 ISR) ========================== */

static void Serial_Execute(void)
{
    s_serial_buf[s_serial_len] = '\0';

    /* 跳过换行/空格 */
    char *p = s_serial_buf;
    while (*p == '\r' || *p == '\n' || *p == ' ') p++;
    if (*p == '\0') return;

    /* 拷贝到 OLED 显示缓冲 (转大写, 单色屏只支持 ASCII) */
    {
        char *d = g_oled_cmd; const char *s = p; int i = 0;
        while (*s && i < (int)sizeof(g_oled_cmd) - 1) {
            char c = *s;
            if (c >= 'a' && c <= 'z') c -= 32;
            *d++ = c; s++; i++;
        }
        *d = '\0';
    }
    snprintf(g_oled_ack, sizeof(g_oled_ack), "OK");

    /* ---- 串口接管 + 视觉反馈 ----
     * 任何串口命令都先把云台切到 SERIAL 模式，避免被 Pan/Sweep/Dance 的主循环逻辑覆盖；
     * 同时让板载 LED 亮 150ms，给出明确的"已收到"视觉反馈。 */
    g_gimbal_mode = GIMBAL_SERIAL;
    g_feedback_until = HAL_GetTick() + 150;
    LED_ON();
    printf("\r\n> %s\r\n", s_serial_buf);   /* 回显用户发出的命令 */

    /* ---- 单字符命令 (不区分大小写) ---- */
    switch (*p | 0x20) {
        case 'p':   /* 选择 Pan 电机 */
            g_serial_motor = &g_motor_pan;
            printf("# OK → 选中 Pan 电机 (水平)\r\n");
            return;
        case 't':   /* 选择 Tilt 电机 */
            g_serial_motor = &g_motor_tilt;
            printf("# OK → 选中 Tilt 电机 (垂直)\r\n");
            return;
        case 'f':
            Stepper_SetMode(g_serial_motor, STEP_MODE_FULL);
            printf("# OK 全步步进 (大力矩)\r\n");
            return;
        case 'h':
            Stepper_SetMode(g_serial_motor, STEP_MODE_HALF);
            printf("# OK 半步步进 (平滑)\r\n");
            return;
        case 'c':   /* 连续正转 */
            Stepper_SetDirection(g_serial_motor, DIR_CW);
            Stepper_RunContinuously(g_serial_motor);
            printf("# OK %s 连续正转 (CW)\r\n", g_serial_motor->label);
            return;
        case 'a':   /* 连续反转 */
            Stepper_SetDirection(g_serial_motor, DIR_CCW);
            Stepper_RunContinuously(g_serial_motor);
            printf("# OK %s 连续反转 (CCW)\r\n", g_serial_motor->label);
            return;
        case 's':   /* 停止 */
            Stepper_Stop(g_serial_motor);
            printf("# OK %s 停止\r\n", g_serial_motor->label);
            return;
        case 'd':   /* 进入舞蹈模式 */
            Stepper_RunContinuously(&g_motor_pan);
            Stepper_RunContinuously(&g_motor_tilt);
            g_gimbal_mode = GIMBAL_DANCE;
            g_dance_phase_start = 0;
            printf("# OK 进入舞蹈模式 🎵\r\n");
            return;
        case '?':
            Stepper_PrintStatus(&g_motor_pan);
            Stepper_PrintStatus(&g_motor_tilt);
            printf("串口当前选中: %s\r\n", g_serial_motor->label);
            printf("# OK 状态已上报 (上方)\r\n");
            return;
        default:
            break;
    }

    /* ---- 带数字参数的命令: +N, -N, rN, wN, vN ---- */
    char type = *p;
    char *num_str = p + 1;
    int32_t num = atoi(num_str);

    switch (type) {
        case '+':
            Stepper_SetDirection(g_serial_motor, DIR_CW);
            Stepper_StepBy(g_serial_motor, (uint32_t)num);
            printf("# OK %s 前进 %ld 步\r\n", g_serial_motor->label, (long)num);
            break;
        case '-':
            Stepper_SetDirection(g_serial_motor, DIR_CCW);
            Stepper_StepBy(g_serial_motor, (uint32_t)num);
            printf("# OK %s 后退 %ld 步\r\n", g_serial_motor->label, (long)num);
            break;
        case 'r':
        case 'R':
            Stepper_RotateDegrees(g_serial_motor, (float)num);
            printf("# OK %s 旋转 %ld°\r\n", g_serial_motor->label, (long)num);
            break;
        case 'w':
            Stepper_SweepEnable(g_serial_motor, (uint32_t)num);
            printf("# OK %s 扫描 ±%ld°\r\n", g_serial_motor->label, (long)(num / 2));
            break;
        case 'v':
        case 'V': {
            uint32_t d = (uint32_t)num;
            if (d < STEPPER_MIN_DELAY_US) d = STEPPER_MIN_DELAY_US;
            if (d > STEPPER_MAX_DELAY_US) d = STEPPER_MAX_DELAY_US;
            Stepper_SetSpeed(g_serial_motor, d);
            printf("# OK 速度 %lu µs/步 (越小越快)\r\n", (unsigned long)d);
            break;
        }
        default:
            printf("# ERR 未知命令 '%c' → 输入 ? 查看帮助\r\n", *p);
            snprintf(g_oled_ack, sizeof(g_oled_ack), "ERR");
            break;
    }
}

/* ========================== 云台按键扫描 ========================== */

static void Gimbal_KeyScan(void)
{
    static uint32_t last_tick = 0;
    uint32_t now = HAL_GetTick();

    if ((now - last_tick) < 50) return;  /* 50ms 消抖 */
    last_tick = now;

    /* ---- KEY_MODE: 循环切换云台模式 (Pan → Tilt → Sweep → Serial) ---- */
    if (KEY_MODE_READ() == GPIO_PIN_RESET) {
        Stepper_Stop(&g_motor_pan);
        Stepper_Stop(&g_motor_tilt);

        g_gimbal_mode = (GimbalMode)((g_gimbal_mode + 1) % 5);

        switch (g_gimbal_mode) {
            case GIMBAL_PAN:
                g_active_motor = &g_motor_pan;
                printf("\r\n[云台] → Pan 手动模式 (控制水平电机)\r\n");
                printf("  UP=加速  DOWN=减速  DIR=换向\r\n");
                Stepper_RunContinuously(&g_motor_pan);
                break;
            case GIMBAL_TILT:
                g_active_motor = &g_motor_tilt;
                printf("\r\n[云台] → Tilt 手动模式 (控制垂直电机)\r\n");
                printf("  UP=加速  DOWN=减速  DIR=换向\r\n");
                Stepper_RunContinuously(&g_motor_tilt);
                break;
            case GIMBAL_SWEEP:
                printf("\r\n[云台] → 双轴扫描模式\r\n");
                Stepper_SweepEnable(&g_motor_pan, 180);   /* ±90° */
                Stepper_SweepEnable(&g_motor_tilt, 180);
                break;
            case GIMBAL_SERIAL:
                printf("\r\n[云台] → 串口控制模式\r\n");
                printf("  p(选Pan) t(选Tilt) f(全步) h(半步)\r\n");
                printf("  c(正转) a(反转) s(停) ?(状态)\r\n");
                printf("  +N(步进) -N(后退) rN(旋转°) wN(扫描±°)\r\n");
                break;
            case GIMBAL_DANCE:
                printf("\r\n🎵 [云台] → 舞蹈模式！云台开始蹦迪！\r\n");
                Stepper_RunContinuously(&g_motor_pan);
                Stepper_RunContinuously(&g_motor_tilt);
                g_dance_phase_start = 0;  /* 重头开始跳 */
                break;
        }
        return;
    }

    /* 扫描模式: UP/DOWN 同时调双轴速度 */
    if (g_gimbal_mode == GIMBAL_SWEEP) {
        if (KEY_UP_READ() == GPIO_PIN_RESET) {
            uint32_t s = g_motor_pan.step_delay_us;
            if (s > STEPPER_MIN_DELAY_US) {
                Stepper_SetSpeed(&g_motor_pan, s - 200);
                Stepper_SetSpeed(&g_motor_tilt, s - 200);
            }
            return;
        }
        if (KEY_DOWN_READ() == GPIO_PIN_RESET) {
            uint32_t s = g_motor_pan.step_delay_us;
            if (s < STEPPER_MAX_DELAY_US) {
                Stepper_SetSpeed(&g_motor_pan, s + 200);
                Stepper_SetSpeed(&g_motor_tilt, s + 200);
            }
            return;
        }
        return;
    }

    /* 舞蹈模式: UP/DOWN 调整体速度 */
    if (g_gimbal_mode == GIMBAL_DANCE) {
        if (KEY_UP_READ() == GPIO_PIN_RESET) {
            uint32_t s = g_motor_pan.step_delay_us;
            if (s > STEPPER_MIN_DELAY_US) {
                Stepper_SetSpeed(&g_motor_pan, s - 200);
                Stepper_SetSpeed(&g_motor_tilt, s - 200);
            }
            return;
        }
        if (KEY_DOWN_READ() == GPIO_PIN_RESET) {
            uint32_t s = g_motor_pan.step_delay_us;
            if (s < STEPPER_MAX_DELAY_US) {
                Stepper_SetSpeed(&g_motor_pan, s + 200);
                Stepper_SetSpeed(&g_motor_tilt, s + 200);
            }
            return;
        }
        return;
    }

    /* 串口模式: 按键无效 */
    if (g_gimbal_mode == GIMBAL_SERIAL) {
        return;
    }

    /* ====== Pan / Tilt 手动模式 ====== */

    /* KEY_UP: 加速 */
    if (KEY_UP_READ() == GPIO_PIN_RESET) {
        uint32_t new_delay = g_active_motor->step_delay_us;
        if (new_delay > STEPPER_MIN_DELAY_US) {
            new_delay -= 200;
            Stepper_SetSpeed(g_active_motor, new_delay);
        }
        if (Stepper_GetState(g_active_motor) == STATE_IDLE) {
            Stepper_RunContinuously(g_active_motor);
        }
        return;
    }

    /* KEY_DOWN: 减速 */
    if (KEY_DOWN_READ() == GPIO_PIN_RESET) {
        uint32_t new_delay = g_active_motor->step_delay_us;
        if (new_delay < STEPPER_MAX_DELAY_US) {
            new_delay += 200;
            Stepper_SetSpeed(g_active_motor, new_delay);
        }
        if (Stepper_GetState(g_active_motor) == STATE_IDLE) {
            Stepper_RunContinuously(g_active_motor);
        }
        return;
    }

    /* KEY_DIR: 换向 */
    if (KEY_DIR_READ() == GPIO_PIN_RESET) {
        Stepper_SetDirection(g_active_motor,
            (g_active_motor->dir == DIR_CW) ? DIR_CCW : DIR_CW);
        if (Stepper_GetState(g_active_motor) == STATE_IDLE) {
            Stepper_RunContinuously(g_active_motor);
        }
        return;
    }
}

/* ========================== 舞蹈模式 ========================== */

/**
 * @brief 云台跳舞！按时间编排的舞蹈动作序列
 * @note  在 GIMBAL_DANCE 模式下每轮主循环调用
 */
static void Gimbal_Dance(void)
{
    uint32_t now = HAL_GetTick();

    /* 首次进入或重新进入: 重置舞蹈阶段 */
    if (g_dance_phase_start == 0) {
        g_dance_phase_start = now;
        g_dance_phase = 0;
        g_dance_dir_toggle = 0;
        printf("\r\n💃 云台开始跳舞！\r\n");
        return;
    }

    uint32_t elapsed = now - g_dance_phase_start;

    switch (g_dance_phase) {

    /* ── Phase 0: 热身 — 慢速同向 2秒 ── */
    case 0:
        Stepper_SetSpeed(&g_motor_pan, 4000);
        Stepper_SetSpeed(&g_motor_tilt, 4000);
        Stepper_SetDirection(&g_motor_pan, DIR_CW);
        Stepper_SetDirection(&g_motor_tilt, DIR_CW);
        if (elapsed > 2000) {
            g_dance_phase = 1;
            g_dance_phase_start = now;
            g_dance_dir_toggle = 0;
        }
        break;

    /* ── Phase 1: 甩头 — 快速方向切换 3秒 ── */
    case 1:
        Stepper_SetSpeed(&g_motor_pan, 1200);
        Stepper_SetSpeed(&g_motor_tilt, 1500);
        if ((now - g_dance_dir_toggle) > 350) {
            g_dance_dir_toggle = now;
            Stepper_SetDirection(&g_motor_pan,
                (g_motor_pan.dir == DIR_CW) ? DIR_CCW : DIR_CW);
            Stepper_SetDirection(&g_motor_tilt,
                (g_motor_tilt.dir == DIR_CW) ? DIR_CCW : DIR_CW);
        }
        if (elapsed > 3000) {
            g_dance_phase = 2;
            g_dance_phase_start = now;
            g_dance_dir_toggle = 0;
        }
        break;

    /* ── Phase 2: 旋转 — Pan 狂转, Tilt 扫描头上扬 3秒 ── */
    case 2:
        Stepper_SetSpeed(&g_motor_pan, 1000);
        Stepper_SetDirection(&g_motor_pan, DIR_CW);
        if (Stepper_GetState(&g_motor_tilt) != STATE_RUNNING ||
            g_motor_tilt.work_mode != WORK_MODE_SWEEP) {
            Stepper_Stop(&g_motor_tilt);
            Stepper_SweepEnable(&g_motor_tilt, 160);
            Stepper_SetSpeed(&g_motor_tilt, 2500);
        }
        if (elapsed > 3000) {
            g_dance_phase = 3;
            g_dance_phase_start = now;
        }
        break;

    /* ── Phase 3: 反旋转 — Pan 反向, Tilt 低头 3秒 ── */
    case 3:
        Stepper_SetSpeed(&g_motor_pan, 1000);
        Stepper_SetDirection(&g_motor_pan, DIR_CCW);
        if (Stepper_GetState(&g_motor_tilt) != STATE_RUNNING ||
            g_motor_tilt.work_mode != WORK_MODE_SWEEP) {
            Stepper_Stop(&g_motor_tilt);
            Stepper_SweepEnable(&g_motor_tilt, 160);
            Stepper_SetSpeed(&g_motor_tilt, 2500);
        }
        if (elapsed > 3000) {
            g_dance_phase = 4;
            g_dance_phase_start = now;
            g_dance_dir_toggle = 0;
        }
        break;

    /* ── Phase 4: 加速冲刺 — 逐渐加速后变向 4秒 ── */
    case 4: {
        float progress = (float)elapsed / 4000.0f;
        uint32_t speed = 2500 - (uint32_t)(progress * 1500);
        if (speed < 1000) speed = 1000;
        Stepper_SetSpeed(&g_motor_pan, speed);
        /* 恢复连续模式 (如果之前是扫描) */
        if (g_motor_tilt.work_mode == WORK_MODE_SWEEP) {
            Stepper_Stop(&g_motor_tilt);
            Stepper_RunContinuously(&g_motor_tilt);
        }
        Stepper_SetSpeed(&g_motor_tilt, speed);
        /* 每 500ms 换向 */
        if ((now - g_dance_dir_toggle) > 500) {
            g_dance_dir_toggle = now;
            Stepper_SetDirection(&g_motor_pan,
                (g_motor_pan.dir == DIR_CW) ? DIR_CCW : DIR_CW);
        }
        /* Tilt 每 1000ms 换向 (慢于 Pan, 有交错感) */
        if ((now - g_dance_dir_toggle) > 1000 || g_dance_dir_toggle == 0) {
            if ((now - g_dance_dir_toggle) > 1000) {
                Stepper_SetDirection(&g_motor_tilt,
                    (g_motor_tilt.dir == DIR_CW) ? DIR_CCW : DIR_CW);
            }
        }
        if (elapsed > 4000) {
            g_dance_phase = 5;
            g_dance_phase_start = now;
        }
        break;
    }

    /* ── Phase 5: 缓步收尾 — 放慢速 2秒 ── */
    case 5:
        Stepper_SetSpeed(&g_motor_pan, 5000);
        Stepper_SetSpeed(&g_motor_tilt, 5000);
        Stepper_SetDirection(&g_motor_pan, DIR_CW);
        Stepper_SetDirection(&g_motor_tilt, DIR_CCW);
        if (elapsed > 2000) {
            /* 舞蹈结束, 回到 Pan 手动模式 */
            printf("\r\n🎉 舞蹈表演结束！\r\n");
            g_gimbal_mode = GIMBAL_PAN;
            g_active_motor = &g_motor_pan;
            g_dance_phase_start = 0;
            g_dance_phase = 0;
            printf("[云台] → Pan 手动模式\r\n");
            printf("  UP=加速  DOWN=减速  DIR=换向  MODE=切换\r\n");
        }
        break;

    default:
        g_dance_phase = 0;
        g_dance_phase_start = 0;
        break;
    }
}

/* ========================== OLED 调试刷新 ========================== */

/** @brief 把单个电机状态格式化成一行 ASCII (供 OLED 显示) */
static void OLED_MotorLine(const StepperMotor *m, char *out, size_t sz)
{
    static const char *st[] = {"IDLE", "RUN", "STEP", "ANG"};
    static const char *dr[] = {"CW",   "CCW"};
    static const char *wm[] = {"MAN",  "SWP", "SER"};
    snprintf(out, sz, "%s:%s %s %s %.1fms",
             m->label, st[m->state], dr[m->dir], wm[m->work_mode],
             (double)m->step_delay_us / 1000.0);
}

/** @brief 收集当前云台状态并刷新到 OLED (无 OLED 时直接返回) */
static void OLED_Refresh(void)
{
    if (!OLED_Debug_IsReady()) return;

    static const char *mode_names[5] = {
        "PAN", "TILT", "SWEEP", "SERIAL", "DANCE"
    };

    OledDebugInfo info;
    info.mode = mode_names[g_gimbal_mode];
    OLED_MotorLine(&g_motor_pan,  info.pan_line,  sizeof(info.pan_line));
    OLED_MotorLine(&g_motor_tilt, info.tilt_line, sizeof(info.tilt_line));
    snprintf(info.serial_line, sizeof(info.serial_line), "SEL:%s", g_serial_motor->label);
    snprintf(info.cmd_line,    sizeof(info.cmd_line),    "RX:%s", g_oled_cmd);
    snprintf(info.ack_line,    sizeof(info.ack_line),    "ACK:%s", g_oled_ack);
    snprintf(info.time_line,   sizeof(info.time_line),   "UPT:%.1fs",
             (double)HAL_GetTick() / 1000.0);

    OLED_Debug_Update(&info);
}

/* ========================== 主函数 ========================== */

int main(void)
{
    HAL_Init();
    SystemClock_Config();

    GPIO_Init();
    USART1_Init();
    USART2_Init();
    I2C1_Init();
    NVIC_Config();

    /* OLED 初始化 (设备不存在时自动跳过, 不影响其余功能) */
    {
        uint8_t oled_ok = OLED_Debug_Init();
        if (oled_ok) {
            printf("[系统] OLED 已就绪 (I2C1 PB8/PB9)\r\n");
        } else {
            printf("[系统] ⚠ OLED 未检测到 (已尝试 0x3C 和 0x3D, I2C 总线已复位)\r\n");
            printf("      请确认:\r\n");
            printf("      ① VCC→3.3V (少数模块要 5V, 可尝试改接 5V)\r\n");
            printf("      ② GND→GND\r\n");
            printf("      ③ SCL→PB8  SDA→PB9\r\n");
            printf("      ④ 模块 SCK/SDA 引脚接了 4.7kΩ 上拉到 3.3V? (部分模块不自带上拉)\r\n");
        }
    }

    /* 初始化两个步进电机 (Pan + Tilt) */
    Stepper_Init(&g_motor_pan,  &g_pan_pins,  &htim2, "Pan");
    Stepper_Init(&g_motor_tilt, &g_tilt_pins, &htim3, "Tilt");

    /* 启动两个步进定时器 */
    TIM2_Init();
    TIM3_Init();

    /* 用户要求: 上电电机不动, 收到串口/蓝牙命令才动 */
    /* (电机初始化后处于 idle 状态, step 0 保持力矩) */

    printf("\r\n========================================\r\n");
    printf("  28BYJ-48 双轴云台 Demo 已就绪\r\n");
    printf("  STM32F103C8T6 @ 72MHz\r\n");
    printf("  电机: Pan(水平) + Tilt(垂直)\r\n");
    printf("========================================\r\n\r\n");
    printf("[云台] → 串口控制模式 (上电空闲, 收到命令才动)\r\n");
    printf("  p(选Pan) t(选Tilt) c(正转) a(反转) s(停) ?(状态)\r\n");
    printf("  +N(步进) -N(后退) rN(旋转°) wN(扫描±°) d(舞蹈)\r\n\r\n");

    while (1) {
        /* [修复] 串口命令在主循环处理，避免 printf 在 ISR 中阻塞 */
        if (s_cmd_ready) {
            s_cmd_ready = 0;
            Serial_Execute();
            s_serial_len = 0;
        }

        /* 按键扫描 */
        Gimbal_KeyScan();

        /* 舞蹈模式: 编排动作 */
        if (g_gimbal_mode == GIMBAL_DANCE) {
            Gimbal_Dance();
        }

        /* OLED 调试显示: 每 200ms 刷新一次 (I2C 不宜过快) */
        static uint32_t oled_tick = 0;
        if (HAL_GetTick() - oled_tick >= 200) {
            oled_tick = HAL_GetTick();
            OLED_Refresh();
        }

        /* LED 反馈: 串口命令到达时亮 150ms；平时双轴空闲则 500ms 慢闪 */
        static uint32_t led_tick = 0;
        if (HAL_GetTick() < g_feedback_until) {
            LED_ON();
        } else if (HAL_GetTick() - led_tick > 500) {
            led_tick = HAL_GetTick();
            if (Stepper_GetState(&g_motor_pan) == STATE_IDLE &&
                Stepper_GetState(&g_motor_tilt) == STATE_IDLE) {
                LED_TOG();
            }
        }
    }
}

/* ========================== 中断服务函数 ========================== */

/**
 * @brief TIM2 中断 — Pan 电机步进节拍
 */
void TIM2_IRQHandler(void)
{
    if (__HAL_TIM_GET_FLAG(&htim2, TIM_FLAG_UPDATE) != RESET) {
        if (__HAL_TIM_GET_IT_SOURCE(&htim2, TIM_IT_UPDATE) != RESET) {
            __HAL_TIM_CLEAR_IT(&htim2, TIM_IT_UPDATE);
            Stepper_TIM_IRQHandler(&g_motor_pan);
        }
    }
}

/**
 * @brief TIM3 中断 — Tilt 电机步进节拍
 */
void TIM3_IRQHandler(void)
{
    if (__HAL_TIM_GET_FLAG(&htim3, TIM_FLAG_UPDATE) != RESET) {
        if (__HAL_TIM_GET_IT_SOURCE(&htim3, TIM_IT_UPDATE) != RESET) {
            __HAL_TIM_CLEAR_IT(&htim3, TIM_IT_UPDATE);
            Stepper_TIM_IRQHandler(&g_motor_tilt);
        }
    }
}

/**
 * @brief USART1 中断 — [修复] 只缓冲字节，不调用 printf
 *        命令解析在主循环 Serial_Execute() 中执行
 */
void USART1_IRQHandler(void)
{
    if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_RXNE) != RESET) {
        uint8_t byte = (uint8_t)(huart1.Instance->DR & 0xFF);

        /* 回显 (可选) */
        HAL_UART_Transmit(&huart1, &byte, 1, 0);

        /* 缓冲接收数据 */
        if (s_serial_len < SERIAL_BUF_SIZE - 1) {
            s_serial_buf[s_serial_len++] = (char)byte;
        }
        if (byte == '\n' || byte == '\r') {
            s_cmd_ready = 1;
        } else if (s_serial_len >= SERIAL_BUF_SIZE - 1) {
            /* 缓冲区满，丢弃后续字符，强制触发命令处理 */
            s_cmd_ready = 1;
        }
    }

    /* 清除溢出错误 */
    if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_ORE) != RESET) {
        __HAL_UART_CLEAR_OREFLAG(&huart1);
    }
}

/**
 * @brief USART2 中断 (JDY-31 蓝牙) — 同样只缓冲字节，命令在主循环解析
 *        不回显 (手机 App 不需要回显)
 */
void USART2_IRQHandler(void)
{
    if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_RXNE) != RESET) {
        uint8_t byte = (uint8_t)(huart2.Instance->DR & 0xFF);

        /* 缓冲接收数据 (与 USART1 共用缓冲区) */
        if (s_serial_len < SERIAL_BUF_SIZE - 1) {
            s_serial_buf[s_serial_len++] = (char)byte;
        }
        if (byte == '\n' || byte == '\r') {
            s_cmd_ready = 1;
        } else if (s_serial_len >= SERIAL_BUF_SIZE - 1) {
            s_cmd_ready = 1;
        }
    }

    /* 清除溢出错误 */
    if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_ORE) != RESET) {
        __HAL_UART_CLEAR_OREFLAG(&huart2);
    }
}

/**
 * @brief SysTick 中断 — HAL 库时基
 */
void SysTick_Handler(void)
{
    HAL_IncTick();
}

/**
 * @brief HardFault 异常
 */
void HardFault_Handler(void)
{
    printf("\r\n!!! HARD FAULT !!!\r\n");
    while (1) {
        LED_TOG();
        for (volatile uint32_t i = 0; i < 500000; i++);
    }
}

/**
 * @brief 错误处理
 */
void Error_Handler(void)
{
    printf("\r\n!!! ERROR !!!\r\n");
    __disable_irq();
    while (1) {
        LED_TOG();
        for (volatile uint32_t i = 0; i < 1000000; i++);
    }
}

