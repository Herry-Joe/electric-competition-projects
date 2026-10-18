#include "stm32f1xx_hal.h"
#include <stdio.h>
#include <string.h>
#include "jdy31.h"
#include "oled_draw.h"

static UART_HandleTypeDef huart1;

/* 重定向 printf 到 USART1（PA9 -> PC 调试串口） */
int _write(int fd, char *ptr, int len)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)ptr, len, 0xFFFF);
    return len;
}

static void SystemClock_Config(void);

int main(void)
{
    static attitude_pkt_t pkt;
    uint32_t last_print = 0;
    uint32_t last_oled = 0;
    uint32_t last_rx = 0;
    uint32_t last_hb = 0;
    uint32_t boot_tick = 0;
    uint32_t pkts = 0;          /* 累计收到包数（诊断用） */
    uint8_t link_ok = 0;

    HAL_Init();
    SystemClock_Config();

    /* USART1: PA9=TX，接 CH340 / ST-Link 虚拟串口 -> 电脑（调试输出） */
    __HAL_RCC_USART1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    GPIO_InitTypeDef g = {0};
    g.Pin = GPIO_PIN_9;
    g.Mode = GPIO_MODE_AF_PP;
    g.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &g);

    huart1.Instance = USART1;
    huart1.Init.BaudRate = 115200;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.Mode = UART_MODE_TX;
    HAL_UART_Init(&huart1);

    /* OLED 初始化（软件 I2C1: PB8/SCL, PB9/SDA；由 PB6 供电、PB7 接地） */
    OLED_Draw_Init();
    OLED_Draw_Clear();
    OLED_Draw_String(0, 0, "BT RX");
    OLED_Draw_String(0, 16, "Waiting...");
    OLED_Draw_String(0, 32, "OLED OK");
    OLED_Draw_Refresh();
    HAL_Delay(500);

    /* JDY-31 蓝牙接收端（USART2: PA2=TX, PA3=RX） */
    JDY31_Init();

    /* 调试：确认 USART2 已起来。JDY-31 是从机，需对端主机连上才开始透传。
     * 若一直 NO LINK，多半是对端不是主机 / 没配对（见 README_JDY31.md）。 */
    printf("BT RX ready (JDY-31 USART2 @%d, slave)\r\n", JDY31_BAUD);

    boot_tick = HAL_GetTick();

    while (1) {
        if (JDY31_DataReady()) {
            JDY31_Read(&pkt);
            last_rx = HAL_GetTick();
            link_ok = 1;
            pkts++;

            /* 限流打印（约 50ms 一次），避免串口刷屏 */
            uint32_t now = HAL_GetTick();
            if (now - last_print >= 50) {
                last_print = now;
                printf("R:%.2f P:%.2f Y:%.2f T:%.2f\r\n",
                       pkt.roll, pkt.pitch, pkt.yaw, pkt.temperature);
            }
        }

        /* 心跳诊断：每 1s 打印 uptime/收包数/链路状态。 */
        uint32_t now = HAL_GetTick();
        if (now - last_hb >= 1000) {
            last_hb = now;
            uint8_t rx_now = (now - last_rx < 1000) ? 1 : 0;
            printf("[up=%lus pkts=%lu link=%s]\r\n",
                   (unsigned long)((now - boot_tick) / 1000),
                   (unsigned long)pkts, rx_now ? "OK" : "NO");
        }

        /* OLED 刷新（约 50ms） */
        if (now - last_oled >= 50) {
            last_oled = now;

            if (link_ok && (now - last_rx < 1000)) {
                /* 有信号：显示姿态 */
                Attitude_Display_t att;
                att.roll = pkt.roll;
                att.pitch = pkt.pitch;
                att.yaw = pkt.yaw;
                att.temperature = pkt.temperature;
                OLED_Draw_Clear();
                OLED_Draw_Attitude(&att);
            } else {
                /* 丢信号：提示 NO LINK */
                OLED_Draw_Clear();
                OLED_Draw_String(0, 24, "NO LINK");
                OLED_Draw_String(0, 40, "WAITING...");
            }
            OLED_Draw_Refresh();
        }
    }
}

static void SystemClock_Config(void)
{
    RCC_OscInitTypeDef osc = {0};
    RCC_ClkInitTypeDef clk = {0};

    osc.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    osc.HSEState       = RCC_HSE_ON;
    osc.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    osc.PLL.PLLState   = RCC_PLL_ON;
    osc.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
    osc.PLL.PLLMUL     = RCC_PLL_MUL9;
    HAL_RCC_OscConfig(&osc);

    clk.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                        | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    clk.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    clk.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    clk.APB1CLKDivider = RCC_HCLK_DIV2;
    clk.APB2CLKDivider = RCC_SYSCLK_DIV1;
    HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_2);
}

void SysTick_Handler(void)
{
    HAL_IncTick();
}
