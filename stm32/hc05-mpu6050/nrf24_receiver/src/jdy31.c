#include "jdy31.h"
#include <string.h>

/* USART2 句柄（PA2=TX -> JDY31 RXD, PA3=RX <- JDY31 TXD）。
 * 与调试用的 USART1（PA9）互不影响。 */
static UART_HandleTypeDef huart2;

/* ---- RX 环形缓冲（由 USART2 接收中断持续填充）---- */
#define RX_RING_SIZE  128
static uint8_t  g_rx_ring[RX_RING_SIZE];
static volatile uint16_t g_rx_head = 0;
static volatile uint16_t g_rx_tail = 0;
static uint8_t  g_rx_byte;          /* 单字节接收缓冲（配合 HAL_UART_Receive_IT） */

/* ---- 发送计数 / 最近解出的一帧 ---- */
static uint32_t g_sent = 0;
static attitude_pkt_t g_decoded;
static uint8_t  g_have_pkt = 0;

/* ============================================================
 * 中断相关（USART2 接收 + 全局中断）
 * ============================================================ */
static void ring_put(uint8_t b)
{
    uint16_t next = (g_rx_head + 1) % RX_RING_SIZE;
    if (next != g_rx_tail) {          /* 满了就丢弃最旧字节，保证不溢出 */
        g_rx_ring[g_rx_head] = b;
        g_rx_head = next;
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2) {
        ring_put(g_rx_byte);
        HAL_UART_Receive_IT(&huart2, &g_rx_byte, 1);   /* 重新开启接收 */
    }
}

void USART2_IRQHandler(void)
{
    HAL_UART_IRQHandler(&huart2);
}

/* ============================================================
 * 初始化
 * ============================================================ */
void JDY31_Init(void)
{
    __HAL_RCC_USART2_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitTypeDef g = {0};
    /* PA2 = TX 复用推挽；PA3 = RX 浮空输入 */
    g.Pin       = GPIO_PIN_2;
    g.Mode      = GPIO_MODE_AF_PP;
    g.Speed     = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &g);

    g.Pin       = GPIO_PIN_3;
    g.Mode      = GPIO_MODE_INPUT;
    g.Pull      = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &g);

    huart2.Instance = USART2;
    huart2.Init.BaudRate   = JDY31_BAUD;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits   = UART_STOPBITS_1;
    huart2.Init.Parity     = UART_PARITY_NONE;
    huart2.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
    huart2.Init.Mode       = UART_MODE_TX_RX;
    HAL_UART_Init(&huart2);

    /* 使能 USART2 全局中断，并启动首次接收 */
    HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
    HAL_UART_Receive_IT(&huart2, &g_rx_byte, 1);
}

/* ============================================================
 * 发送
 * ============================================================ */
void JDY31_Send(const attitude_pkt_t *pkt)
{
    uint8_t frame[JDY31_FRAME_LEN];
    frame[0] = JDY31_SYNC0;
    frame[1] = JDY31_SYNC1;
    memcpy(&frame[2], pkt, JDY31_PAYLOAD_LEN);
    HAL_UART_Transmit(&huart2, frame, JDY31_FRAME_LEN, 100);
    g_sent++;
}

uint32_t JDY31_SentCount(void)
{
    return g_sent;
}

/* ============================================================
 * 接收：扫描环形缓冲，找 0xAA 0x55 同步头并取出 16 字节载荷
 * ============================================================ */
uint8_t JDY31_DataReady(void)
{
    if (g_have_pkt) return 1;

    while (g_rx_tail != g_rx_head) {
        /* 只看当前字节，不消费 */
        uint8_t b = g_rx_ring[g_rx_tail];
        if (b != JDY31_SYNC0) {
            g_rx_tail = (g_rx_tail + 1) % RX_RING_SIZE;   /* 非同步字节，跳过 */
            continue;
        }

        /* 当前是 0xAA：先看缓冲区里有没有足够拼出一整帧 */
        uint16_t avail = (g_rx_head >= g_rx_tail)
                       ? (g_rx_head - g_rx_tail)
                       : (RX_RING_SIZE - g_rx_tail + g_rx_head);
        if (avail < JDY31_FRAME_LEN) {
            return 0;   /* 数据还不够，保留 0xAA 等待后续字节（不消费） */
        }

        /* 有整帧空间了，确认第二字节是不是 0x55 */
        uint16_t t1 = (g_rx_tail + 1) % RX_RING_SIZE;
        if (g_rx_ring[t1] != JDY31_SYNC1) {
            g_rx_tail = (g_rx_tail + 1) % RX_RING_SIZE;   /* 假同步头，跳过这个 0xAA */
            continue;
        }

        /* 真同步头：消费 2 字节同步头 + 取出 16 字节载荷 */
        g_rx_tail = (g_rx_tail + 2) % RX_RING_SIZE;
        uint8_t buf[JDY31_PAYLOAD_LEN];
        for (uint8_t i = 0; i < JDY31_PAYLOAD_LEN; i++) {
            buf[i] = g_rx_ring[g_rx_tail];
            g_rx_tail = (g_rx_tail + 1) % RX_RING_SIZE;
        }
        memcpy(&g_decoded, buf, JDY31_PAYLOAD_LEN);
        g_have_pkt = 1;
        return 1;
    }
    return 0;
}

void JDY31_Read(attitude_pkt_t *pkt)
{
    *pkt = g_decoded;
    g_have_pkt = 0;
}

/* ============================================================
 * 透传字符串 / AT 指令（调试用，未连接时 AT 才生效）
 * ============================================================ */
void JDY31_SendString(const char *s)
{
    if (s) HAL_UART_Transmit(&huart2, (uint8_t *)s, strlen(s), 100);
}

void JDY31_SendAT(const char *cmd)
{
    JDY31_SendString(cmd);
    JDY31_SendString("\r\n");
}
