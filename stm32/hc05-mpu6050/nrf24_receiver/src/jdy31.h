#ifndef __JDY31_H
#define __JDY31_H

#include "stm32f1xx_hal.h"
#include <stdint.h>

/*
 * JDY-31 蓝牙串口透传驱动 (STM32F103C8T6, USART2)
 * 发送端与接收端共用本文件，逻辑完全一致（都是串口透传，无需区分主从）。
 *
 * 接线 (USART2):  PA2 = TX  -> JDY-31 的 RXD
 *                 PA3 = RX  -> JDY-31 的 TXD
 *                 GND -> GND
 *                 VCC -> 3.3V~5V（模块支持 3.6~6V，与 MCU 共地）
 *
 * ★★★ 重要硬件限制 ★★★
 * JDY-31 是“从机(Slave) Only”模块，没有 ROLE/主从 AT 指令。
 * 因此【两块板若都插 JDY-31 是无法互相连接的】——蓝牙必须由一端主动发起。
 * 可行的板对板拓扑（固件侧两者都是透明串口，无需改动）：
 *   方案A（推荐，改动最小）：发射板换 HC-05 设为主机，绑定接收板 JDY-31 的 MAC；
 *                            两端都仍接 PA2/PA3，固件不变。
 *   方案B：两端都用 HC-05（一个主一个从）。
 *   方案C（只用 JDY-31）：一端接电脑/手机(主机)，JDY-31 当从机被连接，
 *                        数据直接进 PC 上位机；此时板对板 OLED 链路不成立。
 * 详见随附 README_JDY31.md。
 *
 * 默认波特率 9600（JDY-31 出厂默认），无需 AT 即可工作。
 * 如需更高速率：先对模块发 AT+BAUD8（=115200），再把本文件的 JDY31_BAUD 改 115200。
 *
 * 帧格式（透传）： [0xAA][0x55] + 16 字节 attitude_pkt_t
 *   —— 加 2 字节同步头是为了在串口字节流中正确对齐（避免丢字节后错位）。
 */

#ifndef JDY31_BAUD
#define JDY31_BAUD   9600
#endif

/* 收发双方共用的姿态数据包（16 字节 = 4 个 float，与旧 nRF24 完全一致） */
typedef struct {
    float roll;
    float pitch;
    float yaw;
    float temperature;
} attitude_pkt_t;

#define JDY31_PAYLOAD_LEN  ((uint8_t)sizeof(attitude_pkt_t))   /* 16 */
#define JDY31_SYNC0  0xAA
#define JDY31_SYNC1  0x55
#define JDY31_FRAME_LEN  (2 + JDY31_PAYLOAD_LEN)               /* 18 */

void JDY31_Init(void);                  /* USART2 + GPIO 初始化，并启动 RX 中断 */
void JDY31_Send(const attitude_pkt_t *pkt);   /* 发送一帧（含同步头） */
uint32_t JDY31_SentCount(void);         /* 发送成功计数（诊断用） */
uint8_t JDY31_DataReady(void);          /* 1 = 已解出一帧 */
void JDY31_Read(attitude_pkt_t *pkt);   /* 取出解出的帧 */
void JDY31_SendString(const char *s);   /* 透传发送字符串（调试用） */
void JDY31_SendAT(const char *cmd);     /* 发送 AT 指令（自动补 \r\n，未连接时有效） */

#endif /* __JDY31_H */
