#ifndef __MYI2C_H
#define __MYI2C_H

#include "stm32f1xx_hal.h"
#include <stdint.h>

/*
 * Dual Software I2C driver for STM32F103C8T6
 *
 * I2C1 (OLED):    SCL = PB8, SDA = PB9
 * I2C2 (MPU6050): SDA = PB11, SCL = PB10
 * Power:          OLED 外部供电, PB6 默认低电平
 */

/* I2C1 pins (OLED) - PB8=SCL, PB9=SDA */
#define I2C1_SCL_PORT   GPIOB
#define I2C1_SCL_PIN    GPIO_PIN_8
#define I2C1_SDA_PORT   GPIOB
#define I2C1_SDA_PIN    GPIO_PIN_9

/* I2C2 pins (MPU6050) - PB11=SDA, PB10=SCL */
#define I2C2_SCL_PORT   GPIOB
#define I2C2_SCL_PIN    GPIO_PIN_10
#define I2C2_SDA_PORT   GPIOB
#define I2C2_SDA_PIN    GPIO_PIN_11

#define I2C_DELAY_US    5

void I2C_InitAll(void);

/* I2C1 (OLED) */
void    I2C1_Start(void);
void    I2C1_Stop(void);
uint8_t I2C1_WaitAck(void);
void    I2C1_WriteByte(uint8_t data);
uint8_t I2C1_ReadByte(uint8_t ack);
void    I2C1_WriteReg(uint8_t addr, uint8_t reg, uint8_t data);
uint8_t I2C1_ReadReg(uint8_t addr, uint8_t reg);
void    I2C1_ReadBytes(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf);

/* I2C2 (MPU6050) */
void    I2C2_Start(void);
void    I2C2_Stop(void);
uint8_t I2C2_WaitAck(void);
void    I2C2_WriteByte(uint8_t data);
uint8_t I2C2_ReadByte(uint8_t ack);
void    I2C2_WriteReg(uint8_t addr, uint8_t reg, uint8_t data);
uint8_t I2C2_ReadReg(uint8_t addr, uint8_t reg);
void    I2C2_ReadBytes(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf);

#endif
