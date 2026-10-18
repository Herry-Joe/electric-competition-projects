#ifndef __MPU6050_H
#define __MPU6050_H

#include "stm32f1xx_hal.h"
#include <stdint.h>

/*
 * MPU6050 6-axis IMU driver for STM32F103C8T6 (I2C interface)
 * 接软件 I2C2：PB10=SCL, PB11=SDA，7 位地址 0x68
 *
 * 输出单位（与 attitude.c 衔接）：
 *   加速度  -> g
 *   角速度  -> °/s
 *   温度    -> °C
 */

#define MPU6050_ADDR          0x68

/* 寄存器地址 */
#define MPU6050_SMPLRT_DIV    0x19
#define MPU6050_CONFIG        0x1A
#define MPU6050_GYRO_CONFIG   0x1B
#define MPU6050_ACCEL_CONFIG  0x1C
#define MPU6050_PWR_MGMT_1    0x6B
#define MPU6050_PWR_MGMT_2    0x6C
#define MPU6050_WHO_AM_I      0x75
#define MPU6050_ACCEL_XOUT_H  0x3B   /* 连续 14 字节: accel(6)+temp(2)+gyro(6) */

#define MPU6050_ID            0x68

/* 灵敏度（LSB / 单位）
 * ACCEL: ±2g=16384, ±4g=8192, ±8g=4096, ±16g=2048 LSB/g
 * GYRO : ±250=131, ±500=65.5, ±1000=32.8, ±2000=16.4 LSB/(°/s)
 */
#define MPU6050_ACCEL_SENS   16384.0f   /* ±2g */
#define MPU6050_GYRO_SENS    16.4f      /* ±2000°/s */

/* 转换后数据结构：字段名与 attitude.c 期望一致 */
typedef struct {
    float ax, ay, az;       /* g */
    float gx, gy, gz;       /* °/s */
    float temperature;      /* °C */
} MPU6050_Data;

uint8_t MPU6050_Init(void);
void    MPU6050_ReadData(MPU6050_Data *data);
void    MPU6050_Calibrate(uint16_t samples,
                          float *gyro_offset_x, float *gyro_offset_y, float *gyro_offset_z);

#endif
