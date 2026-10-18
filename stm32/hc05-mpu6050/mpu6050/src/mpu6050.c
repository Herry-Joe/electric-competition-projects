#include "mpu6050.h"
#include "myi2c.h"

static uint8_t read_reg(uint8_t reg)          { return I2C2_ReadReg(MPU6050_ADDR, reg); }
static void    write_reg(uint8_t reg, uint8_t val) { I2C2_WriteReg(MPU6050_ADDR, reg, val); }

/*
 * 初始化序列：
 * 1) 唤醒（PWR_MGMT_1=0x00，内部 8MHz 时钟）
 * 2) 读 WHO_AM_I 验证 I2C 通信与器件（应为 0x68）
 * 3) 配置采样率 / DLPF / 量程，全轴使能
 */
uint8_t MPU6050_Init(void)
{
    HAL_Delay(50);

    /* 唤醒，选择内部 8MHz 时钟（退出睡眠） */
    write_reg(MPU6050_PWR_MGMT_1, 0x00);
    HAL_Delay(50);

    /* 验证通信 */
    if (read_reg(MPU6050_WHO_AM_I) != MPU6050_ID) {
        return 1;   /* I2C 通信失败或器件不符 */
    }

    write_reg(MPU6050_SMPLRT_DIV, 0x07);    /* 1kHz/(1+7) = 125Hz 采样 */
    write_reg(MPU6050_CONFIG, 0x06);        /* DLPF: ~5Hz 带宽 */
    write_reg(MPU6050_GYRO_CONFIG, 0x18);   /* ±2000°/s (FS_SEL=3) */
    write_reg(MPU6050_ACCEL_CONFIG, 0x00);  /* ±2g  (AFS_SEL=0) */
    write_reg(MPU6050_PWR_MGMT_2, 0x00);    /* 全轴使能 */
    HAL_Delay(50);

    return 0;
}

void MPU6050_ReadData(MPU6050_Data *data)
{
    uint8_t buf[14];
    I2C2_ReadBytes(MPU6050_ADDR, MPU6050_ACCEL_XOUT_H, 14, buf);

    /* 大端（MSB 在前），顺序: accel(6) + temp(2) + gyro(6) */
    int16_t ax       = (int16_t)((buf[0]  << 8) | buf[1]);
    int16_t ay       = (int16_t)((buf[2]  << 8) | buf[3]);
    int16_t az       = (int16_t)((buf[4]  << 8) | buf[5]);
    int16_t temp_raw = (int16_t)((buf[6]  << 8) | buf[7]);
    int16_t gx       = (int16_t)((buf[8]  << 8) | buf[9]);
    int16_t gy       = (int16_t)((buf[10] << 8) | buf[11]);
    int16_t gz       = (int16_t)((buf[12] << 8) | buf[13]);

    data->ax = ax / MPU6050_ACCEL_SENS;
    data->ay = ay / MPU6050_ACCEL_SENS;
    data->az = az / MPU6050_ACCEL_SENS;
    data->gx = gx / MPU6050_GYRO_SENS;
    data->gy = gy / MPU6050_GYRO_SENS;
    data->gz = gz / MPU6050_GYRO_SENS;
    data->temperature = temp_raw / 340.0f + 36.53f;   /* °C */
}

void MPU6050_Calibrate(uint16_t samples,
                       float *gyro_offset_x, float *gyro_offset_y, float *gyro_offset_z)
{
    int32_t sx = 0, sy = 0, sz = 0;
    MPU6050_Data d;
    for (uint16_t i = 0; i < samples; i++) {
        MPU6050_ReadData(&d);
        sx += (int32_t)(d.gx * MPU6050_GYRO_SENS);
        sy += (int32_t)(d.gy * MPU6050_GYRO_SENS);
        sz += (int32_t)(d.gz * MPU6050_GYRO_SENS);
        HAL_Delay(2);
    }
    *gyro_offset_x = (float)sx / samples / MPU6050_GYRO_SENS;
    *gyro_offset_y = (float)sy / samples / MPU6050_GYRO_SENS;
    *gyro_offset_z = (float)sz / samples / MPU6050_GYRO_SENS;
}
