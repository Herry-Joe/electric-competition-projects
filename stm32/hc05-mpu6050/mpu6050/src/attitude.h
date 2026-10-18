#ifndef __ATTITUDE_H
#define __ATTITUDE_H

#include "mpu6050.h"

/*
 * Complementary filter attitude estimation
 * Fuses accelerometer + gyroscope for stable Roll/Pitch/Yaw
 */

typedef struct {
    float roll;   /* degrees, rotation around X */
    float pitch;  /* degrees, rotation around Y */
    float yaw;    /* degrees, rotation around Z (drifts without magnetometer) */
} EulerAngle;

void        Attitude_Init(float gyro_offset_x, float gyro_offset_y, float gyro_offset_z);
EulerAngle  Attitude_Update(MPU6050_Data *sensor, float dt);

#endif
