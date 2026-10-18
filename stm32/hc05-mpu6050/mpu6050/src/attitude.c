#include "attitude.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

#define RAD_TO_DEG  (180.0f / M_PI)

/* Complementary filter: 0.98 gyro, 0.02 accel */
#define CF_ALPHA    0.98f

static float gyro_offset_x, gyro_offset_y, gyro_offset_z;
static float angle_roll, angle_pitch, angle_yaw;

void Attitude_Init(float gox, float goy, float goz)
{
    gyro_offset_x = gox;
    gyro_offset_y = goy;
    gyro_offset_z = goz;
    angle_roll  = 0.0f;
    angle_pitch = 0.0f;
    angle_yaw   = 0.0f;
}

EulerAngle Attitude_Update(MPU6050_Data *sensor, float dt)
{
    EulerAngle euler;
    float gx = sensor->gx - gyro_offset_x;
    float gy = sensor->gy - gyro_offset_y;
    float gz = sensor->gz - gyro_offset_z;
    float ax = sensor->ax;
    float ay = sensor->ay;
    float az = sensor->az;
    float accel_roll, accel_pitch;

    /* Gyro integration */
    angle_roll  += gx * dt;
    angle_pitch += gy * dt;
    angle_yaw   += gz * dt;

    /* Accel angles (GY-521: ax=倾斜轴) */
    accel_roll  = atan2f(-ax, sqrtf(ay * ay + az * az)) * RAD_TO_DEG;
    accel_pitch = atan2f(ay, az) * RAD_TO_DEG;

    /* Complementary filter */
    angle_roll  = CF_ALPHA * angle_roll  + (1.0f - CF_ALPHA) * accel_roll;
    angle_pitch = CF_ALPHA * angle_pitch + (1.0f - CF_ALPHA) * accel_pitch;
    /* yaw: gyro only (no magnetometer) */

    euler.roll  = angle_roll;
    euler.pitch = angle_pitch;
    euler.yaw   = angle_yaw;
    return euler;
}
