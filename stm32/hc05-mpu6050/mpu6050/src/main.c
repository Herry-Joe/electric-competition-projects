#include "stm32f1xx_hal.h"
#include <stdio.h>
#include "oled_draw.h"
#include "myi2c.h"
#include "mpu6050.h"
#include "attitude.h"
#include "servo.h"
#include "jdy31.h"

static void SystemClock_Config(void);

int main(void)
{
    static MPU6050_Data imu_data;
    static Attitude_Display_t attitude;
    static float gyro_offset_x, gyro_offset_y, gyro_offset_z;
    static EulerAngle euler = {0};
    static attitude_pkt_t pkt;
    uint32_t last_oled_tick = 0;
    uint32_t last_mpu_tick = 0;
    uint32_t last_tx_tick = 0;

    HAL_Init();
    SystemClock_Config();

    /* 初始化 OLED */
    OLED_Draw_Init();
    OLED_Draw_Clear();
    OLED_Draw_String(0, 0, "Initializing...");
    OLED_Draw_Refresh();
    HAL_Delay(500);

    /* 初始化 I2C（OLED + MPU6050 软 I2C） */
    I2C_InitAll();
    HAL_Delay(100);

    /* JDY-31 蓝牙发射端（USART2: PA2=TX, PA3=RX） */
    JDY31_Init();

    /* 初始化 MPU6050 */
    if (MPU6050_Init() != 0) {
        OLED_Draw_Clear();
        OLED_Draw_String(0, 0, "MPU6050 FAIL!");
        OLED_Draw_Refresh();
        while (1);
    }

    OLED_Draw_Clear();
    OLED_Draw_String(0, 0, "MPU6050 OK");
    OLED_Draw_String(0, 16, "Calibrating...");
    OLED_Draw_Refresh();

    MPU6050_Calibrate(200, &gyro_offset_x, &gyro_offset_y, &gyro_offset_z);
    Attitude_Init(gyro_offset_x, gyro_offset_y, gyro_offset_z);

    /* 初始化舵机（保留本地云台演示） */
    Servo_Init();
    Servo_SetPan(90);
    Servo_SetTilt(90);

    while (1) {
        uint32_t now = HAL_GetTick();

        /* IMU 读取 + 姿态解算 (10ms) */
        if ((now - last_mpu_tick) >= 10) {
            last_mpu_tick = now;
            MPU6050_ReadData(&imu_data);

            euler = Attitude_Update(&imu_data, 0.01f);

            attitude.roll = euler.roll;
            attitude.pitch = euler.pitch;
            attitude.yaw = euler.yaw;
            attitude.temperature = imu_data.temperature;

            /* 舵机控制 */
            {
                int16_t tilt_angle = 90 - (int16_t)(euler.roll);
                static float pan_angle_f = 90.0f;
                pan_angle_f += imu_data.gz * 0.02f;
                if (pan_angle_f < 0) pan_angle_f = 0;
                if (pan_angle_f > 180) pan_angle_f = 180;
                if (imu_data.gz > -2.0f && imu_data.gz < 2.0f) {
                    pan_angle_f += (90.0f - pan_angle_f) * 0.02f;
                }
                int16_t pan_angle = (int16_t)pan_angle_f;
                if (tilt_angle < 0) tilt_angle = 0;
                if (tilt_angle > 180) tilt_angle = 180;
                Servo_SetPan(pan_angle);
                Servo_SetTilt(tilt_angle);
            }
        }

        /* 蓝牙发送姿态包 (40ms)。
         * 9600 波特下每帧 18 字节 ≈ 18.8ms，40ms 间隔留足余量不丢包；
         * 若已 AT+BAUD8 升到 115200，可改回 20ms。 */
        if ((now - last_tx_tick) >= 40) {
            last_tx_tick = now;
            pkt.roll = euler.roll;
            pkt.pitch = euler.pitch;
            pkt.yaw = euler.yaw;
            pkt.temperature = imu_data.temperature;
            JDY31_Send(&pkt);
        }

        /* OLED 刷新 (50ms)：姿态屏与状态屏每 1 秒交替，便于盲调发送端 */
        if ((now - last_oled_tick) >= 50) {
            last_oled_tick = now;
            OLED_Draw_Clear();
            char line[24];
            if ((now / 1000) % 2 == 0) {
                OLED_Draw_Attitude(&attitude);
                /* 底行右侧常驻发送计数：一眼判断 TX 是否真的发出去 */
                snprintf(line, sizeof(line), "S:%lu", (unsigned long)JDY31_SentCount());
                OLED_Draw_String(80, 48, line);
            } else {
                OLED_Draw_String(0, 0, "TX STATUS");
                snprintf(line, sizeof(line), "SENT:%lu", (unsigned long)JDY31_SentCount());
                OLED_Draw_String(0, 16, line);
                snprintf(line, sizeof(line), "BAUD:%d", JDY31_BAUD);
                OLED_Draw_String(0, 32, line);
                OLED_Draw_String(0, 48, "USART2 PA2/3");
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
