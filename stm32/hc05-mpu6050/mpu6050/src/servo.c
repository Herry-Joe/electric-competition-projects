#include "servo.h"

/* SG90: 50Hz, 0.5ms~2.5ms (0°~180°) */
/* TIM3 PA6=CH1(Pan), PA7=CH2(Tilt) */

/* 72MHz / 72 = 1MHz, 1MHz / 20000 = 50Hz */
/* 0.5ms = 500, 1.5ms = 1500, 2.5ms = 2500 */

static TIM_HandleTypeDef htim3;

static void set_angle(uint32_t channel, int16_t angle)
{
    uint32_t pulse;
    if (angle < 0) angle = 0;
    if (angle > 180) angle = 180;
    pulse = 500 + (uint32_t)angle * 2000 / 180;
    __HAL_TIM_SET_COMPARE(&htim3, channel, pulse);
}

void Servo_Init(void)
{
    GPIO_InitTypeDef g = {0};
    TIM_OC_InitTypeDef sConfig = {0};

    __HAL_RCC_TIM3_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    /* PA6, PA7 复用推挽输出 */
    g.Pin = GPIO_PIN_6 | GPIO_PIN_7;
    g.Mode = GPIO_MODE_AF_PP;
    g.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &g);

    /* TIM3: 72MHz / 72 = 1MHz, ARR=19999 → 50Hz */
    htim3.Instance = TIM3;
    htim3.Init.Prescaler = 72 - 1;
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim3.Init.Period = 20000 - 1;
    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_PWM_Init(&htim3);

    /* CH1, CH2 PWM */
    sConfig.OCMode = TIM_OCMODE_PWM1;
    sConfig.Pulse = 1500;
    sConfig.OCPolarity = TIM_OCPOLARITY_HIGH;
    HAL_TIM_PWM_ConfigChannel(&htim3, &sConfig, TIM_CHANNEL_1);
    HAL_TIM_PWM_ConfigChannel(&htim3, &sConfig, TIM_CHANNEL_2);

    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
}

void Servo_SetPan(int16_t angle)
{
    set_angle(TIM_CHANNEL_1, angle);
}

void Servo_SetTilt(int16_t angle)
{
    set_angle(TIM_CHANNEL_2, angle);
}
