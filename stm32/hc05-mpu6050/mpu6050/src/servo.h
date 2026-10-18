#ifndef __SERVO_H
#define __SERVO_H

#include "stm32f1xx_hal.h"
#include <stdint.h>

/* SG90舵机控制 (TIM3 CH1=Pan, CH2=Trim) */
/* 引脚: PA6=PAN(偏航), PA7=TRIM(俯仰) */

void Servo_Init(void);
void Servo_SetPan(int16_t angle);   /* 0~180度 */
void Servo_SetTilt(int16_t angle);  /* 0~180度 */

#endif
