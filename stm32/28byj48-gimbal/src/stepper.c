/**
 * @file    stepper.c
 * @brief   28BYJ-48 步进电机驱动实现 — TB6612 双 H 桥版本 (多实例)
 *
 * === 硬件连接说明 ==============================================
 *
 *  TB6612                   28BYJ-48 (5线)         STM32F103C8T6
 *  ┌──────────┐             ┌──────────┐           ┌──────────────┐
 *  │ VM  ───────────────>   │ 红 (COM) │─── 5V     │              │
 *  │ VCC ──────────────────────────────────────>   │ 3.3V         │
 *  │ GND ──────────────────────────────────────>   │ GND          │
 *  │                                                           │
 *  │ AOUT1 ──────────────>  │ 橙 (Coil1)│           │              │
 *  │ AOUT2 ──────────────>  │ 粉 (Coil2)│           │              │
 *  │ BOUT1 ──────────────>  │ 黄 (Coil3)│           │              │
 *  │ BOUT2 ──────────────>  │ 蓝 (Coil4)│           │              │
 *  │                                                           │
 *  │ PWMA ──────────────────────────────────────>   │ PA0 (PWM)    │
 *  │ AIN1 ──────────────────────────────────────>   │ PA4          │
 *  │ AIN2 ──────────────────────────────────────>   │ PA5          │
 *  │ PWMB ──────────────────────────────────────>   │ PA1 (PWM)    │
 *  │ BIN1 ──────────────────────────────────────>   │ PA6          │
 *  │ BIN2 ──────────────────────────────────────>   │ PA7          │
 *  │ STBY ──────────────────────────────────────>   │ PB0          │
 *  └──────────┘             └──────────┘           └──────────────┘
 *
 *  注：PB3/PB4/PB5/PB6 用于垂直电机 (Tilt)，需在 GPIO 初始化时解除 JTAG 复用。
 *
 * === 步进序列 (半步步进 / 8 拍) ================================
 *
 *  TB6612 真值表:
 *    IN1=0, IN2=0 → OUT1=Z, OUT2=Z  (高阻/滑行)
 *    IN1=1, IN2=0 → OUT1=H, OUT2=L  (正转)
 *    IN1=0, IN2=1 → OUT1=L, OUT2=H  (反转)
 *    IN1=1, IN2=1 → OUT1=L, OUT2=L  (刹车)
 *
 *  28BYJ-48 公共端(红)接 5V，各线圈通过 TB6612 OUTx 拉低到 GND 来励磁。
 *  当 OUTx = L 时线圈通电，OUTx = H 时线圈两端同电位无电流。
 *
 *  半步步进序列 (8拍)：
 *  拍号 | 橙(Coil1) | 粉(Coil2) | 黄(Coil3) | 蓝(Coil4) | A(IN1,IN2) | B(IN1,IN2)
 *  ─────┼───────────┼───────────┼───────────┼───────────┼────────────┼────────────
 *   0   |    L      |    H      |    H      |    H      |  0,1       |  0,0 (Z)
 *   1   |    L      |    H      |    L      |    H      |  0,1       |  0,1
 *   2   |    H      |    H      |    L      |    H      |  0,0 (Z)   |  0,1
 *   3   |    H      |    L      |    L      |    H      |  1,0       |  0,1
 *   4   |    H      |    L      |    H      |    H      |  1,0       |  0,0 (Z)
 *   5   |    H      |    L      |    H      |    L      |  1,0       |  1,0
 *   6   |    H      |    H      |    H      |    L      |  0,0 (Z)   |  1,0
 *   7   |    L      |    H      |    H      |    L      |  0,1       |  1,0
 *
 *  全步进序列 (4拍)：
 *  拍号 | 橙(Coil1) | 粉(Coil2) | 黄(Coil3) | 蓝(Coil4)
 *  ─────┼───────────┼───────────┼───────────┼───────────
 *   0   |    L      |    H      |    L      |    H
 *   1   |    H      |    L      |    L      |    H
 *   2   |    H      |    L      |    H      |    L
 *   3   |    L      |    H      |    H      |    L
 */

#include "stepper.h"
#include <stdlib.h>

/* ========================== 步进序列查找表 ========================== */

/**
 * @brief 半步步进序列 (8 拍)
 *
 * 每个元素编码低 8 位含义：
 *   bit[1:0] = A 桥 (AIN1, AIN2) → 控制橙(Coil1)/粉(Coil2)
 *   bit[3:2] = B 桥 (BIN1, BIN2) → 控制黄(Coil3)/蓝(Coil4)
 *
 * 编码规则：
 *   0b00 = IN1=0,IN2=0 → 两输出高阻 (Z) → 线圈不通电
 *   0b01 = IN1=0,IN2=1 → OUT1=L,OUT2=H → Coil1 通电 (OUT1=L)
 *   0b10 = IN1=1,IN2=0 → OUT1=H,OUT2=L → Coil2 通电 (OUT2=L)
 */
static const uint8_t half_step_seq[8] = {
    0x01,   /* 拍0: A(0,1)=0b01, B(0,0)=0b00 → 橙通电 */
    0x05,   /* 拍1: A(0,1)=0b01, B(0,1)=0b01 → 橙+黄通电 */
    0x04,   /* 拍2: A(0,0)=0b00, B(0,1)=0b01 → 黄通电 */
    0x06,   /* 拍3: A(1,0)=0b10, B(0,1)=0b01 → 粉+黄通电 */
    0x02,   /* 拍4: A(1,0)=0b10, B(0,0)=0b00 → 粉通电 */
    0x0A,   /* 拍5: A(1,0)=0b10, B(1,0)=0b10 → 粉+蓝通电 */
    0x08,   /* 拍6: A(0,0)=0b00, B(1,0)=0b10 → 蓝通电 */
    0x09,   /* 拍7: A(0,1)=0b01, B(1,0)=0b10 → 蓝+橙通电 */
};

/** @brief 全步进序列 (4拍) — 从半步步进中取偶数拍 */
static const uint8_t full_step_seq[4] = {
    0x05,   /* 橙+黄通电 */
    0x06,   /* 粉+黄通电 */
    0x0A,   /* 粉+蓝通电 */
    0x09,   /* 蓝+橙通电 */
};

/* ========================== 底层硬件操作 ========================== */

static inline void Stepper_SetA(StepperMotor *motor, uint8_t ain_pair)
{
    HAL_GPIO_WritePin(motor->pins.ain1_port, motor->pins.ain1_pin,
                      (ain_pair & 0x02) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(motor->pins.ain2_port, motor->pins.ain2_pin,
                      (ain_pair & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static inline void Stepper_SetB(StepperMotor *motor, uint8_t bin_pair)
{
    HAL_GPIO_WritePin(motor->pins.bin1_port, motor->pins.bin1_pin,
                      (bin_pair & 0x02) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(motor->pins.bin2_port, motor->pins.bin2_pin,
                      (bin_pair & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static inline void Stepper_OutputStep(StepperMotor *motor, uint8_t step_code)
{
    Stepper_SetA(motor, step_code & 0x03);
    Stepper_SetB(motor, (step_code >> 2) & 0x03);
}

static inline void Stepper_Coast(StepperMotor *motor)
{
    Stepper_SetA(motor, 0x00);
    Stepper_SetB(motor, 0x00);
}

/* ========================== 步进电机核心逻辑 ========================== */

static uint8_t Stepper_AdvanceOneStep(StepperMotor *motor)
{
    const uint8_t *seq;
    uint8_t max_steps;

    if (motor->mode == STEP_MODE_HALF) {
        seq = half_step_seq;
        max_steps = 8;
    } else {
        seq = full_step_seq;
        max_steps = 4;
    }

    /* 更新步序索引 (CW 递增, CCW 递减) */
    if (motor->dir == DIR_CW) {
        motor->step_index = (motor->step_index + 1) % max_steps;
    } else {
        motor->step_index = (motor->step_index == 0)
                            ? (max_steps - 1)
                            : (motor->step_index - 1);
    }

    Stepper_OutputStep(motor, seq[motor->step_index]);

    /* 目标步数递减 */
    if (motor->target_steps > 0) {
        motor->target_steps--;
        if (motor->target_steps == 0) {
            /* 扫描模式：到达限位后由 IRQ 处理自动换向，不进入空闲 */
            if (motor->work_mode == WORK_MODE_SWEEP) {
                /* 保持运行状态，IRQ 中 target_steps <= 0 触发换向 */
            } else {
                motor->state = STATE_IDLE;
                /* [修复] 旋转完成后恢复原始方向 */
                if (motor->restore_dir_flag) {
                    motor->dir = motor->saved_dir;
                    motor->restore_dir_flag = 0;
                }
            }
        }
    }

    return motor->step_index;
}

/* ========================== 公共接口实现 ========================== */

void Stepper_Init(StepperMotor *motor, const StepperPins *pins,
                  TIM_HandleTypeDef *htim, const char *label)
{
    memset(motor, 0, sizeof(StepperMotor));

    motor->mode           = STEP_MODE_HALF;
    motor->dir            = DIR_CW;
    motor->state          = STATE_IDLE;
    motor->work_mode      = WORK_MODE_MANUAL;
    motor->step_delay_us  = STEPPER_DEFAULT_DELAY;
    motor->angle_limit_steps = STEPPER_HALF_REV / 4;
    motor->htim           = htim;
    motor->soft_start     = 0;
    motor->soft_start_delay = 0;

    if (pins) {
        motor->pins = *pins;
    }
    if (label) {
        strncpy(motor->label, label, sizeof(motor->label) - 1);
        motor->label[sizeof(motor->label) - 1] = '\0';
    }

    /* 上电不输出励磁, 避免电机瞬间大电流把蓝牙模块电压拉垮 */
    /* 等收到串口命令时 Stepper_RunContinuously / StepBy 才会通电 */
    Stepper_Coast(motor);

    printf("\r\n%s 电机初始化完成\r\n", motor->label);
}

void Stepper_SetMode(StepperMotor *motor, StepMode mode)
{
    motor->mode = mode;
    /* [修复] 切换模式时复位步序索引，防止数组越界 */
    motor->step_index = 0;
    /* 输出新模式第一拍，保持线圈励磁 */
    Stepper_OutputStep(motor,
        (mode == STEP_MODE_HALF) ? half_step_seq[0] : full_step_seq[0]);
    printf("[%s] %s\r\n", motor->label,
           (mode == STEP_MODE_HALF) ? "半步步进" : "全步步进");
}

void Stepper_SetDirection(StepperMotor *motor, Direction dir)
{
    motor->dir = dir;
    printf("[%s] %s\r\n", motor->label,
           (dir == DIR_CW) ? "顺时针 (CW)" : "逆时针 (CCW)");
}

void Stepper_SetSpeed(StepperMotor *motor, uint32_t delay_us)
{
    if (delay_us < STEPPER_MIN_DELAY_US)  delay_us = STEPPER_MIN_DELAY_US;
    if (delay_us > STEPPER_MAX_DELAY_US)  delay_us = STEPPER_MAX_DELAY_US;

    motor->step_delay_us = delay_us;

    /* [修复] 更新定时器周期前先停止中断，防止 ISR 中读取不一致 */
    __HAL_TIM_DISABLE_IT(motor->htim, TIM_IT_UPDATE);
    __HAL_TIM_SET_AUTORELOAD(motor->htim, delay_us - 1);
    __HAL_TIM_ENABLE_IT(motor->htim, TIM_IT_UPDATE);

    printf("[%s] %lu us/步\r\n", motor->label, (unsigned long)delay_us);
}

void Stepper_RunContinuously(StepperMotor *motor)
{
    /* [修复] 临界区保护，防止 main loop 和 ISR 竞态 */
    __disable_irq();

    /* 从空闲进入连续运行 → 启用软加速, 起步更平稳 */
    if (motor->state == STATE_IDLE) {
        motor->soft_start = 8;                      /* 8 步内完成加速 */
        motor->soft_start_delay = motor->step_delay_us * 3; /* 起步延迟 = 3x 目标 */
        /* 设置定时器为软启动起始速度 */
        __HAL_TIM_SET_AUTORELOAD(motor->htim, motor->soft_start_delay - 1);
    }

    motor->state = STATE_RUNNING;
    motor->target_steps = 0;
    motor->work_mode = WORK_MODE_MANUAL;
    __enable_irq();
    printf("[%s] 连续运转\r\n", motor->label);
}

void Stepper_StepBy(StepperMotor *motor, uint32_t steps)
{
    __disable_irq();
    motor->target_steps = (int32_t)steps;
    motor->state = STATE_STEPPING;
    motor->work_mode = WORK_MODE_MANUAL;
    __enable_irq();
    printf("[%s] %lu 步\r\n", motor->label, (unsigned long)steps);
}

void Stepper_RotateDegrees(StepperMotor *motor, float degrees)
{
    uint32_t steps_per_rev;
    uint32_t steps;

    steps_per_rev = (motor->mode == STEP_MODE_HALF)
                    ? STEPPER_HALF_REV : STEPPER_FULL_REV;

    float abs_degrees = (degrees < 0.0f) ? -degrees : degrees;
    steps = (uint32_t)((abs_degrees / 360.0f) * (float)steps_per_rev);

    /* [修复] 保存当前方向，旋转完成后恢复 */
    __disable_irq();
    motor->saved_dir = motor->dir;
    motor->restore_dir_flag = 1;

    if (degrees < 0) {
        motor->dir = DIR_CCW;
    } else {
        motor->dir = DIR_CW;
    }

    motor->target_steps = (int32_t)steps;
    motor->state = STATE_TARGET;
    motor->work_mode = WORK_MODE_MANUAL;
    __enable_irq();

    printf("[%s] 旋转 %.1f° → %lu 步 (%s)\r\n",
           motor->label, (double)degrees, (unsigned long)steps,
           (motor->dir == DIR_CW) ? "CW" : "CCW");
}

void Stepper_SweepEnable(StepperMotor *motor, uint32_t limit_degrees)
{
    if (limit_degrees > 360) limit_degrees = 360;
    if (limit_degrees == 0)  limit_degrees = 90;

    uint32_t steps_per_rev = (motor->mode == STEP_MODE_HALF)
                             ? STEPPER_HALF_REV : STEPPER_FULL_REV;

    motor->angle_limit_steps = (uint32_t)(
        ((float)limit_degrees / 360.0f) * (float)steps_per_rev
    );

    motor->work_mode = WORK_MODE_SWEEP;
    motor->target_steps = (int32_t)motor->angle_limit_steps;
    motor->state = STATE_RUNNING;

    printf("[%s] 扫描 ±%lu° → %lu 步/方向\r\n",
           motor->label,
           (unsigned long)(limit_degrees / 2),
           (unsigned long)motor->angle_limit_steps);
}

void Stepper_Stop(StepperMotor *motor)
{
    __disable_irq();
    motor->state = STATE_IDLE;
    motor->target_steps = 0;
    motor->work_mode = WORK_MODE_MANUAL;
    __enable_irq();
    /* 不调用 Stepper_Coast: 保持当前步序的线圈励磁, 输出保持力矩,
     * 云台不会因重力而下垂。若需自由滑行, 可调用 Stepper_Coast */
    printf("[%s] 停止 (保持力矩)\r\n", motor->label);
}

/* ========================== 定时器中断处理 ========================== */

void Stepper_TIM_IRQHandler(StepperMotor *motor)
{
    if (motor->state == STATE_IDLE) {
        return;
    }

    /* 扫描模式：到达限位自动换向 */
    if (motor->work_mode == WORK_MODE_SWEEP) {
        if (motor->target_steps <= 0) {
            motor->dir = (motor->dir == DIR_CW) ? DIR_CCW : DIR_CW;
            motor->target_steps = (int32_t)motor->angle_limit_steps;
        }
    }

    Stepper_AdvanceOneStep(motor);

    /* 软件加速: 每一步逐渐提高速度 (降低延迟), 8 步后到达目标 */
    if (motor->soft_start > 0) {
        motor->soft_start--;
        uint32_t ramp_steps = 8;  /* 总加速步数 */
        uint32_t done = ramp_steps - motor->soft_start;  /* 已完成步数 */
        /* 线性插值: 从 soft_start_delay 逐步降至 step_delay_us */
        uint32_t target = motor->step_delay_us;
        uint32_t start = motor->soft_start_delay;
        if (done > 0) {
            uint32_t new_delay;
            if (start > target) {
                new_delay = start - (start - target) * done / ramp_steps;
            } else {
                new_delay = target;  /* 目标已经比起点快 */
            }
            if (new_delay < target) new_delay = target;
            __HAL_TIM_SET_AUTORELOAD(motor->htim, new_delay - 1);
        }
    }
}

/* ========================== 状态查询 ========================== */

uint8_t Stepper_IsRunning(const StepperMotor *motor)
{
    return (motor->state != STATE_IDLE);
}

MotorState Stepper_GetState(const StepperMotor *motor)
{
    return motor->state;
}

void Stepper_PrintStatus(const StepperMotor *motor)
{
    const char *mode_str[]  = {"全步步进", "半步步进"};
    const char *dir_str[]   = {"顺时针(CW)", "逆时针(CCW)"};
    const char *state_str[] = {"空闲", "连续运行", "定步运行", "定角运行"};
    const char *work_str[]  = {"手动", "扫描", "串口"};

    printf("\r\n========== %s 电机状态 ==========\r\n", motor->label);
    printf(" 步进模式 : %s\r\n",   mode_str[motor->mode]);
    printf(" 旋转方向 : %s\r\n",   dir_str[motor->dir]);
    printf(" 运行状态 : %s\r\n",   state_str[motor->state]);
    printf(" 工作模式 : %s\r\n",   work_str[motor->work_mode]);
    printf(" 当前速度 : %lu us/步\r\n", (unsigned long)motor->step_delay_us);
    printf(" 当前步序 : %d\r\n",   motor->step_index);
    printf(" 剩余步数 : %ld\r\n",  (long)motor->target_steps);
    printf("===============================\r\n\r\n");
}