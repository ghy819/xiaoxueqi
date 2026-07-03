/**
  * @file    servo.c
  * @brief   舵机驱动 — 基于 STM32 标准外设库 (SPL)
  *          PB1 → TIM3_CH4, 50Hz PWM, 0.5~2.5ms 脉冲
    */

#include "servo.h"

static uint8_t  g_servo_angle = 90;
static uint16_t g_servo_pulse = 1500;

/* ================================================================
 * 初始化 — PB1 + TIM3_CH4
 * ================================================================ */

void SERVO_Init(void)
{
    GPIO_InitTypeDef         gpio;
    TIM_TimeBaseInitTypeDef  tim_base;
    TIM_OCInitTypeDef        oc;

    /* 使能 GPIOB / AFIO / TIM3 时钟 */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB |
                           RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

    /* ---- PB1 → 复用推挽输出 (TIM3_CH4) ---- */
    GPIO_StructInit(&gpio);
    gpio.GPIO_Mode  = GPIO_Mode_AF_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_Pin   = GPIO_Pin_1;
    GPIO_Init(GPIOB, &gpio);

    /* ---- 时基: 72MHz / 72 = 1MHz, 1MHz / 20000 = 50Hz (20ms) ---- */
    TIM_TimeBaseStructInit(&tim_base);
    tim_base.TIM_Prescaler         = SERVO_PWM_PSC;       /* 71 */
    tim_base.TIM_CounterMode       = TIM_CounterMode_Up;
    tim_base.TIM_Period            = SERVO_PWM_PERIOD;    /* 19999 */
    tim_base.TIM_ClockDivision     = TIM_CKD_DIV1;
    tim_base.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM3, &tim_base);

    /* ---- CH4 PWM1, 初始 90° (1.5ms) ---- */
    TIM_OCStructInit(&oc);
    oc.TIM_OCMode      = TIM_OCMode_PWM1;
    oc.TIM_OutputState = TIM_OutputState_Enable;
    oc.TIM_Pulse       = 1500;         /* 1.5ms = 90° 居中 */
    oc.TIM_OCPolarity  = TIM_OCPolarity_High;
    TIM_OC4Init(TIM3, &oc);

    /* 启动定时器 */
    TIM_Cmd(TIM3, ENABLE);
}


/* ================================================================
 * 设置角度 (0~180°)
 * ================================================================ */

void SERVO_SetAngle(uint8_t angle)
{
    uint32_t pulse;

    if (angle > 180) angle = 180;

    pulse = SERVO_PULSE_MIN +
            (uint32_t)angle * (SERVO_PULSE_MAX - SERVO_PULSE_MIN) / 180;

    g_servo_angle = angle;
    g_servo_pulse = (uint16_t)pulse;
    TIM_SetCompare4(TIM3, (uint16_t)pulse);
}

/* ================================================================
 * 直接设置脉冲宽度 (500~2500 μs)
 * ================================================================ */

void SERVO_SetPulse(uint16_t pulse)
{
    if (pulse < SERVO_PULSE_MIN) pulse = SERVO_PULSE_MIN;
    if (pulse > SERVO_PULSE_MAX) pulse = SERVO_PULSE_MAX;

    g_servo_pulse = pulse;
    g_servo_angle = (uint8_t)(
        (uint32_t)(pulse - SERVO_PULSE_MIN) * 180U /
        (SERVO_PULSE_MAX - SERVO_PULSE_MIN));
    TIM_SetCompare4(TIM3, pulse);
}

uint8_t SERVO_GetAngle(void)
{
    return g_servo_angle;
}

uint16_t SERVO_GetPulse(void)
{
    return g_servo_pulse;
}
