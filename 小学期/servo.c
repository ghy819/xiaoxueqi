/**
  * @file    servo.c
  * @brief   舵机驱动 — 基于 STM32 标准外设库 (SPL)
  *          PA8 → TIM1_CH1, 50Hz PWM, 0.5~2.5ms 脉冲
  *
  *          TIM1 是高级定时器, 必须调用 TIM_CtrlPWMOutputs()
  *          使能 MOE 位才能输出 PWM, 否则舵机不动!
  */

#include "servo.h"

/* ================================================================
 * 初始化 — PA8 + TIM1_CH1
 * ================================================================ */

void SERVO_Init(void)
{
    GPIO_InitTypeDef         gpio;
    TIM_TimeBaseInitTypeDef  tim_base;
    TIM_OCInitTypeDef        oc;

    /* 使能 GPIOA / TIM1 / AFIO 时钟 */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA |
                           RCC_APB2Periph_TIM1  |
                           RCC_APB2Periph_AFIO, ENABLE);

    /* ---- PA8 → 复用推挽输出 (TIM1_CH1) ---- */
    GPIO_StructInit(&gpio);
    gpio.GPIO_Mode  = GPIO_Mode_AF_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_Pin   = GPIO_Pin_8;
    GPIO_Init(GPIOA, &gpio);

    /* ---- 时基: 72MHz / 72 = 1MHz, 1MHz / 20000 = 50Hz (20ms) ---- */
    TIM_TimeBaseStructInit(&tim_base);
    tim_base.TIM_Prescaler         = SERVO_PWM_PSC;       /* 71 */
    tim_base.TIM_CounterMode       = TIM_CounterMode_Up;
    tim_base.TIM_Period            = SERVO_PWM_PERIOD;    /* 19999 */
    tim_base.TIM_ClockDivision     = TIM_CKD_DIV1;
    tim_base.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM1, &tim_base);

    /* ---- CH1 PWM1, 初始 90° (1.5ms) ---- */
    TIM_OCStructInit(&oc);
    oc.TIM_OCMode      = TIM_OCMode_PWM1;
    oc.TIM_OutputState = TIM_OutputState_Enable;
    oc.TIM_Pulse       = 1500;         /* 1.5ms = 90° 居中 */
    oc.TIM_OCPolarity  = TIM_OCPolarity_High;
    TIM_OC1Init(TIM1, &oc);

    /* 高级定时器: 必须使能主输出 (MOE), 否则 PWM 不输出! */
    TIM_CtrlPWMOutputs(TIM1, ENABLE);

    /* 启动定时器 */
    TIM_Cmd(TIM1, ENABLE);
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

    TIM_SetCompare1(TIM1, (uint16_t)pulse);
}

/* ================================================================
 * 直接设置脉冲宽度 (500~2500 μs)
 * ================================================================ */

void SERVO_SetPulse(uint16_t pulse)
{
    if (pulse < SERVO_PULSE_MIN) pulse = SERVO_PULSE_MIN;
    if (pulse > SERVO_PULSE_MAX) pulse = SERVO_PULSE_MAX;

    TIM_SetCompare1(TIM1, pulse);
}