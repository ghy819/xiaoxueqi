#ifndef __MOTOR_H
#define __MOTOR_H

#include "stm32f10x.h"
#include <stdint.h>

/* ================================================================
 * 引脚定义 — 两相步进电机驱动芯片 (TIM4 PWM)
 *
 *   左侧电机:  BI = PB8 (TIM4_CH3),  FI = PB9 (TIM4_CH4)
 *   右侧电机:  BI = PB6 (TIM4_CH1),  FI = PB7 (TIM4_CH2)
 *
 *   TIM4 默认映射 PB6~PB9, 无需重映射到 PD12~PD15
 * ================================================================ */

/* ---------- 电机数量 ---------- */
#define MOTOR_COUNT     2

/* ---------- 电机索引 ---------- */
#define MOTOR_LEFT      0   /* 左侧 */
#define MOTOR_RIGHT     1   /* 右侧 */

/* ---------- 引脚宏 (供外部参考, 驱动内部通过 TIM4 通道控制) ---------- */
#define MOTOR0_FI_PORT      GPIOB
#define MOTOR0_FI_PIN       GPIO_Pin_9
#define MOTOR0_BI_PORT      GPIOB
#define MOTOR0_BI_PIN       GPIO_Pin_8

#define MOTOR1_FI_PORT      GPIOB
#define MOTOR1_FI_PIN       GPIO_Pin_7
#define MOTOR1_BI_PORT      GPIOB
#define MOTOR1_BI_PIN       GPIO_Pin_6

/* ---------- 电机方向修正 ---------- */
#define MOTOR_RIGHT_INVERT  1       /* 1 = 右侧电机反转 (对向安装)    */

/* ---------- PWM 参数 ---------- */
#define MOTOR_PWM_PERIOD    999     /* ARR: 1MHz / 1000 = 1kHz PWM */
#define MOTOR_PWM_PSC       71      /* PSC: 72MHz / 72 = 1MHz */
#define MOTOR_SPEED_MAX     100     /* 速度百分比最大值 */
#define MOTOR_SPEED_MIN     -100    /* 速度百分比最小值 */

/* ================================================================
 * API 函数声明
 * ================================================================ */

void MOTOR_Init(void);
void MOTOR_SetSpeed(uint8_t id, int16_t speed);
void MOTOR_Stop(uint8_t id);
void MOTOR_Brake(uint8_t id);
void MOTOR_SetSpeeds(int16_t left, int16_t right);
int16_t MOTOR_GetSpeed(uint8_t id);

#endif /* __MOTOR_H */