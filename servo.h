#ifndef __SERVO_H
#define __SERVO_H

#include "stm32f10x.h"
#include <stdint.h>

/* ================================================================
 * 舵机参数
 *
 *   PB1 = TIM3_CH4, 默认映射, 无需重映射
 *   周期 20ms (50Hz), 脉冲范围 0.5ms~2.5ms 对应 0°~180°
 * ================================================================ */

#define SERVO_PWM_PERIOD    19999   /* ARR: 20ms (1MHz / 20000 = 50Hz)   */
#define SERVO_PWM_PSC       71      /* PSC: 72MHz / 72 = 1MHz             */
#define SERVO_PULSE_MIN     500     /* 0.5ms →  0° (1MHz * 0.5ms = 500)  */
#define SERVO_PULSE_MAX     2500    /* 2.5ms → 180°                       */

void SERVO_Init(void);
void SERVO_SetAngle(uint8_t angle);   /* 0~180°                           */
void SERVO_SetPulse(uint16_t pulse);  /* 500~2500 (μs)                    */
uint8_t  SERVO_GetAngle(void);
uint16_t SERVO_GetPulse(void);

#endif /* __SERVO_H */
