#ifndef __TRACK_H
#define __TRACK_H

#include "stm32f10x.h"
#include <stdint.h>

/* ================================================================
 * 循迹传感器引脚定义
 *
 *   5 路光电传感器，从左到右: OUT1 ~ OUT5
 *   黑线在正下方 -> 输出低电平 (Bit_RESET)
 * ================================================================ */

/* ---------- OUT1: PC0 ---------- */
#define TRACK1_PORT         GPIOC
#define TRACK1_PIN          GPIO_Pin_0
#define TRACK1_CLK          RCC_APB2Periph_GPIOC

/* ---------- OUT2: PC1 ---------- */
#define TRACK2_PORT         GPIOC
#define TRACK2_PIN          GPIO_Pin_1
#define TRACK2_CLK          RCC_APB2Periph_GPIOC

/* ---------- OUT3: PC2 ---------- */
#define TRACK3_PORT         GPIOC
#define TRACK3_PIN          GPIO_Pin_2
#define TRACK3_CLK          RCC_APB2Periph_GPIOC

/* ---------- OUT4: PC3 ---------- */
#define TRACK4_PORT         GPIOC
#define TRACK4_PIN          GPIO_Pin_3
#define TRACK4_CLK          RCC_APB2Periph_GPIOC

/* ---------- OUT5: PC13 ---------- */
#define TRACK5_PORT         GPIOC
#define TRACK5_PIN          GPIO_Pin_13
#define TRACK5_CLK          RCC_APB2Periph_GPIOC

/* ================================================================
 * 控制参数 - 误差均为放大 10 倍后的定点数 (-20 ~ +20)
 * ================================================================ */

#define TRACK_CONTROL_PERIOD_MS       3

/* PID 与输出平滑 */
#define TRACK_PID_KP                  12
#define TRACK_PID_KD                  4
#define TRACK_PID_KI_DIV              40
#define TRACK_INTEGRAL_LIMIT          80
#define TRACK_CORRECTION_LIMIT        35
#define TRACK_SERVO_FILTER_OLD        2
#define TRACK_SERVO_FILTER_NEW        3

/* 舵机转向范围 */
#define TRACK_SERVO_CENTER_ANGLE      90
#define TRACK_SERVO_MIN_ANGLE         55
#define TRACK_SERVO_MAX_ANGLE         130

/* 分级速度与差速 */
#define TRACK_STRAIGHT_SPEED          95
#define TRACK_CURVE_SPEED             72
#define TRACK_SHARP_SPEED             45
#define TRACK_SEARCH_SPEED            40
#define TRACK_DIFF_RATIO              65
#define TRACK_SPEED_RECOVERY_STEP     2
#define TRACK_STRAIGHT_ERROR_X10      5

/* 状态确认/保持时间，避免传感器抖动造成频繁切换 */
#define TRACK_SHARP_CONFIRM_COUNT     3
#define TRACK_SHARP_MIN_HOLD_COUNT    20
#define TRACK_SHARP_EXIT_COUNT        5
#define TRACK_LOST_CONFIRM_COUNT      5
#define TRACK_REACQUIRE_COUNT         3
#define TRACK_FULL_BLACK_CONFIRM_COUNT 13

/* ================================================================
 * API
 * ================================================================ */

void    TRACK_Init(void);
uint8_t TRACK_Read(void);
uint8_t TRACK_FilterSample(uint8_t raw);
int8_t  TRACK_GetError(void);
int8_t  TRACK_GetErrorFromRaw(uint8_t raw);
int16_t TRACK_GetErrorX10(uint8_t raw);
uint8_t TRACK_SelectLikelyLine(uint8_t raw, uint8_t previous_raw);
int16_t TRACK_GetCorrection(int8_t error);

#endif /* __TRACK_H */
