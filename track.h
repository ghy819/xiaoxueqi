#ifndef __TRACK_H
#define __TRACK_H

#include "stm32f10x.h"
#include <stdint.h>

/* ================================================================
 * 循迹传感器引脚定义
 *
 *   5 路光电传感器, 从左到右: OUT1 ~ OUT5
 *   黑线在正下方 → 输出低电平 (Bit_RESET)
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
 * 修正参数 — 根据实测调整
 * ================================================================ */

#define TRACK_BASE_SPEED    60      /* 基准速度 (0~100)                */

/* 分段修正系数: 偏差越大, 修正越强 */
#define TRACK_KP_SMALL      8       /* |error| <= 1 : 轻微修正         */
#define TRACK_KP_MEDIUM     16      /* |error| <= 2 : 中等修正         */
#define TRACK_KP_LARGE      24      /* |error| <= 3 : 较大修正         */
#define TRACK_KP_EXTREME    30      /* |error| <= 4 : 极限修正         */

/* PD 控制: 微分项系数 (预判弯道趋势) */
#define TRACK_KD            12      /* 微分增益, 根据实测调整          */

/* 弯道减速: 偏差大时自动降速, 防止冲出赛道 */
#define TRACK_CURVE_SLOW    3       /* OUT1/OUT5 等大偏差时才触发减速 */
#define TRACK_CURVE_RATIO   75      /* 弯道速度 = 基准 × 75%           */
#define TRACK_DIFF_RATIO    80      /* 普通转弯差速 = 修正量 × 80%     */
#define TRACK_PIVOT_SPEED   45      /* OUT1/OUT5 原地纠偏轮速          */

/* ================================================================
 * API
 * ================================================================ */

void    TRACK_Init(void);
uint8_t TRACK_Read(void);           /* 返回 5-bit, bit0=OUT1 黑线      */
int8_t  TRACK_GetError(void);       /* 位置偏差, -4 ~ +4, 0=居中       */
int16_t TRACK_GetCorrection(int8_t error);  /* 偏差 → 修正量           */

#endif /* __TRACK_H */
