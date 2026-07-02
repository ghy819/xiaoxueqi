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

#define TRACK_CONTROL_PERIOD_MS 3   /* 控制与传感器采样周期            */
#define TRACK_BASE_SPEED    100     /* 直线基准速度 (0~100)            */

/* 分段修正系数: 偏差越大, 修正越强 */
#define TRACK_KP_SMALL      8       /* |error| <= 1 : 轻微修正         */
#define TRACK_KP_MEDIUM     16      /* |error| <= 2 : 中等修正         */
#define TRACK_KP_LARGE      24      /* |error| <= 3 : 较大修正         */
#define TRACK_KP_EXTREME    30      /* |error| <= 4 : 极限修正         */

/* PD 控制: 微分项系数 (预判弯道趋势) */
#define TRACK_KD            12      /* 微分增益, 根据实测调整          */
#define TRACK_INNER_STEER_RATIO 60  /* OUT2/OUT4 转向量保留 60%        */
#define TRACK_INNER_SERVO_OLD_WEIGHT 3 /* OUT2/OUT4 平滑时保留旧角度    */
#define TRACK_INNER_SERVO_NEW_WEIGHT 2 /* OUT2/OUT4 平滑时加入目标角度  */

/* 舵机转向范围: 以 90° 为中位，限制机械摆角避免急甩和轮胎阻力 */
#define TRACK_SERVO_MIN_ANGLE 55
#define TRACK_SERVO_MAX_ANGLE 125

/* 弯道分级减速: 刚出现偏差就减速，严重偏离时进一步降速 */
#define TRACK_CURVE_ENTRY_RATIO  90 /* |error| = 1: 基准速度的 90%     */
#define TRACK_CURVE_SHARP_RATIO  75 /* |error| >= 2: 基准速度的 75%    */
#define TRACK_SPEED_RECOVERY_STEP 3 /* 出弯后每周期恢复 3%             */
#define TRACK_DIFF_RATIO    80      /* 普通转弯差速 = 修正量 × 80%     */
#define TRACK_EDGE_INNER_SPEED 60   /* OUT1/OUT5 纠偏时内侧轮速度      */
#define TRACK_EDGE_OUTER_SPEED 90   /* OUT1/OUT5 纠偏时外侧轮速度      */
#define TRACK_LOST_CONFIRM_COUNT 5  /* 连续全白约 15ms 才确认脱线      */
#define TRACK_FULL_BLACK_CONFIRM_COUNT 13 /* 连续全黑约 40ms 才确认停止 */

/* ================================================================
 * API
 * ================================================================ */

void    TRACK_Init(void);
uint8_t TRACK_Read(void);           /* 返回 5-bit, bit0=OUT1 黑线      */
uint8_t TRACK_FilterSample(uint8_t raw); /* 3 次跨周期多数滤波          */
int8_t  TRACK_GetError(void);       /* 位置偏差, -4 ~ +4, 0=居中       */
int8_t  TRACK_GetErrorFromRaw(uint8_t raw); /* 根据一次采样计算偏差     */
uint8_t TRACK_SelectLikelyLine(uint8_t raw, uint8_t previous_raw);
int16_t TRACK_GetCorrection(int8_t error);  /* 偏差 → 修正量           */

#endif /* __TRACK_H */
