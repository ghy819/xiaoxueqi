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
#define TRACK_D_LIMIT       12      /* 限制微分冲击，避免舵机反向抖动 */
#define TRACK_INNER_STEER_INITIAL_RATIO 35 /* 首次压到 OUT2/OUT4 轻纠偏 */
#define TRACK_OUT2_STEER_RATIO 60   /* OUT2 转向量保留 60%             */
#define TRACK_OUT4_STEER_RATIO 75   /* 顺时针时增强 OUT4 右转响应      */
#define TRACK_INNER_SERVO_OLD_WEIGHT 3 /* OUT2/OUT4 平滑时保留旧角度    */
#define TRACK_INNER_SERVO_NEW_WEIGHT 2 /* OUT2/OUT4 平滑时加入目标角度  */

/* 顺时针椭圆赛道的直道/弯道识别与直道稳定参数 */
#define TRACK_CURVE_ENTER_COUNT 2   /* OUT4 连续 2 次后进入弯道模式    */
#define TRACK_STRAIGHT_ENTER_COUNT 20 /* 回中约 60ms 后进入直道模式     */
#define TRACK_STRAIGHT_OUTER_CONFIRM_COUNT 3 /* 直道外侧信号确认次数     */
#define TRACK_STRAIGHT_SMALL_STEER_RATIO 0  /* 直道中心附近设为转向死区 */
#define TRACK_STRAIGHT_INNER_STEER_RATIO 25 /* 直道 OUT2/OUT4 转向量    */
#define TRACK_STRAIGHT_SERVO_OLD_WEIGHT 5
#define TRACK_STRAIGHT_SERVO_NEW_WEIGHT 1

/* 舵机转向范围: 以 90° 为中位，限制机械摆角避免急甩和轮胎阻力 */
#define TRACK_SERVO_MIN_ANGLE 55
#define TRACK_SERVO_MAX_ANGLE 130   /* 右转多保留 5°，防止冲向外圈     */

/* 弯道分级减速: 刚出现偏差就减速，严重偏离时进一步降速 */
#define TRACK_CURVE_ENTRY_RATIO 100 /* |error| = 1: 直线轻微偏差不减速 */
#define TRACK_CURVE_SHARP_RATIO  78 /* |error| >= 2: 基准速度的 78%    */
#define TRACK_SPEED_RECOVERY_STEP 3 /* 出弯后每周期恢复 3%             */
#define TRACK_DIFF_RATIO    80      /* 普通转弯差速 = 修正量 × 80%     */
#define TRACK_STRAIGHT_DIFF_RATIO 25 /* 直道微调减小差速以保持高速      */
#define TRACK_EDGE_INNER_SPEED 63   /* OUT1/OUT5 纠偏时内侧轮速度      */
#define TRACK_EDGE_OUTER_SPEED 93   /* OUT1/OUT5 纠偏时外侧轮速度      */
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
