/**
  * @file    track.c
  * @brief   五路循迹传感器 — 加权位置偏差 + 分段修正
  *
  *          传感器布局 (俯视):
  *            OUT1    OUT2    OUT3    OUT4    OUT5
  *            PA15    PC10    PC11    PC12    PD2
  *            (左)                     (右)
  *
  *          权重:       -4      -2       0      +2      +4
  *
  *          偏差 = Σ(权重[i] × 是否压线[i]) / 压线传感器数量
  */

#include "track.h"

/* ================================================================
 * 传感器权重表 (从左到右)
 * ================================================================ */

static const int8_t TRACK_WEIGHT[5] = { -4, -2, 0, 2, 4 };

/* ================================================================
 * 初始化 — 5 路 GPIO 上拉输入 (跨 GPIOA / GPIOC / GPIOD)
 * ================================================================ */

void TRACK_Init(void)
{
    GPIO_InitTypeDef gpio;

    /* 使能各端口时钟 */
    RCC_APB2PeriphClockCmd(TRACK1_CLK | TRACK2_CLK | TRACK5_CLK, ENABLE);

    GPIO_StructInit(&gpio);
    gpio.GPIO_Mode = GPIO_Mode_IPU;   /* 上拉输入: 未压线=高, 压线=低 */

    /* PA15 */
    gpio.GPIO_Pin = TRACK1_PIN;
    GPIO_Init(TRACK1_PORT, &gpio);

    /* PC10, PC11, PC12 (同端口合并) */
    gpio.GPIO_Pin = TRACK2_PIN | TRACK3_PIN | TRACK4_PIN;
    GPIO_Init(TRACK2_PORT, &gpio);

    /* PD2 */
    gpio.GPIO_Pin = TRACK5_PIN;
    GPIO_Init(TRACK5_PORT, &gpio);
}

/* ================================================================
 * 读取 5 路传感器原始值
 * 返回: bit4=OUT5, bit3=OUT4, ..., bit0=OUT1
 *       1 = 检测到黑线 (低电平)
 * ================================================================ */

uint8_t TRACK_Read(void)
{
    uint8_t result = 0;

    if (GPIO_ReadInputDataBit(TRACK1_PORT, TRACK1_PIN) == Bit_RESET) result |= 0x01;
    if (GPIO_ReadInputDataBit(TRACK2_PORT, TRACK2_PIN) == Bit_RESET) result |= 0x02;
    if (GPIO_ReadInputDataBit(TRACK3_PORT, TRACK3_PIN) == Bit_RESET) result |= 0x04;
    if (GPIO_ReadInputDataBit(TRACK4_PORT, TRACK4_PIN) == Bit_RESET) result |= 0x08;
    if (GPIO_ReadInputDataBit(TRACK5_PORT, TRACK5_PIN) == Bit_RESET) result |= 0x10;

    return result;
}

/* ================================================================
 * 计算位置偏差 (-4 ~ +4)
 *
 *   0  = 居中 (OUT3 压线, 或对称压线)
 *  <0  = 偏右 (左边传感器压线, 需左转修正)
 *  >0  = 偏左 (右边传感器压线, 需右转修正)
 * ================================================================ */

int8_t TRACK_GetError(void)
{
    uint8_t raw = TRACK_Read();
    int8_t  error_sum = 0;
    int8_t  count = 0;

    for (uint8_t i = 0; i < 5; i++) {
        if (raw & (0x01 << i)) {
            error_sum += TRACK_WEIGHT[i];
            count++;
        }
    }

    if (count == 0) return 0;   /* 完全脱线: 保持直行 */

    return error_sum / count;
}

/* ================================================================
 * 偏差 → 修正量 (分段 KP)
 * ================================================================ */

int16_t TRACK_GetCorrection(int8_t error)
{
    if (error == 0) return 0;

    int8_t  abs_err = (error > 0) ? error : -error;
    int16_t correction;

    if (abs_err <= 1) {
        correction = (int16_t)abs_err * TRACK_KP_SMALL;
    } else if (abs_err <= 2) {
        correction = (int16_t)abs_err * TRACK_KP_MEDIUM;
    } else if (abs_err <= 3) {
        correction = (int16_t)abs_err * TRACK_KP_LARGE;
    } else {
        correction = (int16_t)abs_err * TRACK_KP_EXTREME;
    }

    return (error > 0) ? correction : -correction;
}