/**
  * @file    track.c
  * @brief   五路循迹传感器 — 加权位置偏差 + 分段修正
  *
  *          传感器布局 (俯视):
  *            OUT1    OUT2    OUT3    OUT4    OUT5
  *            PC0     PC1     PC2     PC3     PC13
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
 * 初始化 — 5 路 GPIOC 上拉输入
 * ================================================================ */

void TRACK_Init(void)
{
    GPIO_InitTypeDef gpio;

    /* 使能各端口时钟 */
    RCC_APB2PeriphClockCmd(TRACK1_CLK | TRACK2_CLK | TRACK5_CLK, ENABLE);

    GPIO_StructInit(&gpio);
    gpio.GPIO_Mode = GPIO_Mode_IPU;   /* 上拉输入: 未压线=高, 压线=低 */

    /* PC0, PC1, PC2, PC3, PC13 (同端口合并) */
    gpio.GPIO_Pin = TRACK1_PIN | TRACK2_PIN | TRACK3_PIN |
                    TRACK4_PIN | TRACK5_PIN;
    GPIO_Init(TRACK1_PORT, &gpio);
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
 * 三次跨周期多数滤波
 *
 * 每一路传感器最近 3 次采样中至少有 2 次为黑线，才输出黑线。
 * 首次调用直接用当前值填满历史，避免车辆启动时等待两帧。
 * ================================================================ */

uint8_t TRACK_FilterSample(uint8_t raw)
{
    static uint8_t history[3];
    static uint8_t history_index = 0;
    static uint8_t initialized = 0;
    uint8_t filtered = 0;
    uint8_t i;

    raw &= 0x1F;

    if (!initialized) {
        history[0] = raw;
        history[1] = raw;
        history[2] = raw;
        initialized = 1;
        return raw;
    }

    history[history_index] = raw;
    history_index = (uint8_t)((history_index + 1U) % 3U);

    for (i = 0; i < 5; i++) {
        uint8_t mask = (uint8_t)(1U << i);
        uint8_t votes = 0;

        if (history[0] & mask) votes++;
        if (history[1] & mask) votes++;
        if (history[2] & mask) votes++;

        if (votes >= 2) {
            filtered |= mask;
        }
    }

    return filtered;
}

/* ================================================================
 * 计算位置偏差 (-4 ~ +4)
 *
 *   0  = 居中 (OUT3 压线, 或对称压线)
 *  <0  = 偏右 (左边传感器压线, 需左转修正)
 *  >0  = 偏左 (右边传感器压线, 需右转修正)
 * ================================================================ */

int8_t TRACK_GetErrorFromRaw(uint8_t raw)
{
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

int8_t TRACK_GetError(void)
{
    return TRACK_GetErrorFromRaw(TRACK_Read());
}

/* ================================================================
 * 从多个互不相连的黑色区域中选择真正的赛道
 *
 * 例如上一帧赛道在 OUT5，本帧读到 OUT1 + OUT5 (10001)，
 * 则保留离上一位置更近的 OUT5，将 OUT1 视为阴影或污渍。
 * 连续的一片黑色仍作为同一个区域保留。
 * ================================================================ */

uint8_t TRACK_SelectLikelyLine(uint8_t raw, uint8_t previous_raw)
{
    uint8_t i = 0;
    uint8_t component_count = 0;
    uint8_t best_mask = raw;
    int8_t previous_error;
    int8_t best_distance = 127;

    raw &= 0x1F;
    previous_raw &= 0x1F;

    if (raw == 0x00 || raw == 0x1F || previous_raw == 0x00) {
        return raw;
    }

    previous_error = TRACK_GetErrorFromRaw(previous_raw);

    while (i < 5) {
        uint8_t component_mask = 0;
        int8_t component_error;
        int8_t distance;

        while (i < 5 && !(raw & (1U << i))) {
            i++;
        }

        if (i >= 5) {
            break;
        }

        while (i < 5 && (raw & (1U << i))) {
            component_mask |= (uint8_t)(1U << i);
            i++;
        }

        component_count++;
        component_error = TRACK_GetErrorFromRaw(component_mask);
        distance = component_error - previous_error;
        if (distance < 0) {
            distance = -distance;
        }

        if (distance < best_distance) {
            best_distance = distance;
            best_mask = component_mask;
        }
    }

    return (component_count > 1) ? best_mask : raw;
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
