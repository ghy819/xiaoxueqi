#ifndef __ENCODER_H
#define __ENCODER_H

#include "stm32f10x.h"
#include <stdint.h>

#define ENCODER_LEFT   0
#define ENCODER_RIGHT  1
#define ENCODER_COUNT  2

/*
 * 左编码器: PA6/PA7，使用 EXTI 双边沿正交解码
 * 右编码器: PA0/PA1 = TIM2_CH1/CH2，使用硬件编码器模式
 */
void    ENCODER_Init(void);
int16_t ENCODER_GetDelta(uint8_t id);
int32_t ENCODER_GetTotal(uint8_t id);
void    ENCODER_Reset(uint8_t id);

#endif /* __ENCODER_H */
