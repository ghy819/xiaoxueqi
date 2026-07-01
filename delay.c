/**
  * @file    delay.c
  * @brief   SysTick 微秒/毫秒延时模块
  *          基于 STM32F103 (72MHz, SysTick 1ms 中断)
  */

#include "delay.h"

/* ================================================================
 * 全局变量
 * ================================================================ */

static volatile uint32_t g_systick_ms = 0;

/* ================================================================
 * SysTick 中断处理
 * ================================================================ */

void SysTick_Handler(void)
{
    g_systick_ms++;
}

/* ================================================================
 * 毫秒延时
 * ================================================================ */

void Delay_ms(uint32_t ms)
{
    uint32_t start = g_systick_ms;
    while ((g_systick_ms - start) < ms);
}

uint32_t Delay_GetTick(void)
{
    return g_systick_ms;
}

/* ================================================================
 * SysTick 初始化
 * ================================================================ */

void SysTick_Init(void)
{
    /* SysTick 配置: 72MHz / 72000 = 1kHz (1ms 中断) */
    if (SysTick_Config(SystemCoreClock / 1000)) {
        while (1);  /* 配置失败则死循环 */
    }
    /* SysTick 中断优先级设为最低 */
    NVIC_SetPriority(SysTick_IRQn, 0x0F);
}
