/**
  * @file    stm32f1xx_it.c
  * @brief   系统中断处理
  */

#include "stm32f1xx_it.h"

/* main.c 中定义的 tick 计数器递增函数 */
extern void SysTick_Handler(void);

void NMI_Handler(void)          {}
void HardFault_Handler(void)    { while (1); }
void MemManage_Handler(void)    { while (1); }
void BusFault_Handler(void)     { while (1); }
void UsageFault_Handler(void)   { while (1); }
void SVC_Handler(void)          {}
void DebugMon_Handler(void)     {}
void PendSV_Handler(void)       {}
