#ifndef __UART_H
#define __UART_H

#include "stm32f10x.h"
#include <stdint.h>

/* USART1: PA9=TX, PA10=RX, 115200-8-N-1 */
void UART_Init(void);
void UART_SendChar(char ch);
void UART_SendString(const char *text);
void UART_SendInt32(int32_t value);

#endif /* __UART_H */
