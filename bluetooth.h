#ifndef __BLUETOOTH_H
#define __BLUETOOTH_H

#include "stm32f10x.h"
#include <stdint.h>

/* USART3: PB10=TX, PB11=RX, 115200-8-N-1 */
#define BLUETOOTH_BAUDRATE 115200

void BLUETOOTH_Init(void);
void BLUETOOTH_SendChar(char ch);
void BLUETOOTH_SendString(const char *text);
void BLUETOOTH_SendInt32(int32_t value);
uint8_t BLUETOOTH_Available(void);
uint8_t BLUETOOTH_ReadChar(char *ch);

#endif /* __BLUETOOTH_H */
