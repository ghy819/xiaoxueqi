#ifndef __OLED_H
#define __OLED_H

#include "stm32f10x.h"
#include <stdint.h>

/*
 * 1.3 inch 128x64 SH1106 OLED, 4-wire software SPI
 *
 * OLED GND -> JP3-1 GND
 * OLED VCC -> JP3-2 3.3V
 * OLED D0  -> JP3-3 PB5  (SCLK)
 * OLED D1  -> JP3-4 PB4  (MOSI)
 * OLED RES -> JP3-5 PB3
 * OLED DC  -> JP3-6 PA15
 * OLED CS  -> GND
 */

void OLED_Init(void);
void OLED_Refresh(void);
void OLED_RefreshStep(void);
void OLED_UpdateDashboard(int16_t motor_left,
                          int16_t motor_right,
                          int32_t encoder_left,
                          int32_t encoder_right,
                          uint8_t curve_mode);

#endif /* __OLED_H */
