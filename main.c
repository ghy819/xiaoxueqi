/**
  * @file    main.c
  * @brief   智能车主控 — 基于 STM32F103RCT6 + 标准外设库
  *          功能: 五路循迹黑线跟随
  */

#include "stm32f10x.h"
#include "motor.h"
#include "delay.h"
#include "servo.h"
#include "track.h"
#include "uart.h"
#include "encoder.h"

#define TELEMETRY_INTERVAL_MS  50

/* ================================================================
 * NVIC 配置
 * ================================================================ */

static void NVIC_Configuration(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
}

/* ================================================================
 * 串口遥测: 115200-8-N-1, USART1 TX=PA9 / RX=PA10
 * ================================================================ */

static void Telemetry_Send(uint8_t track_raw, int8_t error)
{
    int16_t encoder_left_delta = ENCODER_GetDelta(ENCODER_LEFT);
    int16_t encoder_right_delta = ENCODER_GetDelta(ENCODER_RIGHT);
    uint8_t i;

    UART_SendString("TRK=");
    for (i = 0; i < 5; i++) {
        UART_SendChar((track_raw & (1U << i)) ? '1' : '0');
    }

    UART_SendString(" ERR=");
    UART_SendInt32(error);
    UART_SendString(" ENC_L=");
    UART_SendInt32(ENCODER_GetTotal(ENCODER_LEFT));
    UART_SendString(" DL=");
    UART_SendInt32(encoder_left_delta);
    UART_SendString(" ENC_R=");
    UART_SendInt32(ENCODER_GetTotal(ENCODER_RIGHT));
    UART_SendString(" DR=");
    UART_SendInt32(encoder_right_delta);
    UART_SendString(" SERVO=");
    UART_SendInt32(SERVO_GetAngle());
    UART_SendString(" PULSE=");
    UART_SendInt32(SERVO_GetPulse());
    UART_SendString(" MOTOR_L=");
    UART_SendInt32(MOTOR_GetSpeed(MOTOR_LEFT));
    UART_SendString(" MOTOR_R=");
    UART_SendInt32(MOTOR_GetSpeed(MOTOR_RIGHT));
    UART_SendString("\r\n");
}

/* ================================================================
 * 循迹控制 — 舵机转向 + 电机差速辅助 
 * ================================================================ */

static void Track_Run(void)
{
    int8_t  error = 0, prev_error = 0;
    int16_t correction, derivative;
    int16_t servo_target, servo_angle = 90;
    int16_t base_speed, left_speed, right_speed;
    uint8_t track_raw;
    uint32_t last_telemetry = Delay_GetTick();

    while (1)
    {
        /* 全黑线检测: 5 路全压线 → 停止 */
        track_raw = TRACK_Read();
        if (track_raw == 0x1F) {
            MOTOR_SetSpeeds(0, 0);
            Telemetry_Send(track_raw, error);
            break;
        }

        prev_error = error;
        error      = TRACK_GetError();

        /* ---- PD 控制 ---- */
        correction = TRACK_GetCorrection(error);               /* P 项        */
        derivative = (int16_t)(error - prev_error) * TRACK_KD; /* D 项        */
        correction += derivative;

        /* ---- 弯道自动减速 ---- */
        if ((error > 0 ? error : -error) >= TRACK_CURVE_SLOW) {
            base_speed = TRACK_BASE_SPEED * TRACK_CURVE_RATIO / 100;
        } else {
            base_speed = TRACK_BASE_SPEED;
        }

        /* ---- 舵机转向 (低通滤波, 消除抖动) ---- */
        servo_target = 90 + correction;
        if (servo_target <  30) servo_target =  30;
        if (servo_target > 150) servo_target = 150;
        servo_angle = (servo_angle * 2 + servo_target * 3) / 5;
        SERVO_SetAngle((uint8_t)servo_angle);

        /* ---- 电机差速及边缘原地纠偏 ----
         * OUT1 压线: 线在车辆左侧，左轮后退、右轮前进
         * OUT5 压线: 线在车辆右侧，左轮前进、右轮后退 */
        if ((track_raw & 0x01) && !(track_raw & 0x10)) {
            left_speed  = -TRACK_PIVOT_SPEED;
            right_speed =  TRACK_PIVOT_SPEED;
        } else if ((track_raw & 0x10) && !(track_raw & 0x01)) {
            left_speed  =  TRACK_PIVOT_SPEED;
            right_speed = -TRACK_PIVOT_SPEED;
        } else {
            left_speed  = base_speed +
                          correction * TRACK_DIFF_RATIO / 100;
            right_speed = base_speed -
                          correction * TRACK_DIFF_RATIO / 100;
        }

        if (left_speed  >  100) left_speed  =  100;
        if (left_speed  < -100) left_speed  = -100;
        if (right_speed >  100) right_speed =  100;
        if (right_speed < -100) right_speed = -100;

        MOTOR_SetSpeeds(left_speed, right_speed);

        if ((Delay_GetTick() - last_telemetry) >= TELEMETRY_INTERVAL_MS) {
            last_telemetry = Delay_GetTick();
            Telemetry_Send(track_raw, error);
        }

        Delay_ms(2);
    }
}

/* ================================================================
 * 主函数
 * ================================================================ */

int main(void)
{
    NVIC_Configuration();

    SysTick_Init();
    UART_Init();
    ENCODER_Init();
    MOTOR_Init();
    SERVO_Init();
    TRACK_Init();

    UART_SendString("READY USART1 115200\r\n");

    /* 上电等待 3 秒 */
    Delay_ms(3000);

    Track_Run();

    while (1);
}
