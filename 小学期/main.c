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

/* ================================================================
 * NVIC 配置
 * ================================================================ */

static void NVIC_Configuration(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
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

    while (1)
    {
        /* 全黑线检测: 5 路全压线 → 停止 */
        if (TRACK_Read() == 0x1F) {
            MOTOR_SetSpeeds(0, 0);
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

        /* ---- 电机差速 (辅助修正) ---- */
        left_speed  = base_speed - correction / 2;
        right_speed = base_speed + correction / 2;

        if (left_speed  >  100) left_speed  =  100;
        if (left_speed  < -100) left_speed  = -100;
        if (right_speed >  100) right_speed =  100;
        if (right_speed < -100) right_speed = -100;

        MOTOR_SetSpeeds(left_speed, right_speed);
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
    MOTOR_Init();
    SERVO_Init();
    TRACK_Init();

    /* 上电等待 3 秒 */
    Delay_ms(3000);

    Track_Run();

    while (1);
}