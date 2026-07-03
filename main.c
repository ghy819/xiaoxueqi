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

static void Telemetry_Send(uint8_t track_raw, int8_t error,
                           uint8_t curve_mode)
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
    UART_SendString(" MODE=");
    UART_SendChar(curve_mode ? 'C' : 'S');
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
    int8_t  last_direction = 0;
    int8_t  inner_error_direction = 0;
    int8_t  abs_error;
    int16_t correction, derivative;
    int16_t inner_ratio;
    int16_t diff_ratio;
    int16_t servo_target, servo_angle = 90;
    int16_t base_speed = TRACK_BASE_SPEED;
    int16_t target_base_speed, left_speed, right_speed;
    uint8_t sensor_raw, filtered_raw, track_raw;
    uint8_t control_raw = 0x04;
    uint8_t lost_count = 0;
    uint8_t inner_error_count = 0;
    uint8_t full_black_count = 0;
    uint8_t curve_mode = 0;
    uint8_t curve_enter_count = 0;
    uint8_t straight_enter_count = 0;
    uint8_t outer_candidate_raw = 0;
    uint8_t outer_candidate_count = 0;
    uint32_t last_telemetry = Delay_GetTick();

    while (1)
    {
        sensor_raw = TRACK_Read();
        filtered_raw = TRACK_FilterSample(sensor_raw);

        /* 短暂全黑可能来自阴影或污渍，连续约 40ms 才确认停止。
         * 确认前保持上一帧可靠的赛道位置，避免突然直行或急转。 */
        if (filtered_raw == 0x1F) {
            if (full_black_count < TRACK_FULL_BLACK_CONFIRM_COUNT) {
                full_black_count++;
            }

            if (full_black_count >= TRACK_FULL_BLACK_CONFIRM_COUNT) {
                MOTOR_SetSpeeds(0, 0);
                Telemetry_Send(sensor_raw, error, curve_mode);
                break;
            }

            track_raw = control_raw;
        } else {
            full_black_count = 0;

            /* 若同时出现多块互不相连的黑色，只跟随最接近上一帧
             * 赛道位置的一块，过滤远处阴影和污渍。 */
            track_raw = TRACK_SelectLikelyLine(filtered_raw, control_raw);
        }

        /* 直道中突然单独出现 OUT1/OUT5，可能是阴影或污渍。
         * 连续确认前保持上一位置；弯道模式不做此延迟。 */
        if (!curve_mode && (track_raw == 0x01 || track_raw == 0x10)) {
            if (outer_candidate_raw == track_raw) {
                if (outer_candidate_count <
                    TRACK_STRAIGHT_OUTER_CONFIRM_COUNT) {
                    outer_candidate_count++;
                }
            } else {
                outer_candidate_raw = track_raw;
                outer_candidate_count = 1;
            }

            if (outer_candidate_count <
                TRACK_STRAIGHT_OUTER_CONFIRM_COUNT) {
                track_raw = control_raw;
            }
        } else {
            outer_candidate_raw = 0;
            outer_candidate_count = 0;
        }

        /* 一次采样同时用于偏差和动作判断，避免前后两次读取不一致。
         * 短暂全白保持上一判断；连续全白才向赛道最后所在侧救车。 */
        prev_error = error;
        if (track_raw != 0x00) {
            lost_count = 0;
            control_raw = track_raw;
            error = TRACK_GetErrorFromRaw(track_raw);

            if (error < 0) {
                last_direction = -1;
            } else if (error > 0) {
                last_direction = 1;
            }
        } else {
            if (lost_count < TRACK_LOST_CONFIRM_COUNT) {
                lost_count++;
            }

            prev_error = error;
            if (lost_count >= TRACK_LOST_CONFIRM_COUNT) {
                if (last_direction < 0) {
                    error = -4;
                    prev_error = error;
                    control_raw = 0x01;
                } else if (last_direction > 0) {
                    error = 4;
                    prev_error = error;
                    control_raw = 0x10;
                }
            }
        }

        /* OUT2/OUT4 连续出现时逐步增加转向量。
         * 直线上的短暂偏差只轻微纠正，持续偏差才按弯道处理。 */
        if (error == -2 || error == 2) {
            if (inner_error_direction == error) {
                if (inner_error_count < 3) {
                    inner_error_count++;
                }
            } else {
                inner_error_direction = error;
                inner_error_count = 1;
            }
        } else {
            inner_error_direction = 0;
            inner_error_count = 0;
        }

        /* 顺时针椭圆赛道状态判断:
         * OUT4 连续出现或 OUT5 出现时进入弯道；
         * 回到中心并稳定约 60ms 后才恢复直道，防止状态来回切换。 */
        if (!curve_mode) {
            straight_enter_count = 0;
            if (error >= 3) {
                curve_mode = 1;
                curve_enter_count = 0;
            } else if (error >= 2) {
                if (curve_enter_count < TRACK_CURVE_ENTER_COUNT) {
                    curve_enter_count++;
                }
                if (curve_enter_count >= TRACK_CURVE_ENTER_COUNT) {
                    curve_mode = 1;
                    curve_enter_count = 0;
                }
            } else {
                curve_enter_count = 0;
            }
        } else {
            /* 采用累计证据而不是要求连续完全居中。
             * 顺时针弯道中的右侧偏差会扣分；居中和左侧回正会加分。
             * 偶发一次 OUT2/OUT4 不再把直道判断进度全部清零。 */
            if (error >= 2) {
                if (straight_enter_count > 3) {
                    straight_enter_count -= 3;
                } else {
                    straight_enter_count = 0;
                }
            } else {
                uint8_t evidence_step = (error <= -2) ? 3 : 1;

                if (straight_enter_count + evidence_step >=
                    TRACK_STRAIGHT_ENTER_COUNT) {
                    curve_mode = 0;
                    straight_enter_count = 0;
                } else {
                    straight_enter_count += evidence_step;
                }
            }
        }

        /* ---- PD 控制 ---- */
        correction = TRACK_GetCorrection(error);               /* P 项        */

        /* 中心附近只做柔和的比例纠偏，避免误差在 -1/0/+1 间变化时
         * 微分项把舵机反复推向两侧。进入明显弯道后才启用微分。 */
        if (error >= -1 && error <= 1 &&
            prev_error >= -1 && prev_error <= 1) {
            derivative = 0;
        } else {
            derivative = (int16_t)(error - prev_error) * TRACK_KD;
            if (derivative > TRACK_D_LIMIT) {
                derivative = TRACK_D_LIMIT;
            }
            if (derivative < -TRACK_D_LIMIT) {
                derivative = -TRACK_D_LIMIT;
            }
        }
        correction += derivative;

        /* OUT2/OUT4 靠近中线，降低其转向幅度以抑制直线抖动。
         * OUT1/OUT5 的极限纠偏幅度保持不变。 */
        if (!curve_mode && error >= -1 && error <= 1) {
            correction =
                correction * TRACK_STRAIGHT_SMALL_STEER_RATIO / 100;
        } else if (!curve_mode && (error == -2 || error == 2)) {
            correction =
                correction * TRACK_STRAIGHT_INNER_STEER_RATIO / 100;
        } else if (error == -2) {
            inner_ratio = TRACK_OUT2_STEER_RATIO;
            if (inner_error_count == 1) {
                inner_ratio = TRACK_INNER_STEER_INITIAL_RATIO;
            } else if (inner_error_count == 2) {
                inner_ratio = (TRACK_INNER_STEER_INITIAL_RATIO +
                               TRACK_OUT2_STEER_RATIO) / 2;
            }
            correction = correction * inner_ratio / 100;
        } else if (error == 2) {
            inner_ratio = TRACK_OUT4_STEER_RATIO;
            if (inner_error_count == 1) {
                inner_ratio = TRACK_INNER_STEER_INITIAL_RATIO;
            } else if (inner_error_count == 2) {
                inner_ratio = (TRACK_INNER_STEER_INITIAL_RATIO +
                               TRACK_OUT4_STEER_RATIO) / 2;
            }
            correction = correction * inner_ratio / 100;
        }

        /* ---- 弯道分级减速 ---- */
        abs_error = (error > 0) ? error : -error;
        if (!curve_mode && abs_error <= 2) {
            target_base_speed = TRACK_BASE_SPEED;
        } else if (abs_error >= 2) {
            target_base_speed = TRACK_BASE_SPEED *
                                TRACK_CURVE_SHARP_RATIO / 100;
        } else if (abs_error == 1) {
            target_base_speed = TRACK_BASE_SPEED *
                                TRACK_CURVE_ENTRY_RATIO / 100;
        } else {
            target_base_speed = TRACK_BASE_SPEED;
        }

        if (target_base_speed < base_speed) {
            base_speed = target_base_speed;
        } else if (base_speed < target_base_speed) {
            base_speed += TRACK_SPEED_RECOVERY_STEP;
            if (base_speed > target_base_speed) {
                base_speed = target_base_speed;
            }
        }

        /* ---- 舵机转向 (低通滤波, 消除抖动) ---- */
        servo_target = 90 + correction;
        if (servo_target < TRACK_SERVO_MIN_ANGLE) {
            servo_target = TRACK_SERVO_MIN_ANGLE;
        }
        if (servo_target > TRACK_SERVO_MAX_ANGLE) {
            servo_target = TRACK_SERVO_MAX_ANGLE;
        }

        /* 轻微偏差、OUT2/OUT4 触发及随后回中时缓慢跟随；
         * OUT1/OUT5 仍使用原来的快速响应。 */
        if (!curve_mode &&
            ((error >= -2 && error <= 2 && error != 0) ||
             (error == 0 && prev_error >= -2 && prev_error <= 2 &&
              prev_error != 0))) {
            servo_angle =
                (servo_angle * TRACK_STRAIGHT_SERVO_OLD_WEIGHT +
                 servo_target * TRACK_STRAIGHT_SERVO_NEW_WEIGHT) /
                (TRACK_STRAIGHT_SERVO_OLD_WEIGHT +
                 TRACK_STRAIGHT_SERVO_NEW_WEIGHT);
        } else if ((error >= -2 && error <= 2 && error != 0) ||
            (error == 0 && prev_error >= -2 && prev_error <= 2 &&
             prev_error != 0)) {
            servo_angle =
                (servo_angle * TRACK_INNER_SERVO_OLD_WEIGHT +
                 servo_target * TRACK_INNER_SERVO_NEW_WEIGHT) /
                (TRACK_INNER_SERVO_OLD_WEIGHT +
                 TRACK_INNER_SERVO_NEW_WEIGHT);
        } else {
            servo_angle = (servo_angle * 2 + servo_target * 3) / 5;
        }
        SERVO_SetAngle((uint8_t)servo_angle);

        /* ---- 电机差速及边缘前进纠偏 ----
         * OUT1 压线: 线在车辆左侧，左轮慢、右轮快
         * OUT5 压线: 线在车辆右侧，左轮快、右轮慢
         * 两轮始终向前，避免原地旋转造成车辆停顿。 */
        if ((control_raw & 0x01) && !(control_raw & 0x10)) {
            left_speed  = TRACK_EDGE_INNER_SPEED;
            right_speed = TRACK_EDGE_OUTER_SPEED;
        } else if ((control_raw & 0x10) && !(control_raw & 0x01)) {
            left_speed  = TRACK_EDGE_OUTER_SPEED;
            right_speed = TRACK_EDGE_INNER_SPEED;
        } else {
            diff_ratio = curve_mode ?
                         TRACK_DIFF_RATIO : TRACK_STRAIGHT_DIFF_RATIO;
            left_speed  = base_speed +
                          correction * diff_ratio / 100;
            right_speed = base_speed -
                          correction * diff_ratio / 100;
        }

        if (left_speed  >  100) left_speed  =  100;
        if (left_speed  < -100) left_speed  = -100;
        if (right_speed >  100) right_speed =  100;
        if (right_speed < -100) right_speed = -100;

        MOTOR_SetSpeeds(left_speed, right_speed);

        if ((Delay_GetTick() - last_telemetry) >= TELEMETRY_INTERVAL_MS) {
            last_telemetry = Delay_GetTick();
            Telemetry_Send(sensor_raw, error, curve_mode);
        }

        Delay_ms(TRACK_CONTROL_PERIOD_MS);
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
