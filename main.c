/**
  * @file    main.c
  * @brief   五路循迹控制：正常 PID、锐角强制转向、独立丢线搜索
  */

#include "stm32f10x.h"
#include "motor.h"
#include "delay.h"
#include "servo.h"
#include "track.h"
#include "uart.h"
#include "encoder.h"
#include "bluetooth.h"
#include "oled.h"

#define TELEMETRY_INTERVAL_MS             50
#define BLUETOOTH_TELEMETRY_INTERVAL_MS   50
#define OLED_UPDATE_INTERVAL_MS          100

typedef enum {
    TRACK_STATE_NORMAL = 0,
    TRACK_STATE_SHARP_LEFT,
    TRACK_STATE_SHARP_RIGHT,
    TRACK_STATE_LOST
} TrackState;

static char Track_StateChar(TrackState state)
{
    switch (state) {
        case TRACK_STATE_SHARP_LEFT:  return 'L';
        case TRACK_STATE_SHARP_RIGHT: return 'R';
        case TRACK_STATE_LOST:        return 'X';
        default:                      return 'N';
    }
}

static int16_t ClampSpeed(int16_t speed)
{
    if (speed > 100)  return 100;
    if (speed < -100) return -100;
    return speed;
}

static void NVIC_Configuration(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
}

/* ERR 使用放大 10 倍后的定点误差；MODE: N=正常,L/R=锐角,X=丢线。 */
static void Telemetry_Send(uint8_t track_raw, int16_t error_x10,
                           TrackState state)
{
    int16_t encoder_left_delta = ENCODER_GetDelta(ENCODER_LEFT);
    int16_t encoder_right_delta = ENCODER_GetDelta(ENCODER_RIGHT);
    uint8_t i;

    UART_SendString("TRK=");
    for (i = 0; i < 5; i++) {
        UART_SendChar((track_raw & (1U << i)) ? '1' : '0');
    }
    UART_SendString(" ERR=");
    UART_SendInt32(error_x10);
    UART_SendString(" MODE=");
    UART_SendChar(Track_StateChar(state));
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

static void Bluetooth_Telemetry_Send(uint8_t track_raw, int16_t error_x10,
                                     TrackState state)
{
    static int32_t last_encoder_left = 0;
    static int32_t last_encoder_right = 0;
    int32_t encoder_left = ENCODER_GetTotal(ENCODER_LEFT);
    int32_t encoder_right = ENCODER_GetTotal(ENCODER_RIGHT);
    int32_t encoder_left_delta = encoder_left - last_encoder_left;
    int32_t encoder_right_delta = encoder_right - last_encoder_right;
    uint8_t i;

    last_encoder_left = encoder_left;
    last_encoder_right = encoder_right;

    BLUETOOTH_SendString("TRK=");
    for (i = 0; i < 5; i++) {
        BLUETOOTH_SendChar((track_raw & (1U << i)) ? '1' : '0');
    }
    BLUETOOTH_SendString(" ERR=");
    BLUETOOTH_SendInt32(error_x10);
    BLUETOOTH_SendString(" MODE=");
    BLUETOOTH_SendChar(Track_StateChar(state));
    BLUETOOTH_SendString(" ENC_L=");
    BLUETOOTH_SendInt32(encoder_left);
    BLUETOOTH_SendString(" DL=");
    BLUETOOTH_SendInt32(encoder_left_delta);
    BLUETOOTH_SendString(" ENC_R=");
    BLUETOOTH_SendInt32(encoder_right);
    BLUETOOTH_SendString(" DR=");
    BLUETOOTH_SendInt32(encoder_right_delta);
    BLUETOOTH_SendString(" SERVO=");
    BLUETOOTH_SendInt32(SERVO_GetAngle());
    BLUETOOTH_SendString(" PULSE=");
    BLUETOOTH_SendInt32(SERVO_GetPulse());
    BLUETOOTH_SendString(" MOTOR_L=");
    BLUETOOTH_SendInt32(MOTOR_GetSpeed(MOTOR_LEFT));
    BLUETOOTH_SendString(" MOTOR_R=");
    BLUETOOTH_SendInt32(MOTOR_GetSpeed(MOTOR_RIGHT));
    BLUETOOTH_SendString("\r\n");
}

/* ================================================================
 * 正常循迹
 *
 * 五路权值为 -2,-1,0,+1,+2，内部把平均误差放大 10 倍，
 * 从而保留 S2+S3 等组合产生的 0.5 级误差。PID 只在本状态运行。
 * ================================================================ */
static void Track_NormalControl(int16_t error_x10, int16_t *previous_error,
                                int16_t *integral, int16_t *base_speed,
                                int16_t *servo_angle)
{
    int16_t derivative;
    int16_t correction;
    int16_t target_speed;
    int16_t servo_target;
    int16_t left_speed;
    int16_t right_speed;
    int16_t abs_error;

    *integral += error_x10;
    if (*integral > TRACK_INTEGRAL_LIMIT) {
        *integral = TRACK_INTEGRAL_LIMIT;
    } else if (*integral < -TRACK_INTEGRAL_LIMIT) {
        *integral = -TRACK_INTEGRAL_LIMIT;
    }
    if (error_x10 == 0) {
        *integral = (int16_t)(*integral * 3 / 4);
    }

    derivative = error_x10 - *previous_error;
    correction = (int16_t)(TRACK_PID_KP * error_x10 / 10);
    correction += (int16_t)(TRACK_PID_KD * derivative / 10);
    correction += (int16_t)(*integral / TRACK_PID_KI_DIV);
    if (correction > TRACK_CORRECTION_LIMIT) {
        correction = TRACK_CORRECTION_LIMIT;
    } else if (correction < -TRACK_CORRECTION_LIMIT) {
        correction = -TRACK_CORRECTION_LIMIT;
    }
    *previous_error = error_x10;

    abs_error = (error_x10 >= 0) ? error_x10 : -error_x10;
    target_speed = (abs_error <= TRACK_STRAIGHT_ERROR_X10) ?
                   TRACK_STRAIGHT_SPEED : TRACK_CURVE_SPEED;

    /* 入弯立即减速，出弯缓慢提速，避免速度阶跃导致摆动。 */
    if (*base_speed > target_speed) {
        *base_speed = target_speed;
    } else if (*base_speed < target_speed) {
        *base_speed += TRACK_SPEED_RECOVERY_STEP;
        if (*base_speed > target_speed) {
            *base_speed = target_speed;
        }
    }

    servo_target = TRACK_SERVO_CENTER_ANGLE + correction;
    if (servo_target < TRACK_SERVO_MIN_ANGLE) {
        servo_target = TRACK_SERVO_MIN_ANGLE;
    } else if (servo_target > TRACK_SERVO_MAX_ANGLE) {
        servo_target = TRACK_SERVO_MAX_ANGLE;
    }
    *servo_angle =
        (int16_t)((*servo_angle * TRACK_SERVO_FILTER_OLD +
                   servo_target * TRACK_SERVO_FILTER_NEW) /
                  (TRACK_SERVO_FILTER_OLD + TRACK_SERVO_FILTER_NEW));
    SERVO_SetAngle((uint8_t)*servo_angle);

    left_speed = *base_speed +
                 correction * TRACK_DIFF_RATIO / 100;
    right_speed = *base_speed -
                  correction * TRACK_DIFF_RATIO / 100;
    MOTOR_SetSpeeds(ClampSpeed(left_speed), ClampSpeed(right_speed));
}

/* 锐角控制完全覆盖 PID：低速、舵机打满、内轮反转辅助掉头。 */
static void Track_SharpControl(TrackState state, int16_t *servo_angle)
{
    if (state == TRACK_STATE_SHARP_LEFT) {
        *servo_angle = TRACK_SERVO_MIN_ANGLE;
        SERVO_SetAngle((uint8_t)*servo_angle);
        MOTOR_SetSpeeds(-(TRACK_SHARP_SPEED / 2), TRACK_SHARP_SPEED);
    } else {
        *servo_angle = TRACK_SERVO_MAX_ANGLE;
        SERVO_SetAngle((uint8_t)*servo_angle);
        MOTOR_SetSpeeds(TRACK_SHARP_SPEED, -(TRACK_SHARP_SPEED / 2));
    }
}

/* 丢线搜索独立于 PID；沿最后一次非零误差方向原地旋转。 */
static void Track_LostControl(int16_t last_error_x10, int16_t *servo_angle)
{
    if (last_error_x10 < 0) {
        *servo_angle = TRACK_SERVO_MIN_ANGLE;
        SERVO_SetAngle((uint8_t)*servo_angle);
        MOTOR_SetSpeeds(-TRACK_SEARCH_SPEED, TRACK_SEARCH_SPEED);
    } else {
        *servo_angle = TRACK_SERVO_MAX_ANGLE;
        SERVO_SetAngle((uint8_t)*servo_angle);
        MOTOR_SetSpeeds(TRACK_SEARCH_SPEED, -TRACK_SEARCH_SPEED);
    }
}

/* ================================================================
 * 状态机优先级
 *
 *   已确认锐角 -> 强制锐角动作（覆盖 PID）
 *   已确认丢线 -> 独立旋转搜索
 *   其余       -> 正常 PID
 *
 * 进入、退出均需要连续多帧证据；锐角还有最短保持时间。
 * ================================================================ */
static void Track_Run(void)
{
    TrackState state = TRACK_STATE_NORMAL;
    int16_t error_x10 = 0;
    int16_t previous_error = 0;
    int16_t last_error_x10 = 10;
    int16_t integral = 0;
    int16_t base_speed = TRACK_STRAIGHT_SPEED;
    int16_t servo_angle = TRACK_SERVO_CENTER_ANGLE;
    uint8_t sensor_raw;
    uint8_t filtered_raw;
    uint8_t track_raw;
    uint8_t reliable_raw = 0x04;
    uint8_t left_sharp_count = 0;
    uint8_t right_sharp_count = 0;
    uint8_t lost_count = 0;
    uint8_t reacquire_count = 0;
    uint8_t sharp_hold_count = 0;
    uint8_t sharp_exit_count = 0;
    uint8_t full_black_count = 0;
    uint8_t left_sharp;
    uint8_t right_sharp;
    uint32_t last_telemetry = Delay_GetTick();
    uint32_t last_bluetooth_telemetry = Delay_GetTick();
    uint32_t last_oled_update = Delay_GetTick();

    while (1) {
        sensor_raw = TRACK_Read();
        filtered_raw = TRACK_FilterSample(sensor_raw);

        /* 全黑保留原项目的终点停车语义，并进行约 40 ms 确认。 */
        if (filtered_raw == 0x1F) {
            if (full_black_count < TRACK_FULL_BLACK_CONFIRM_COUNT) {
                full_black_count++;
            }
            if (full_black_count >= TRACK_FULL_BLACK_CONFIRM_COUNT) {
                MOTOR_SetSpeeds(0, 0);
                Telemetry_Send(sensor_raw, error_x10, state);
                break;
            }
            track_raw = reliable_raw;
        } else {
            full_black_count = 0;
            track_raw = TRACK_SelectLikelyLine(filtered_raw, reliable_raw);
        }

        /* 两侧同时满足时不判锐角，防止宽黑线/终点触发错误转向。 */
        left_sharp = (uint8_t)(((filtered_raw & 0x03) == 0x03) &&
                              ((filtered_raw & 0x18) != 0x18));
        right_sharp = (uint8_t)(((filtered_raw & 0x18) == 0x18) &&
                               ((filtered_raw & 0x03) != 0x03));

        if (left_sharp) {
            if (left_sharp_count < TRACK_SHARP_CONFIRM_COUNT) {
                left_sharp_count++;
            }
        } else {
            left_sharp_count = 0;
        }
        if (right_sharp) {
            if (right_sharp_count < TRACK_SHARP_CONFIRM_COUNT) {
                right_sharp_count++;
            }
        } else {
            right_sharp_count = 0;
        }

        if (filtered_raw == 0x00) {
            if (lost_count < TRACK_LOST_CONFIRM_COUNT) {
                lost_count++;
            }
        } else {
            lost_count = 0;
        }

        if (track_raw != 0x00 && filtered_raw != 0x1F) {
            reliable_raw = track_raw;
            error_x10 = TRACK_GetErrorX10(track_raw);
            if (error_x10 != 0) {
                last_error_x10 = error_x10;
            }
        }

        /* ---------- 状态切换（带迟滞） ---------- */
        if (state == TRACK_STATE_NORMAL) {
            /* 锐角判断先于丢线和 PID。 */
            if (left_sharp_count >= TRACK_SHARP_CONFIRM_COUNT) {
                state = TRACK_STATE_SHARP_LEFT;
                sharp_hold_count = 0;
                sharp_exit_count = 0;
                integral = 0;
            } else if (right_sharp_count >= TRACK_SHARP_CONFIRM_COUNT) {
                state = TRACK_STATE_SHARP_RIGHT;
                sharp_hold_count = 0;
                sharp_exit_count = 0;
                integral = 0;
            } else if (lost_count >= TRACK_LOST_CONFIRM_COUNT) {
                state = TRACK_STATE_LOST;
                reacquire_count = 0;
                integral = 0;
            }
        } else if (state == TRACK_STATE_LOST) {
            if (filtered_raw != 0x00 && filtered_raw != 0x1F) {
                if (reacquire_count < TRACK_REACQUIRE_COUNT) {
                    reacquire_count++;
                }
            } else {
                reacquire_count = 0;
            }

            if (reacquire_count >= TRACK_REACQUIRE_COUNT) {
                if (left_sharp_count >= TRACK_SHARP_CONFIRM_COUNT) {
                    state = TRACK_STATE_SHARP_LEFT;
                    sharp_hold_count = 0;
                    sharp_exit_count = 0;
                } else if (right_sharp_count >= TRACK_SHARP_CONFIRM_COUNT) {
                    state = TRACK_STATE_SHARP_RIGHT;
                    sharp_hold_count = 0;
                    sharp_exit_count = 0;
                } else {
                    state = TRACK_STATE_NORMAL;
                    previous_error = error_x10;
                    base_speed = TRACK_CURVE_SPEED;
                }
            }
        } else {
            if (sharp_hold_count < TRACK_SHARP_MIN_HOLD_COUNT) {
                sharp_hold_count++;
            }

            /* 锐角中若确认完全丢线，交给独立搜索状态。 */
            if (lost_count >= TRACK_LOST_CONFIRM_COUNT) {
                state = TRACK_STATE_LOST;
                reacquire_count = 0;
            } else if (state == TRACK_STATE_SHARP_LEFT &&
                       right_sharp_count >= TRACK_SHARP_CONFIRM_COUNT) {
                state = TRACK_STATE_SHARP_RIGHT;
                sharp_hold_count = 0;
                sharp_exit_count = 0;
            } else if (state == TRACK_STATE_SHARP_RIGHT &&
                       left_sharp_count >= TRACK_SHARP_CONFIRM_COUNT) {
                state = TRACK_STATE_SHARP_LEFT;
                sharp_hold_count = 0;
                sharp_exit_count = 0;
            } else {
                if ((state == TRACK_STATE_SHARP_LEFT && left_sharp) ||
                    (state == TRACK_STATE_SHARP_RIGHT && right_sharp)) {
                    sharp_exit_count = 0;
                } else if (filtered_raw != 0x00) {
                    if (sharp_exit_count < TRACK_SHARP_EXIT_COUNT) {
                        sharp_exit_count++;
                    }
                }

                if (sharp_hold_count >= TRACK_SHARP_MIN_HOLD_COUNT &&
                    sharp_exit_count >= TRACK_SHARP_EXIT_COUNT) {
                    state = TRACK_STATE_NORMAL;
                    previous_error = error_x10;
                    integral = 0;
                    base_speed = TRACK_CURVE_SPEED;
                }
            }
        }

        /* ---------- 各状态动作彼此独立 ---------- */
        if (state == TRACK_STATE_SHARP_LEFT ||
            state == TRACK_STATE_SHARP_RIGHT) {
            Track_SharpControl(state, &servo_angle);
        } else if (state == TRACK_STATE_LOST) {
            Track_LostControl(last_error_x10, &servo_angle);
        } else {
            /* 丢线确认期间沿用最后可靠误差，不让 PID 突然回中。 */
            if (filtered_raw == 0x00) {
                error_x10 = TRACK_GetErrorX10(reliable_raw);
            }
            Track_NormalControl(error_x10, &previous_error, &integral,
                                &base_speed, &servo_angle);
        }

        if ((Delay_GetTick() - last_telemetry) >= TELEMETRY_INTERVAL_MS) {
            last_telemetry = Delay_GetTick();
            Telemetry_Send(sensor_raw, error_x10, state);
        }
        if ((Delay_GetTick() - last_bluetooth_telemetry) >=
            BLUETOOTH_TELEMETRY_INTERVAL_MS) {
            last_bluetooth_telemetry = Delay_GetTick();
            Bluetooth_Telemetry_Send(sensor_raw, error_x10, state);
        }
        if ((Delay_GetTick() - last_oled_update) >= OLED_UPDATE_INTERVAL_MS) {
            last_oled_update = Delay_GetTick();
            OLED_UpdateDashboard(MOTOR_GetSpeed(MOTOR_LEFT),
                                 MOTOR_GetSpeed(MOTOR_RIGHT),
                                 ENCODER_GetTotal(ENCODER_LEFT),
                                 ENCODER_GetTotal(ENCODER_RIGHT),
                                 (uint8_t)(state != TRACK_STATE_NORMAL));
        }
        OLED_RefreshStep();
        Delay_ms(TRACK_CONTROL_PERIOD_MS);
    }
}

int main(void)
{
    NVIC_Configuration();

    SysTick_Init();
    UART_Init();
    BLUETOOTH_Init();
    ENCODER_Init();
    MOTOR_Init();
    SERVO_Init();
    TRACK_Init();
    OLED_Init();

    UART_SendString("READY USART1 115200\r\n");
    BLUETOOTH_SendString("READY USART3 115200\r\n");
    OLED_UpdateDashboard(0, 0, 0, 0, 0);
    OLED_Refresh();

    Delay_ms(3000);
    Track_Run();

    while (1);
}
