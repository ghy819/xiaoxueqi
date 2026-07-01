/**
  * @file    motor.c
  * @brief   两相步进电机驱动 — 基于 STM32 标准外设库 (SPL)
  *          双电机 PWM 调速 + 方向控制 (FI/BI 接口)
  *
  *          定时器映射 (STM32F103RCT6):
  *            TIM4_CH1 = PB6 (右 BI)     TIM4_CH3 = PB8 (左 BI)
  *            TIM4_CH2 = PB7 (右 FI)     TIM4_CH4 = PB9 (左 FI)
  *            TIM4 默认映射即可, 无需重映射到 PD12~PD15
  */

#include "motor.h"

/* ================================================================
 * 私有数据结构
 * ================================================================ */

typedef struct {
    uint8_t  fi_ch;          /* TIM4 通道号 (FI pin) */
    uint8_t  bi_ch;          /* TIM4 通道号 (BI pin) */
    int16_t  current_speed;  /* -100 ~ +100 */
} Motor_Handle;

static Motor_Handle g_motors[MOTOR_COUNT];

/* ================================================================
 * 装载电机配置 (TIM4 通道对应关系)
 * ================================================================ */

static void Motor_LoadConfig(void)
{
    /* 左侧电机: FI=PB9=TIM4_CH4,  BI=PB8=TIM4_CH3 */
    g_motors[0].fi_ch = 4;
    g_motors[0].bi_ch = 3;

    /* 右侧电机: FI=PB7=TIM4_CH2,  BI=PB6=TIM4_CH1 */
    g_motors[1].fi_ch = 2;
    g_motors[1].bi_ch = 1;

    for (int i = 0; i < MOTOR_COUNT; i++) {
        g_motors[i].current_speed = 0;
    }
}

/* ================================================================
 * TIM4 通道 → CCR 寄存器 写操作
 * ================================================================ */

static void TIM4_SetCompare(uint8_t ch, uint16_t pulse)
{
    switch (ch) {
        case 1:  TIM_SetCompare1(TIM4, pulse); break;
        case 2:  TIM_SetCompare2(TIM4, pulse); break;
        case 3:  TIM_SetCompare3(TIM4, pulse); break;
        case 4:  TIM_SetCompare4(TIM4, pulse); break;
        default: break;
    }
}

/* ================================================================
 * GPIO 初始化 (PB6~PB9 → AF_PP, TIM4 PWM 输出)
 * ================================================================ */

static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef gpio;

    /* 使能 GPIOB 和 AFIO 时钟
     * AFIO 时钟必须开启, 否则 GPIO_PinRemapConfig 访问 AFIO 寄存器
     * 会触发 HardFault, 导致程序卡死、电机不动 */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);

    /* 确保 TIM4 使用默认映射 PB6~PB9, 而非重映射到 PD12~PD15 */
    GPIO_PinRemapConfig(GPIO_Remap_TIM4, DISABLE);

    /* FI/BI 引脚 → 复用推挽输出 (TIM4 PWM) */
    GPIO_StructInit(&gpio);
    gpio.GPIO_Mode  = GPIO_Mode_AF_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_Pin   = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;
    GPIO_Init(GPIOB, &gpio);
}

/* ================================================================
 * TIM4 初始化 (四通道 PWM, 1kHz)
 * ================================================================ */

static void MX_TIM4_Init(void)
{
    TIM_TimeBaseInitTypeDef tim_base;
    TIM_OCInitTypeDef       oc;

    /* 使能 TIM4 时钟 (APB1, 72MHz) */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

    /* ---- 时基: 72MHz / 72 = 1MHz, 1MHz / 1000 = 1kHz PWM ---- */
    TIM_TimeBaseStructInit(&tim_base);
    tim_base.TIM_Prescaler         = MOTOR_PWM_PSC;       /* 71 */
    tim_base.TIM_CounterMode       = TIM_CounterMode_Up;
    tim_base.TIM_Period            = MOTOR_PWM_PERIOD;    /* 999 */
    tim_base.TIM_ClockDivision     = TIM_CKD_DIV1;
    tim_base.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM4, &tim_base);

    /* ---- 四通道 PWM1 模式, 初始占空比 0% ---- */
    TIM_OCStructInit(&oc);
    oc.TIM_OCMode      = TIM_OCMode_PWM1;
    oc.TIM_OutputState = TIM_OutputState_Enable;
    oc.TIM_Pulse       = 0;
    oc.TIM_OCPolarity  = TIM_OCPolarity_High;

    TIM_OC1Init(TIM4, &oc);  /* PB6 — 右 BI */
    TIM_OC2Init(TIM4, &oc);  /* PB7 — 右 FI */
    TIM_OC3Init(TIM4, &oc);  /* PB8 — 左 BI */
    TIM_OC4Init(TIM4, &oc);  /* PB9 — 左 FI */

    /* 预装载 + 启动 */
    TIM_ARRPreloadConfig(TIM4, ENABLE);
    TIM_Cmd(TIM4, ENABLE);
}

/* ================================================================
 * 内部辅助
 * ================================================================ */

/** 将 -100~+100 映射到 0~ARR 脉冲值 */
static uint16_t SpeedToPulse(int16_t speed)
{
    if (speed == 0) return 0;
    if (speed < 0) speed = -speed;
    if (speed > MOTOR_SPEED_MAX) speed = MOTOR_SPEED_MAX;

    return (uint16_t)((uint32_t)speed * (MOTOR_PWM_PERIOD + 1) / MOTOR_SPEED_MAX);
}

/** 设置方向 + PWM 占空比 */
static void Motor_SetDirection(uint8_t id, int16_t speed)
{
    Motor_Handle *m = &g_motors[id];
    uint16_t pulse = SpeedToPulse(speed);

    if (speed > 0) {
        /* 正转: FI=PWM, BI=0 */
        TIM4_SetCompare(m->fi_ch, pulse);
        TIM4_SetCompare(m->bi_ch, 0);
    } else if (speed < 0) {
        /* 反转: FI=0, BI=PWM */
        TIM4_SetCompare(m->fi_ch, 0);
        TIM4_SetCompare(m->bi_ch, pulse);
    } else {
        /* 停止: FI=0, BI=0 */
        TIM4_SetCompare(m->fi_ch, 0);
        TIM4_SetCompare(m->bi_ch, 0);
    }
}

/* ================================================================
 * API 实现
 * ================================================================ */

void MOTOR_Init(void)
{
    Motor_LoadConfig();
    MX_GPIO_Init();
    MX_TIM4_Init();
}

void MOTOR_SetSpeed(uint8_t id, int16_t speed)
{
    if (id >= MOTOR_COUNT) return;

    if (speed >  MOTOR_SPEED_MAX) speed =  MOTOR_SPEED_MAX;
    if (speed <  MOTOR_SPEED_MIN) speed =  MOTOR_SPEED_MIN;

#if MOTOR_RIGHT_INVERT
    if (id == MOTOR_RIGHT) speed = -speed;
#endif

    g_motors[id].current_speed = speed;
    Motor_SetDirection(id, speed);
}

void MOTOR_Stop(uint8_t id)
{
    if (id >= MOTOR_COUNT) return;

    g_motors[id].current_speed = 0;
    Motor_SetDirection(id, 0);
}

void MOTOR_Brake(uint8_t id)
{
    if (id >= MOTOR_COUNT) return;

    Motor_Handle *m = &g_motors[id];
    g_motors[id].current_speed = 0;

    /* 急刹: FI=100%, BI=100% (两相导通锁死)
     * PWM1 模式: CCR > ARR → 输出恒高 (100% duty) */
    TIM4_SetCompare(m->fi_ch, MOTOR_PWM_PERIOD + 1);
    TIM4_SetCompare(m->bi_ch, MOTOR_PWM_PERIOD + 1);
}

void MOTOR_SetSpeeds(int16_t left, int16_t right)
{
    MOTOR_SetSpeed(MOTOR_LEFT,  left);
    MOTOR_SetSpeed(MOTOR_RIGHT, right);
}

int16_t MOTOR_GetSpeed(uint8_t id)
{
    if (id >= MOTOR_COUNT) return 0;
    return g_motors[id].current_speed;
}