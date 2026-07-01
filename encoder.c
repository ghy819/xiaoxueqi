#include "encoder.h"

/* 调换某个编码器的 A/B 接线后，可将对应方向系数改为 -1。 */
#define ENCODER_LEFT_DIRECTION   1
#define ENCODER_RIGHT_DIRECTION  1

static volatile int32_t g_left_total = 0;
static volatile uint8_t g_left_previous = 0;
static int32_t g_left_last_report = 0;

static uint16_t g_right_last_count = 0;
static int32_t g_right_total = 0;
static int32_t g_right_last_report = 0;

/* 四倍频正交状态转换表：非法跳变计为 0。 */
static const int8_t QUADRATURE_TABLE[16] = {
     0, -1,  1,  0,
     1,  0,  0, -1,
    -1,  0,  0,  1,
     0,  1, -1,  0
};

static uint8_t Encoder_ReadLeftState(void)
{
    uint8_t state = 0;

    if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_6) != Bit_RESET) state |= 0x02;
    if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_7) != Bit_RESET) state |= 0x01;
    return state;
}

static void Encoder_RightTIM2_Init(void)
{
    TIM_TimeBaseInitTypeDef time_base;
    TIM_ICInitTypeDef input_capture;

    TIM_TimeBaseStructInit(&time_base);
    time_base.TIM_Prescaler = 0;
    time_base.TIM_CounterMode = TIM_CounterMode_Up;
    time_base.TIM_Period = 0xFFFF;
    time_base.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM2, &time_base);

    TIM_EncoderInterfaceConfig(TIM2, TIM_EncoderMode_TI12,
                               TIM_ICPolarity_Rising,
                               TIM_ICPolarity_Rising);

    TIM_ICStructInit(&input_capture);
    input_capture.TIM_ICPrescaler = TIM_ICPSC_DIV1;
    input_capture.TIM_ICFilter = 6;

    input_capture.TIM_Channel = TIM_Channel_1;
    input_capture.TIM_ICSelection = TIM_ICSelection_DirectTI;
    TIM_ICInit(TIM2, &input_capture);

    input_capture.TIM_Channel = TIM_Channel_2;
    TIM_ICInit(TIM2, &input_capture);

    TIM_SetCounter(TIM2, 0);
    TIM_Cmd(TIM2, ENABLE);
}

static void Encoder_LeftEXTI_Init(void)
{
    EXTI_InitTypeDef exti;
    NVIC_InitTypeDef nvic;

    GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource6);
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource7);

    EXTI_StructInit(&exti);
    exti.EXTI_Line = EXTI_Line6 | EXTI_Line7;
    exti.EXTI_Mode = EXTI_Mode_Interrupt;
    exti.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
    exti.EXTI_LineCmd = ENABLE;
    EXTI_Init(&exti);

    nvic.NVIC_IRQChannel = EXTI9_5_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 1;
    nvic.NVIC_IRQChannelSubPriority = 0;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);
}

void ENCODER_Init(void)
{
    GPIO_InitTypeDef gpio;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA |
                           RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    GPIO_StructInit(&gpio);
    gpio.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_6 | GPIO_Pin_7;
    gpio.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOA, &gpio);

    g_left_previous = Encoder_ReadLeftState();
    Encoder_RightTIM2_Init();
    Encoder_LeftEXTI_Init();
}

int16_t ENCODER_GetDelta(uint8_t id)
{
    int32_t current;
    int32_t delta;

    if (id == ENCODER_LEFT) {
        current = g_left_total;
        delta = current - g_left_last_report;
        g_left_last_report = current;
    } else if (id == ENCODER_RIGHT) {
        uint16_t counter = TIM_GetCounter(TIM2);
        int16_t increment = (int16_t)(counter - g_right_last_count);

        g_right_last_count = counter;
        g_right_total += (int32_t)increment * ENCODER_RIGHT_DIRECTION;
        current = g_right_total;
        delta = current - g_right_last_report;
        g_right_last_report = current;
    } else {
        return 0;
    }

    if (delta > 32767) return 32767;
    if (delta < -32768) return -32768;
    return (int16_t)delta;
}

int32_t ENCODER_GetTotal(uint8_t id)
{
    if (id == ENCODER_LEFT) return g_left_total;
    if (id == ENCODER_RIGHT) return g_right_total;
    return 0;
}

void ENCODER_Reset(uint8_t id)
{
    if (id == ENCODER_LEFT) {
        g_left_total = 0;
        g_left_last_report = 0;
        g_left_previous = Encoder_ReadLeftState();
    } else if (id == ENCODER_RIGHT) {
        TIM_SetCounter(TIM2, 0);
        g_right_last_count = 0;
        g_right_total = 0;
        g_right_last_report = 0;
    }
}

void EXTI9_5_IRQHandler(void)
{
    uint8_t current;

    if ((EXTI->PR & (EXTI_Line6 | EXTI_Line7)) == 0) return;

    EXTI_ClearITPendingBit(EXTI_Line6 | EXTI_Line7);
    current = Encoder_ReadLeftState();
    g_left_total +=
        (int32_t)QUADRATURE_TABLE[(g_left_previous << 2) | current] *
        ENCODER_LEFT_DIRECTION;
    g_left_previous = current;
}
