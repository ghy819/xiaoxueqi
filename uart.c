#include "uart.h"

#define UART_TX_BUFFER_SIZE  256

static volatile uint16_t g_tx_head = 0;
static volatile uint16_t g_tx_tail = 0;
static uint8_t g_tx_buffer[UART_TX_BUFFER_SIZE];

void UART_Init(void)
{
    GPIO_InitTypeDef gpio;
    USART_InitTypeDef usart;
    NVIC_InitTypeDef nvic;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA |
                           RCC_APB2Periph_USART1, ENABLE);

    GPIO_StructInit(&gpio);
    gpio.GPIO_Pin = GPIO_Pin_9;
    gpio.GPIO_Mode = GPIO_Mode_AF_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gpio);

    gpio.GPIO_Pin = GPIO_Pin_10;
    gpio.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &gpio);

    USART_StructInit(&usart);
    usart.USART_BaudRate = 115200;
    usart.USART_WordLength = USART_WordLength_8b;
    usart.USART_StopBits = USART_StopBits_1;
    usart.USART_Parity = USART_Parity_No;
    usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    usart.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART1, &usart);

    nvic.NVIC_IRQChannel = USART1_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 2;
    nvic.NVIC_IRQChannelSubPriority = 0;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);

    USART_Cmd(USART1, ENABLE);
}

void UART_SendChar(char ch)
{
    uint16_t next = (uint16_t)((g_tx_head + 1U) % UART_TX_BUFFER_SIZE);

    /* 缓冲区满时等待中断发送，避免丢失一行中的部分数据。 */
    while (next == g_tx_tail);

    g_tx_buffer[g_tx_head] = (uint8_t)ch;
    g_tx_head = next;
    USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
}

void UART_SendString(const char *text)
{
    while (*text != '\0') {
        UART_SendChar(*text++);
    }
}

void UART_SendInt32(int32_t value)
{
    char digits[11];
    uint8_t length = 0;
    uint32_t magnitude;

    if (value < 0) {
        UART_SendChar('-');
        magnitude = (uint32_t)(-(value + 1)) + 1U;
    } else {
        magnitude = (uint32_t)value;
    }

    do {
        digits[length++] = (char)('0' + magnitude % 10U);
        magnitude /= 10U;
    } while (magnitude != 0U);

    while (length > 0U) {
        UART_SendChar(digits[--length]);
    }
}

void USART1_IRQHandler(void)
{
    if (USART_GetITStatus(USART1, USART_IT_TXE) != RESET) {
        if (g_tx_tail != g_tx_head) {
            USART_SendData(USART1, g_tx_buffer[g_tx_tail]);
            g_tx_tail = (uint16_t)((g_tx_tail + 1U) % UART_TX_BUFFER_SIZE);
        } else {
            USART_ITConfig(USART1, USART_IT_TXE, DISABLE);
        }
    }
}
