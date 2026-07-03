#include "bluetooth.h"

#define BLUETOOTH_TX_BUFFER_SIZE 256
#define BLUETOOTH_RX_BUFFER_SIZE 64

static volatile uint16_t g_bt_tx_head = 0;
static volatile uint16_t g_bt_tx_tail = 0;
static uint8_t g_bt_tx_buffer[BLUETOOTH_TX_BUFFER_SIZE];

static volatile uint16_t g_bt_rx_head = 0;
static volatile uint16_t g_bt_rx_tail = 0;
static uint8_t g_bt_rx_buffer[BLUETOOTH_RX_BUFFER_SIZE];

void BLUETOOTH_Init(void)
{
    GPIO_InitTypeDef gpio;
    USART_InitTypeDef usart;
    NVIC_InitTypeDef nvic;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

    /* PB10 = USART3_TX */
    GPIO_StructInit(&gpio);
    gpio.GPIO_Pin = GPIO_Pin_10;
    gpio.GPIO_Mode = GPIO_Mode_AF_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &gpio);

    /* PB11 = USART3_RX */
    gpio.GPIO_Pin = GPIO_Pin_11;
    gpio.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOB, &gpio);

    USART_StructInit(&usart);
    usart.USART_BaudRate = BLUETOOTH_BAUDRATE;
    usart.USART_WordLength = USART_WordLength_8b;
    usart.USART_StopBits = USART_StopBits_1;
    usart.USART_Parity = USART_Parity_No;
    usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    usart.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART3, &usart);

    nvic.NVIC_IRQChannel = USART3_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 2;
    nvic.NVIC_IRQChannelSubPriority = 1;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);

    USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
    USART_Cmd(USART3, ENABLE);
}

void BLUETOOTH_SendChar(char ch)
{
    uint16_t next =
        (uint16_t)((g_bt_tx_head + 1U) % BLUETOOTH_TX_BUFFER_SIZE);

    while (next == g_bt_tx_tail);

    g_bt_tx_buffer[g_bt_tx_head] = (uint8_t)ch;
    g_bt_tx_head = next;
    USART_ITConfig(USART3, USART_IT_TXE, ENABLE);
}

void BLUETOOTH_SendString(const char *text)
{
    while (*text != '\0') {
        BLUETOOTH_SendChar(*text++);
    }
}

void BLUETOOTH_SendInt32(int32_t value)
{
    char digits[11];
    uint8_t length = 0;
    uint32_t magnitude;

    if (value < 0) {
        BLUETOOTH_SendChar('-');
        magnitude = (uint32_t)(-(value + 1)) + 1U;
    } else {
        magnitude = (uint32_t)value;
    }

    do {
        digits[length++] = (char)('0' + magnitude % 10U);
        magnitude /= 10U;
    } while (magnitude != 0U);

    while (length > 0U) {
        BLUETOOTH_SendChar(digits[--length]);
    }
}

uint8_t BLUETOOTH_Available(void)
{
    return (g_bt_rx_head != g_bt_rx_tail) ? 1U : 0U;
}

uint8_t BLUETOOTH_ReadChar(char *ch)
{
    if (ch == 0 || g_bt_rx_head == g_bt_rx_tail) {
        return 0;
    }

    *ch = (char)g_bt_rx_buffer[g_bt_rx_tail];
    g_bt_rx_tail =
        (uint16_t)((g_bt_rx_tail + 1U) % BLUETOOTH_RX_BUFFER_SIZE);
    return 1;
}

void USART3_IRQHandler(void)
{
    if (USART_GetITStatus(USART3, USART_IT_RXNE) != RESET) {
        uint8_t data = (uint8_t)USART_ReceiveData(USART3);
        uint16_t next =
            (uint16_t)((g_bt_rx_head + 1U) % BLUETOOTH_RX_BUFFER_SIZE);

        if (next != g_bt_rx_tail) {
            g_bt_rx_buffer[g_bt_rx_head] = data;
            g_bt_rx_head = next;
        }
    }

    if (USART_GetITStatus(USART3, USART_IT_TXE) != RESET) {
        if (g_bt_tx_tail != g_bt_tx_head) {
            USART_SendData(USART3, g_bt_tx_buffer[g_bt_tx_tail]);
            g_bt_tx_tail =
                (uint16_t)((g_bt_tx_tail + 1U) %
                           BLUETOOTH_TX_BUFFER_SIZE);
        } else {
            USART_ITConfig(USART3, USART_IT_TXE, DISABLE);
        }
    }
}
