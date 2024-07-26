#include "MCP2515.h"
#include <Arduino.h>
#include <string.h>

#define CS0_PIN GPIO_PIN_0
#define CS1_PIN GPIO_PIN_1
#define INT0_PIN GPIO_PIN_10
#define INT1_PIN GPIO_PIN_11


MCP2515 can1;
MCP2515 can2;
CANMSG msg;
SPI_HandleTypeDef hspi;
UART_HandleTypeDef huart;

#define SPI_CAN &hspi
#define MESSAGE msg

volatile bool messageReceived = false;

void SPI_Init() {

    __HAL_RCC_SPI1_CLK_ENABLE();

    hspi.Instance = SPI1;
    hspi.Init.Mode = SPI_MODE_MASTER;
    hspi.Init.Direction = SPI_DIRECTION_2LINES;
    hspi.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi.Init.CLKPhase = SPI_PHASE_1EDGE;
    hspi.Init.NSS = SPI_NSS_SOFT;
    hspi.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
    hspi.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi.Init.CRCPolynomial = 10;
    HAL_SPI_Init(SPI_CAN);
}
// Initialize GPIO pins
void GPIO_Init() {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    __HAL_RCC_GPIOA_CLK_ENABLE(); // Enable clock for GPIOA

    // Configure CS0_PIN
    GPIO_InitStruct.Pin = CS0_PIN; // Chip select for MCP2515 #1
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP; // Push-pull output
    GPIO_InitStruct.Pull = GPIO_NOPULL; // No pull-up or pull-down
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW; // Low speed
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct); // Initialize GPIO for CS0_PIN

    // Configure CS1_PIN
    GPIO_InitStruct.Pin = CS1_PIN; // Chip select for MCP2515 #2
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct); // Initialize GPIO for CS1_PIN

    // Configure INT0_PIN
    GPIO_InitStruct.Pin = INT0_PIN; // Interrupt pin for MCP2515 #1
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING; // Falling edge trigger
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct); // Initialize GPIO for INT0_PIN

    // Configure INT1_PIN
    GPIO_InitStruct.Pin = INT1_PIN; // Interrupt pin for MCP2515 #2
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct); // Initialize GPIO for INT1_PIN
}

void UART_Init()
{
    __HAL_RCC_USART1_CLK_ENABLE(); // Enable clock for USART 2

    huart.Instance = USART2;                        // Use USART2
    huart.Init.BaudRate = 115200;                   // Set baud rate
    huart.Init.WordLength = UART_WORDLENGTH_8B;     // 8 data bits
    huart.Init.StopBits = UART_STOPBITS_1;          // 1 stop bit
    huart.Init.Parity = UART_PARITY_NONE;           // No parity
    huart.Init.Mode = UART_MODE_TX_RX;              // Enable TX and RX
    huart.Init.HwFlowCtl = UART_HWCONTROL_NONE;     // No hardware flow control
    huart.Init.OverSampling = UART_OVERSAMPLING_16; // Oversampling by 16
    HAL_UART_Init(&huart);                          // Initialize UART
}
void UART_SendString(const char *str)
{
    HAL_UART_Transmit(&huart, (uint8_t *)str, strlen(str), HAL_MAX_DELAY);
}

void EXTI0_IRQHandler(void)
{
    HAL_GPIO_EXTI_IRQHandler(INT0_PIN); // Handle The Interrupt
}

void EXTI1_IRQHandler(void)
{
    HAL_GPIO_EXTI_IRQHandler(INT1_PIN);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == INT0_PIN)
    {
        onMessageReceivedCAN0(); // Call the message received function
    }
    else if (GPIO_Pin == INT1_PIN)
    {
        onMessageReceivedCAN1();
    }
}



void setup()
{
    GPIO_Init();
    SPI_Init();
    UART_Init();


    can1.init(SPI_CAN, CS0_PIN, GPIOA);
    can2.init(SPI_CAN, CS1_PIN, GPIOA);

    can1.initCAN();
    can2.initCAN();
    can1.setCANBaud(CAN_BAUD_100K);
    can2.setCANBaud(CAN_BAUD_100K);
    can1.setCANNormalMode(true);
    can2.setCANNormalMode(true);
    // Enable and set EXTI line interrupt
    HAL_NVIC_SetPriority(EXTI0_IRQn, 2, 0); // Set priority
    HAL_NVIC_EnableIRQ(EXTI0_IRQn);         // Enable interrupt
}

void loop()
{
    msg.id = 0x01;
    msg.length = 1;
    msg.data[0] = 42;

    if (can1.transmitCANMessage(msg, 1000))
    {
        UART_SendString("Message sent!\n");
    }
    else
    {
        UART_SendString("Failde to send message.\n");
    }

    delay(1000);
}
void onMessageReceivedCAN0()
{
    if (can1.receiveCANMessage(&msg, 1000))
    {
        messageReceived = true;
        char buffer[50];
        snprintf(buffer, sizeof(buffer), "CAN0 Received a message with ID: %lu\n", msg.id);
        UART_SendString(buffer);
        for (uint8_t i = 0; i < msg.length; i++)
        {
            sniprintf(buffer, sizeof(buffer), "Data: %d\n", msg.data[i]);
            UART_SendString(buffer);
        }
    }
}
void onMessageReceivedCAN1()
{
    if (can2.receiveCANMessage(&msg, 1000))
    {
        messageReceived = true;
        char buffer[50];
        snprintf(buffer, sizeof(buffer), "CAN1 Received a message with ID: %lu\n", msg.id);
        UART_SendString(buffer);
        for (uint8_t i = 0; i < msg.length; i++)
        {
            sniprintf(buffer, sizeof(buffer), "Data: %d\n", msg.data[i]);
            UART_SendString(buffer);
        }
    }
}