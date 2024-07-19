#include "MCP2515.h"
#include <stm32f4xx_hal_gpio.h>
#include <stm32f4xx_hal_spi.h>
#include <string>

#define GPIO_PIN_LED1 GPIO_PIN_0
#define GPIO_PIN_LED2 GPIO_PIN_0
#define GPIO_PORT GPIOA
#define CS1_PIN GPIO_PIN_10
#define CS2_PIN GPIO_PIN_11
#define CS1_PORT GPIOA
#define CS2_PORT GPIOA
#define LOW 0x0
#define HIGH 0x1
#define CHANGE 0x2
#define FALLING 0x3
#define RISING 0x4
#define EOM 0xef
#define UART_BAUDRATE 9600
#define MODE 0


MCP2515 can1;
MCP2515 can2;
CANMSG msg;


void setup()
{   
    UART_HandleTypeDef uart = {0};
    uart.Instance = UART4;
    uart.Init.BaudRate = UART_BAUDRATE;
    uart.Init.Mode = UART_MODE_TX_RX;
    uart.Init.WordLength = UART_WORDLENGTH_8B;
    uart.Init.StopBits = UART_STOPBITS_1;
    uart.Init.Parity = UART_PARITY_EVEN;

    HAL_UART_Init(&uart);


    GPIO_InitTypeDef gpio[2] = {0};

    gpio[0].Pin = GPIO_PIN_LED1;
    gpio[0].Mode = GPIO_MODE_OUTPUT_PP;
    gpio[0].Pull = GPIO_NOPULL;
    gpio[0].Speed = GPIO_SPEED_FREQ_LOW;
    // Init
    HAL_GPIO_Init(GPIO_PORT, &gpio[0]);

    // pin
    gpio[0].Pin = GPIO_PIN_LED2;
    gpio[0].Mode = GPIO_MODE_OUTPUT_PP;
    gpio[0].Pull = GPIO_NOPULL;
    gpio[0].Speed = GPIO_SPEED_FREQ_LOW;

    HAL_GPIO_Init(GPIO_PORT, &gpio[0]);

    SPI_HandleTypeDef hspi = {0};

    hspi.Instance = SPI1;
    hspi.Init.Mode = SPI_MODE_MASTER;
    hspi.Init.Direction = SPI_DIRECTION_2LINES;
    hspi.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi.Init.CLKPolarity = SPI_POLARITY_LOW;

    HAL_SPI_Init(&hspi);

    GPIO_TypeDef *cs1Post = GPIOA;
    uint16_t cs1Pin = CS1_PIN;

    
    if (can1.initCAN(CAN_BAUD_100K) == 0)
    {
        // your logic here if can didn't initialize
        while (1)
            ;
    }

    if (can1.setCANNormalMode(LOW) == 0)
    {
        // SHOW MESSAGE IF CAN DID NOT SET TO NORMAL
        while (1)
            ;
    }
}

void loop()
{
#ifdef MODE == 0
    // receiver
    int i = can1.receiveCANMessage(&msg, 1000);
    if (i && (msg.data[i] == EOM))
    {
        // logic to do for the message received

        HAL_GPIO_WritePin(GPIO_PORT, GPIO_PIN_LED1, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIO_PORT, GPIO_PIN_LED2, GPIO_PIN_RESET);
        HAL_Delay(100);
        HAL_GPIO_WritePin(GPIO_PORT, GPIO_PIN_LED1, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIO_PORT, GPIO_PIN_LED2, GPIO_PIN_SET);
        HAL_Delay(100);
    }
    #else // MODE ==1
    //Transmitter
        msg.adrsValue = 0x7df;
        msg.isExtendedAdrs = false;
        msg.rtr = false;
        msg.dataLength = 8;
        msg.data[0] = 0x02;
        msg.data[1] = 0x01;
        msg.data[2] = 123;
        msg.data[3] = 0;
        msg.data[4] = 0;
        msg.data[5] = 0;
        msg.data[6] = 0;
        msg.data[7] = EOM;

        can1.transmitCANMessage(msg, 1000);

        HAL_GPIO_WritePin(GPIO_PORT, GPIO_PIN_LED1, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIO_PORT, GPIO_PIN_LED2, GPIO_PIN_RESET);
        HAL_Delay(100);
        HAL_GPIO_WritePin(GPIO_PORT, GPIO_PIN_LED1, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIO_PORT, GPIO_PIN_LED2, GPIO_PIN_SET);
        HAL_Delay(100);
    #endif


}