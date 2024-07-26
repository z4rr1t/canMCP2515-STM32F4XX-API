#include "MCP2515.h"

#define CS0_PIN GPIO_PIN_0
#define CS1_PIN GPIO_PIN_1

MCP2515 mcp1;
MCP2515 mcp2;

void setup()
{
    SPI_HandleTypeDef hspi;
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
    HAL_SPI_Init(&hspi);

    mcp1 = MCP2515();  // Adjust GPIO pin and port as needed
    mcp2 = MCP2515();  // Adjust GPIO pin and port as needed

    mcp1.init(&hspi, CS0_PIN, GPIOA);
    mcp2.init(&hspi, CS1_PIN, GPIOA);

    mcp1.initCAN(CAN_BAUD_100K);
    mcp2.initCAN(CAN_BAUD_100K);


};