#ifndef MCP2515_H
#define MCP2515_H

#include "stm32f4xx_hal.h"  // Include your STM32 HAL header

// MCP2515 Register Addresses
#define MCP2515_RXF0SIDH 0x00
#define MCP2515_RXF0SIDL 0x01
#define CAN_BAUD_10K  1
#define CAN_BAUD_50K  2
#define CAN_BAUD_100K 3
#define CAN_BAUD_125K 4
#define CAN_BAUD_250K 5
#define CAN_BAUD_500K 6
// Define other register addresses as needed...

// CAN Message Structure
typedef struct {
    uint32_t adrsValue;
    bool isExtendedAdrs;
    bool rtr;
    uint8_t dataLength;
    uint8_t data[8];
} CANMSG;

class MCP2515 {
public:
    MCP2515(SPI_HandleTypeDef* hspi1, GPIO_TypeDef* csPort, uint16_t csPin);
    void MCP2515::setHspi(SPI_HandleTypeDef& hspi);
    bool initCAN(uint32_t baudrate);
    bool setCANBaud(uint32_t baudrate);
    bool setCANNormalMode(bool singleShot);
    bool setCANReceiveonlyMode();
    bool receiveCANMessage(CANMSG* msg, uint32_t timeout);
    bool transmitCANMessage(CANMSG msg, uint32_t timeout);
    uint8_t getCANTxErrCnt();
    uint8_t getCANRxErrCnt();

private:
    SPI_HandleTypeDef* spi;
    GPIO_TypeDef* csPort;
    uint16_t csPin;

    void writeReg(uint8_t regno, uint8_t val);
    uint8_t readReg(uint8_t regno);
    void writeRegBit(uint8_t regno, uint8_t bitno, uint8_t val);
};

#endif /* MCP2515_H */
