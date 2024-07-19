#include <stm32f4xx_hal_spi.h>
#include <Arduino.h>
#include "MCP2515.h"


SPI_HandleTypeDef hspi;

#define CS_PIN GPIO_PIN_11
#define CS_PORT GPIOA

// MCP2515 Registers
#define RXF0SIDH 0x00
#define RXF0SIDL 0x01
#define RXF0EID8 0x02
#define RXF0EID0 0x03
#define RXF1SIDH 0x04
#define RXF1SIDL 0x05
#define RXF1EID8 0x06
#define RXF1EID0 0x07
#define RXF2SIDH 0x08
#define RXF2SIDL 0x09
#define RXF2EID8 0x0A
#define RXF2EID0 0x0B
#define BFPCTRL 0x0C
#define TXRTSCTRL 0x0D
#define CANSTAT 0x0E
#define CANCTRL 0x0F
#define RXF3SIDH 0x10
#define RXF3SIDL 0x11
#define RXF3EID8 0x12
#define RXF3EID0 0x13
#define RXF4SIDH 0x14
#define RXF4SIDL 0x15
#define RXF4EID8 0x16
#define RXF4EID0 0x17
#define RXF5SIDH 0x18
#define RXF5SIDL 0x19
#define RXF5EID8 0x1A
#define RXF5EID0 0x1B
#define TEC 0x1C
#define REC 0x1D
#define RXM0SIDH 0x20
#define RXM0SIDL 0x21
#define RXM0EID8 0x22
#define RXM0EID0 0x23
#define RXM1SIDH 0x24
#define RXM1SIDL 0x25
#define RXM1EID8 0x26
#define RXM1EID0 0x27
#define CNF3 0x28
#define CNF2 0x29
#define CNF1 0x2A
#define CANINTE 0x2B
#define MERRE 7
#define WAKIE 6
#define ERRIE 5
#define TX2IE 4
#define TX1IE 3
#define TX0IE 2
#define RX1IE 1
#define RX0IE 0
#define CANINTF 0x2C
#define MERRF 7
#define WAKIF 6
#define ERRIF 5
#define TX2IF 4
#define TX1IF 3
#define TX0IF 2
#define RX1IF 1
#define RX0IF 0
#define EFLG 0x2D
#define TXB0CTRL 0x30
#define TXREQ 3
#define TXB0SIDH 0x31
#define TXB0SIDL 0x32
#define EXIDE 3
#define TXB0EID8 0x33
#define TXB0EID0 0x34
#define TXB0DLC 0x35
#define TXRTR 6
#define TXB0D0 0x36

#define RXB0CTRL 0x60
#define RXM1 6
#define RXM0 5
#define RXRTR 3
// Bits 2:0 FILHIT2:0
#define RXB0SIDH 0x61
#define RXB0SIDL 0x62
#define RXB0EID8 0x63
#define RXB0EID0 0x64
#define RXB0DLC 0x65
#define RXB0D0 0x66

// MCP2515 Command Bytes
#define RESET 0xC0
#define READ 0x03
#define READ_RX_BUFFER 0x90
#define WRITE 0x02
#define LOAD_TX_BUFFER 0x40
#define RTS 0x80
#define READ_STATUS 0xA0
#define RX_STATUS 0xB0
#define BIT_MODIFY 0x05

MCP2515::MCP2515(SPI_HandleTypeDef *hspi1, GPIO_TypeDef *csPort, uint16_t csPin)
{
    
    SPI_HandleTypeDef *spi = hspi1;
    this->csPin = csPin;
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin = csPin;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(csPort, &gpio);
    HAL_GPIO_WritePin(csPort, csPin, GPIO_PIN_SET);
    }

boolean MCP2515::initCAN(uint32_t baudConst)
{
    uint8_t mode;
    HAL_SPI_Init(spi);
    HAL_GPIO_WritePin(csPort, csPin, GPIO_PIN_SET);
    uint8_t resetCmd = RESET;
    HAL_SPI_Transmit(spi, &resetCmd, 1, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(csPort, csPin, GPIO_PIN_RESET);
    HAL_Delay(100);

    mode = readReg(CANSTAT) >> 5;
    if (mode != 0b100)
    {
        return false;
    }
    return true;
}

boolean MCP2515::setCANBaud(uint32_t baudConst)
{
    uint8_t brp;

    // BRP<5:0> = 00h, so divisor (0+1)*2 for 125ns per quantum at 16MHz for 500K
    // SJW<1:0> = 00h, Sync jump width = 1
    switch (baudConst)
    {
    case CAN_BAUD_500K:
        brp = 0;
        break;
    case CAN_BAUD_250K:
        brp = 1;
        break;
    case CAN_BAUD_125K:
        brp = 3;
        break;
    case CAN_BAUD_100K:
        brp = 4;
        break;
    default:
        return false;
    }
    HAL_GPIO_WritePin(csPort, csPin, GPIO_PIN_RESET);
    uint8_t writeCmd = WRITE;
    HAL_SPI_Transmit(spi, &writeCmd, 1, HAL_MAX_DELAY);
    uint8_t cnf1Cmd = CNF1;
    HAL_SPI_Transmit(spi, &cnf1Cmd, 1, HAL_MAX_DELAY);
    uint8_t cnf = brp & 0b00111111;
    HAL_SPI_Transmit(spi, &cnf, 1, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(csPort, csPin, GPIO_PIN_SET);

    // PRSEG<2:0> = 0x01, 2 time quantum for prop
    // PHSEG<2:0> = 0x06, 7 time constants to PS1 sample
    // SAM = 0, just 1 sampling
    // BTLMODE = 1, PS2 determined by CNF3
    HAL_GPIO_WritePin(csPort, csPin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(spi, &writeCmd, 1, HAL_MAX_DELAY);
    uint8_t cnf2Cmd = CNF2;
    HAL_SPI_Transmit(spi, &cnf2Cmd, 1, HAL_MAX_DELAY);
    uint8_t phseg = 5;
    HAL_SPI_Transmit(spi, &phseg, 1, HAL_MAX_DELAY);

    // SyncSeg + PropSeg + PS1 + PS2 = 1 + 2 + 7 + 6 = 16
    return true;
}
boolean MCP2515::setCANNormalMode(bool singleShot)
{
    // REQOP2<2:0> = 000 for normal mode
    // ABAT = 0, do not abort pending transmission
    // OSM = 0, not one shot
    // CLKEN = 1, disable output clock
    // CLKPRE = 0b11, clk/8

    uint8_t settings;
    uint8_t mode;

    settings = 0b00000111 | (singleShot << 3);

    writeReg(CANCTRL, settings);
    // Read mode and make sure it is normal
    mode = readReg(CANSTAT) >> 5;
    if (mode != 0)
        return false;
    return true;
}

boolean MCP2515::setCANReceiveonlyMode()
{
    // REQOP2<2:0> = 011 for receive-only mode
    // ABAT = 0, do not abort pending transmission
    // OSM = 0, not one shot
    // CLKEN = 1, disable output clock
    // CLKPRE = 0b11, clk/8

    uint8_t mode;
    writeReg(CANCTRL, 0b01100111);
    // Read mode and make sure it is receive-only

    mode = readReg(CANSTAT) >> 5;
    if (mode != 3)
        return false;

    return true;
}
boolean MCP2515::receiveCANMessage(CANMSG *msg, unsigned long timeout)
{
    unsigned long startTime, endTime;
    unsigned short standardID = 0;
    bool gotMessage;
    uint8_t val;
    int i;

    startTime = millis();
    endTime = startTime + timeout;
    gotMessage = false;
    while (millis() < endTime)
    {
        val = readReg(CANINTF);
        // If we have a message available, read it
        if (bitRead(val, RX0IF) == 1)
        {
            gotMessage = true;
            break;
        }
    }

    if (gotMessage)
    {
        val = readReg(RXB0CTRL);
        msg->rtr = ((bitRead(val, 3) == 1) ? true : false);
        // Address received from
        val = readReg(RXB0SIDH);
        standardID |= (val << 3);
        val = readReg(RXB0SIDL);
        standardID |= (val >> 5);
        msg->adrsValue = long(standardID);
        msg->isExtendedAdrs = ((bitRead(val, EXIDE) == 1) ? true : false);
        if (msg->isExtendedAdrs)
        {
            msg->adrsValue = ((msg->adrsValue << 2) | (val & 0b11));
            val = readReg(RXB0EID8);
            msg->adrsValue = (msg->adrsValue << 8) | val;
            val = readReg(RXB0EID0);
            msg->adrsValue = (msg->adrsValue << 8) | val;
        }
        msg->adrsValue = 0b11111111111111111111111111111 & msg->adrsValue; // mask out extra bits
        // Read data bytes
        val = readReg(RXB0DLC);
        msg->dataLength = (val & 0xf);
        HAL_GPIO_WritePin(csPort, csPin, GPIO_PIN_RESET);
        uint8_t readCmd = READ;
        HAL_SPI_Transmit(spi, &readCmd, 1, HAL_MAX_DELAY);
        uint8_t readFrom = RXB0D0;
        HAL_SPI_Transmit(spi, &readFrom, 1, HAL_MAX_DELAY);
        for (i = 0; i < msg->dataLength; i++)
        {
            uint8_t data = 0;
            msg->data[i] = HAL_SPI_Transmit(spi, &data, 1, HAL_MAX_DELAY);
        }
        HAL_GPIO_WritePin(csPort, csPin, GPIO_PIN_SET);
        // clear read interrupt
        writeRegBit(CANINTF, RX0IF, 0);
    }
    return gotMessage;
}
boolean MCP2515::transmitCANMessage(CANMSG msg, unsigned long timeout)
{
    unsigned long startTime, endTime;
    boolean sentMessage;
    unsigned short val;
    int i;
    unsigned short standardID = 0;

    standardID = short(msg.adrsValue);
    startTime = millis();
    endTime = startTime + timeout;
    sentMessage = false;
    if (!msg.isExtendedAdrs)
    {
        // Write standard ID registers
        val = standardID >> 3;
        writeReg(TXB0SIDH, val);
        val = standardID << 5;
        writeReg(TXB0SIDL, val);
    }
    else
    {
        // Write extended ID registers, which use the standard ID registers
        val = msg.adrsValue >> 21;
        writeReg(TXB0SIDH, val);
        val = msg.adrsValue >> 16;
        val = val & 0b00000011;
        val = val | (msg.adrsValue >> 13 & 0b11100000);
        val |= 1 << EXIDE;
        writeReg(TXB0SIDL, val);
        val = msg.adrsValue >> 8;
        writeReg(TXB0EID8, val);
        val = msg.adrsValue;
        writeReg(TXB0EID0, val);
    }

    val = msg.dataLength & 0x0f;
    if (msg.rtr)
        bitWrite(val, TXRTR, 1);
    writeReg(TXB0DLC, val);

    // Message bytes
    HAL_GPIO_WritePin(csPort, csPin, GPIO_PIN_RESET);
    uint8_t writeCmd = WRITE;
    HAL_SPI_Transmit(spi, &writeCmd, 1, HAL_MAX_DELAY);
    uint8_t writeTo = TXB0D0;
    HAL_SPI_Transmit(spi, &writeTo, 1, HAL_MAX_DELAY);
    for (i = 0; i < msg.dataLength; i++)
    {
        HAL_SPI_Transmit(spi, &msg.data[i], 1, HAL_MAX_DELAY);
    }
    // Transmit the message

    writeRegBit(TXB0CTRL, TXREQ, 1);
    sentMessage = false;
    while (millis() < endTime)
    {
        val = readReg(CANINTF);
        if (bitRead(val, TX0IF) == 1)
        {
            sentMessage = true;
            break;
        }
    }

    // Abort the send if failed
    writeRegBit(TXB0CTRL, TXREQ, 0);

    // And clear write interrupt
    writeRegBit(CANINTF, TX0IF, 0);

    return sentMessage;
}
byte MCP2515::getCANRxErrCnt()
{
    return (readReg(REC));
}
byte MCP2515::getCANTxErrCnt()
{
    return (readReg(TEC));
}

void MCP2515::writeReg(byte regNo, byte val)
{
    byte writeCmd = WRITE;
    HAL_GPIO_WritePin(csPort, csPin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(spi, &writeCmd, 1, HAL_MAX_DELAY);
    HAL_SPI_Transmit(spi, &regNo, 1, HAL_MAX_DELAY);
    HAL_SPI_Transmit(spi, &val, 1, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(csPort, csPin, GPIO_PIN_SET);
}

void MCP2515::writeRegBit(byte regNo, byte bitNo, byte val)
{

    byte writeCmd = BIT_MODIFY;
    HAL_GPIO_WritePin(csPort, csPin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(spi, &writeCmd, 1, HAL_MAX_DELAY);
    HAL_SPI_Transmit(spi, &regNo, 1, HAL_MAX_DELAY);
    byte mask = 1 << bitNo;
    HAL_SPI_Transmit(spi, &mask, 1, HAL_MAX_DELAY);
    if (val != 0)
    {
        byte cont = 0xff;
        HAL_SPI_Transmit(spi, &cont, 1, HAL_MAX_DELAY);
    }
    else
    {
        byte br = 0x00;
        HAL_SPI_Transmit(spi, &br, 1, HAL_MAX_DELAY);
    }
}
byte MCP2515::readReg(byte regNo)
{
    byte val;

    HAL_GPIO_WritePin(csPort, csPin, GPIO_PIN_RESET);
    byte readCmd = READ;
    HAL_SPI_Transmit(spi, &readCmd, 1, HAL_MAX_DELAY);
    HAL_SPI_Transmit(spi, &regNo, 1, HAL_MAX_DELAY);
    byte valInd = 0;
    val = HAL_SPI_Transmit(spi, &valInd, 1, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(csPort, csPin, GPIO_PIN_SET);
    
    return val;
}