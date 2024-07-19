# Project Name

## Description
This project implements a CAN bus communication interface using the MCP2515 controller on an STM32 microcontroller.

## Features
- Initialization of CAN controller
- Setting CAN bus baud rate
- Transmitting and receiving CAN messages
- Error handling for CAN bus

## Dependencies
- STM32F4xx HAL library
- Arduino library
- MCP2515 library

## Setup
1. Include the necessary libraries in your project.
2. Connect the MCP2515 controller to the STM32 microcontroller.
3. Initialize the MCP2515 object with the appropriate SPI and GPIO configurations.
4. Use the provided methods to interact with the CAN bus.

## Usage
1. Initialize the CAN controller using `initCAN()` method.
2. Set the CAN bus baud rate using `setCANBaud()` method.
3. Transmit CAN messages with `transmitCANMessage()` method.
4. Receive CAN messages with `receiveCANMessage()` method.
5. Check CAN bus error counts with `getCANRxErrCnt()` and `getCANTxErrCnt()` methods.

## Troubleshooting
- If encountering errors related to member variables, ensure they are correctly declared in the class definition.
- Check the wiring and connections between the STM32 microcontroller and MCP2515 controller.

## Contributors
- [z4rr1t](https://github.com/z4rr1t)

## License
This project is licensed under the MIT License - see the LICENSE.md file for details.
