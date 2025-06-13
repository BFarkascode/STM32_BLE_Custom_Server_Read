
/*
 *  Created on: June 13, 2025
 *  Author: BalazsFarkas
 *  Project: STM32_BLE_Custom_Server_Read
 *  Processor: STM32WB55MM
 *  Header version: 1.0
 *  File: I2CDriver_STM32WB55MM.c
 *  Change history:
 */

#ifndef INC_I2CDRIVER_STM32L4X2_H_
#define INC_I2CDRIVER_STM32L4X2_H_

#include "stdint.h"
#include "stm32wbxx.h";

//LOCAL CONSTANT
//bus latency
static const uint8_t I2C_bus_latency_us = 0x5A;			//it takes 90 um (0x5A) for a byte and an ACK to cross the bus at 100 kHz
														//it takes 25 um (0x19) for a byte and an ACK to cross the bus at 400 kHz

//FUNCTION PROTOTYPES
void I2CConfig(uint8_t dev_own_addr);
int I2CSCANNER (uint8_t slave_addr);
int I2CTX (uint8_t slave_addr, uint8_t number_of_bytes, uint8_t *bytes_to_send);
int I2CRX (uint8_t slave_addr, uint8_t number_of_bytes, uint8_t* bytes_received);
void I2CReadout(uint8_t slave_addr, uint8_t number_of_readouts, uint8_t* data_buffer);


#endif /* INC_I2CDRIVER_STM32L4X2_H_ */
