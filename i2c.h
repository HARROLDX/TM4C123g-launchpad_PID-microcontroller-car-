/*
 * i2c.h
 *
 *  Created on: 9 May 2024
 *      Author: frank
 * This module works with I2C3 on the TivaC launchpad
 */

#ifndef I2C_H_
#define I2C_H_

#include <stdint.h>
void initI2C();
int i2c_read_register(uint8_t i2c_address, uint8_t reg, uint8_t *p_rxbyte);
int i2c_write_register(uint8_t i2c_address, uint8_t reg, uint8_t txbyte);
#endif /* I2C_H_ */
