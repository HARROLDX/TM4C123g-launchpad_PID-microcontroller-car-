/*
 * i2c.c
 *
 *  Created on: 9 May 2024
 *      Author: frank
 */
#include <stdint.h>
#include "tm4c123.h"

void initI2C()
{
    // uses I2C3
    // see page 1015 in reference manual
    SYSCTL_RCGCI2C |= (1 << 3); // enable I2C3
    SYSCTL_RCGCGPIO |= (1 << 3); // I2C3.SCL @ PD0, I2C3.SDA @ PD1 so must enable Port D
    SYSCTL_GPIOHBCTL |= 0x1f; // turn on AHB access for ports
    GPIOD_DEN |= (1 << 0) | (1 << 1);
    GPIOD_AFSEL |= (1 << 0) | (1 << 1); // enable alternative function for PD0 and PD1
    // PMCx Bit Field Encoding = 3 for both pins
    GPIOD_PCTL &= ~(0xff); // clear lower 8 bits first
    GPIOD_PCTL |= (0x33);  // and select mode 3 for PD0 and PD1
    GPIOD_ODR |= (1 << 1);  // enable open drain for PD1 (SDA)
    I2C3_MCR = 0x00000010;  // initialize I2C master
    I2C3_MTPR = (16000000/(2 * (6+4)*100000))-1;
}

int i2c_read_register(uint8_t i2c_address, uint8_t reg, uint8_t *p_rxbyte)
{

    I2C3_MSA=(i2c_address << 1); // set address and r/w' bit = 0 (write mode)
    I2C3_MDR=reg;
    I2C3_MCS = 3; // send start signal and start the interface
    while(I2C3_MCS & 1);  // wait for last byte to go
    I2C3_MSA=(i2c_address << 1) | 1; // set address and r/w' bit = 1 (read mode)
    I2C3_MCS = 7; // send start signal and start the interface, send stop when done

    I2C3_MDR=0xff; // output a dummy byte
    while(I2C3_MCS & 1);  // wait for last byte to go
    *p_rxbyte=I2C3_MDR;
  //  I2C3_MCS=4; // send stop signal
    return 0;
}
int i2c_write_register(uint8_t i2c_address, uint8_t reg, uint8_t txbyte )
{

    I2C3_MSA=(i2c_address << 1); // set address and r/w' bit = 0 (write mode)
    I2C3_MDR=reg;
    I2C3_MCS = 3; // send start signal and start the interface
    while(I2C3_MCS & 1);  // wait for last byte to go
    I2C3_MDR=txbyte; // output the byte
    I2C3_MCS = 1; // send run signal
    while(I2C3_MCS & 1);  // wait for last byte to go
    I2C3_MCS=4; // send stop signal
    return 0;
}

