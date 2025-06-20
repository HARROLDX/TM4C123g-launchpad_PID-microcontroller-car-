/*
 * accel_6050.c
 *
 *  Created on: 27 May 2024
 *      Author: frank
 */

#include <accel_6050.h>
#include <stdint.h>
#include "i2c.h"
int init_accel_6050(void)
{
    uint8_t who_am_i;
    initI2C();
    // attempt to read the who_am_i byte to be sure the device is there
    i2c_read_register(0x68,117,&who_am_i);
    if (who_am_i == 0x68)
    {
        i2c_write_register(0x68,107,0); // wake mpu6050
        i2c_write_register(0x68,35,8); // enable FIFO
        i2c_write_register(0x68,29,6); // enable low pass filter

        return 0; // no error
    }
    else
        return -1; // device missing
}
int16_t read_x_accel(void)
{
    uint8_t accel_high,accel_low;
    int16_t accel;
    i2c_read_register(0x68,59,&accel_high);
    i2c_read_register(0x68,60,&accel_low);
    accel = accel_high;
    accel = accel << 8;
    accel = accel + accel_low;
    return accel;
}
int16_t read_y_accel(void)
{
    uint8_t accel_high,accel_low;
    int16_t accel;
    i2c_read_register(0x68,61,&accel_high);
    i2c_read_register(0x68,62,&accel_low);
    accel = accel_high;
    accel = accel << 8;
    accel = accel + accel_low;
    return accel;
}
int16_t read_z_accel(void)
{
    uint8_t accel_high,accel_low;
    int16_t accel;
    i2c_read_register(0x68,63,&accel_high);
    i2c_read_register(0x68,64,&accel_low);
    accel = accel_high;
    accel = accel << 8;
    accel = accel + accel_low;
    return accel;
}
int16_t read_temperature()
{
    uint8_t accel_high,accel_low;
    int16_t accel;
    i2c_read_register(0x68,63,&accel_high);
    i2c_read_register(0x68,64,&accel_low);
    accel = accel_high;
    accel = accel << 8;
    accel = accel + accel_low;
    accel = (accel/340)+37;
    return accel;
}
////////////////////
int16_t read_x_gyro(void)
{
    uint8_t accel_high,accel_low;//8bit high value and low value
        int16_t accel;//16bit (highlow)
        i2c_read_register(0x68,67,&accel_high);//X_OUT_LOW is located at 67
        i2c_read_register(0x68,68,&accel_low);//X_OUT_HIGH is located at 68
        accel = accel_high;//8bit High
        accel = accel << 8;//left shift(HIGH0000000)
        accel = accel + accel_low;//(PUT LOW IN)

        return accel;


}
int16_t read_y_gyro(void)
{
    uint8_t accel_high,accel_low;//8bit high value and low value
        int16_t accel;//16bit (highlow)
        i2c_read_register(0x68,69,&accel_high);//Y_OUT_LOW is located at 69
        i2c_read_register(0x68,70,&accel_low);//Y_OUT_HIGH is located at 70
        accel = accel_high;//8bit High
        accel = accel << 8;//left shift(HIGH0000000)
        accel = accel + accel_low;//(PUT LOW IN)

        return accel;


}
int16_t read_z_gyro(void)
{
    uint8_t accel_high,accel_low;//8bit high value and low value
        int16_t accel;//16bit (highlow)
        i2c_read_register(0x68,71,&accel_high);//Z_OUT_LOW is located at 71
        i2c_read_register(0x68,72,&accel_low);//Z_OUT_HIGH is located at 72
        accel = accel_high;//8bit High
        accel = accel << 8;//left shift(HIGH0000000)
        accel = accel + accel_low;//(PUT LOW IN)

        return accel;


}
