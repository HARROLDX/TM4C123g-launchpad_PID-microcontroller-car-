/*
 * accel_9250.h
 *
 *  Created on: 27 May 2024
 *      Author: frank
 */

#ifndef ACCEL_6050_H_
#define ACCEL_6050_H_
#include <stdint.h>
int init_accel_6050(void);
int16_t read_x_accel(void);
int16_t read_y_accel(void);
int16_t read_z_accel(void);
int16_t read_temperature(void);
int16_t read_x_gyro(void);
int16_t read_y_gyro(void);
int16_t read_z_gyro(void);

#endif /* ACCEL_6050_H_ */
