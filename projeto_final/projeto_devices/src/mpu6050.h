#ifndef MPU6050_H
#define MPU6050_H

#include "hardware/i2c.h"

#define MPU_ADDR 0x68
#define MPU_I2C i2c0

void mpu_init();
void mpu_read_data(float *roll, bool *drawer_open);

#endif