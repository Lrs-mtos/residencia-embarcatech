#ifndef MPU6050_H
#define MPU6050_H

#include "hardware/i2c.h"

#define MPU_ADDR 0x68
#define MPU6050_I2C_PORT i2c1
#define MPU6050_SDA_PIN 2
#define MPU6050_SCL_PIN 3

void mpu_init();
void mpu_read_data(float *roll, bool *drawer_open);

#endif