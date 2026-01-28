#ifndef AHT10_H
#define AHT10_H

#include "hardware/i2c.h"

#define AHT_ADDR 0x38
#define AHT_I2C i2c0
#define AHT10_SDA_PIN 0
#define AHT10_SCL_PIN 1

void aht_init();
void aht_read_data(float *temp, float *hum);

#endif