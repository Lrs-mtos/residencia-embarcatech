#ifndef AHT10_H
#define AHT10_H

#include "hardware/i2c.h"

#define AHT_ADDR 0x38
#define AHT_I2C i2c0

void aht_init();
void aht_read_data(float *temp, float *hum);

#endif