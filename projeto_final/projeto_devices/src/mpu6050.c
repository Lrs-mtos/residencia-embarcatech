#include "pico/stdlib.h"
#include "FreeRTOS.h"
#include "mpu6050.h"
#include <math.h>

void mpu_init() {
    uint8_t buf[] = {0x6B, 0x00}; // Wake up
    i2c_write_blocking(MPU_I2C, MPU_ADDR, buf, 2, false);
}

void mpu_read_data(float *roll, bool *drawer_open) {
    uint8_t reg = 0x3B;
    uint8_t data[6];
    i2c_write_blocking(MPU_I2C, MPU_ADDR, &reg, 1, true);
    i2c_read_blocking(MPU_I2C, MPU_ADDR, data, 6, false);

    int16_t acc_x = (data[0] << 8) | data[1];
    int16_t acc_y = (data[2] << 8) | data[3];
    int16_t acc_z = (data[4] << 8) | data[5];

    // Cálculo simplificado de inclinação (Roll)
    *roll = atan2f((float)acc_y, (float)acc_z) * 180.0f / M_PI;

    // Lógica de gaveta: se o eixo X sofrer aceleração brusca ou mudar de posição
    // Ajustar limiar conforme testes físicos
    *drawer_open = (fabs(acc_x) > 5000); 
}