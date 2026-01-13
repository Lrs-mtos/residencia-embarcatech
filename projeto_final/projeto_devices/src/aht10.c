#include "pico/stdlib.h"
#include "FreeRTOS.h"
#include "aht10.h"

void aht_init() {
    // Comando de calibração padrão se necessário
    uint8_t cmd[] = {0xE1, 0x08, 0x00};
    i2c_write_blocking(AHT_I2C, AHT_ADDR, cmd, 3, false);
}

void aht_read_data(float *temp, float *hum) {
    uint8_t trigger[] = {0xAC, 0x33, 0x00};
    uint8_t data[6];

    i2c_write_blocking(AHT_I2C, AHT_ADDR, trigger, 3, false);
    sleep_ms(80); // Necessário para o AHT10 processar
    i2c_read_blocking(AHT_I2C, AHT_ADDR, data, 6, false);

    uint32_t hum_raw = ((uint32_t)data[1] << 12) | ((uint32_t)data[2] << 4) | (data[3] >> 4);
    *hum = (float)hum_raw * 100.0f / (1 << 20);

    uint32_t temp_raw = (((uint32_t)data[3] & 0x0F) << 16) | ((uint32_t)data[4] << 8) | data[5];
    *temp = (float)temp_raw * 200.0f / (1 << 20) - 50.0f;
}