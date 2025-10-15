#include <stdio.h>
#include <string.h> // Necessário para memset
#include "pico/stdlib.h"
#include "hardware/i2c.h"

// Includes for the OLED driver from the 'inc' folder
#include "ssd1306.h"
#include "ssd1306_i2c.h"

// --- OLED Display Configuration ---
#define OLED_I2C_PORT i2c1
#define OLED_SDA_PIN 14
#define OLED_SCL_PIN 15

// --- AHT10 Sensor Configuration ---
#define AHT10_I2C_PORT i2c0
#define AHT10_SDA_PIN 0
#define AHT10_SCL_PIN 1

const int AHT10_ADDR = 0x38;
const uint8_t AHT10_CMD_TRIGGER[] = {0xAC, 0x33, 0x00};

// --- Global variables for the OLED buffer and render area ---
static uint8_t ssd_buffer[ssd1306_buffer_length];
static struct render_area frame_area;


// Function to read and process data from the AHT10
void read_aht10(float* temp, float* humidity) {
    uint8_t data[6];
    i2c_write_blocking(AHT10_I2C_PORT, AHT10_ADDR, AHT10_CMD_TRIGGER, sizeof(AHT10_CMD_TRIGGER), false);
    sleep_ms(80);
    i2c_read_blocking(AHT10_I2C_PORT, AHT10_ADDR, data, 6, false);

    uint32_t raw_humidity = ((uint32_t)data[1] << 12) | ((uint32_t)data[2] << 4) | (data[3] >> 4);
    *humidity = (float)raw_humidity * 100.0f / (1 << 20);

    uint32_t raw_temp = (((uint32_t)data[3] & 0x0F) << 16) | ((uint32_t)data[4] << 8) | data[5];
    *temp = (float)raw_temp * 200.0f / (1 << 20) - 50.0f;
}

// Function to initialize all hardware
void init_all_hardware() {
    // AHT10 Sensor I2C
    i2c_init(AHT10_I2C_PORT, 100 * 1000);
    gpio_set_function(AHT10_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(AHT10_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(AHT10_SDA_PIN);
    gpio_pull_up(AHT10_SCL_PIN);

    // OLED Display I2C
    i2c_init(OLED_I2C_PORT, 400 * 1000);
    gpio_set_function(OLED_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(OLED_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(OLED_SDA_PIN);
    gpio_pull_up(OLED_SCL_PIN);

    // Initialize the OLED display controller
    ssd1306_init();

    // Setup the render area to cover the whole screen
    frame_area.start_column = 0;
    frame_area.end_column = ssd1306_width - 1;
    frame_area.start_page = 0;
    frame_area.end_page = ssd1306_n_pages - 1;
    calculate_render_area_buffer_length(&frame_area);
}

int main() {
    stdio_init_all();
    sleep_ms(2000); // Wait for serial connection

    init_all_hardware();

    float temperature, humidity;
    char temp_str[20];
    char hum_str[20];

    printf("Iniciando monitoramento...\n\n");

    while (true) {
        read_aht10(&temperature, &humidity);

        // **NOVO: Imprime os valores no terminal para depuração**
        printf("Leitura do Sensor:\n");
        printf("  - Temperatura: %.2f C\n", temperature);
        printf("  - Umidade:     %.2f %%\n", humidity);
        printf("------------------------\n");

        // Limpa o buffer do display antes de desenhar novos dados
        memset(ssd_buffer, 0, ssd1306_buffer_length);

        // Prepara as strings para serem exibidas no OLED
        snprintf(temp_str, sizeof(temp_str), "Temp: %.1f C", temperature);
        snprintf(hum_str, sizeof(hum_str), "Umid: %.1f %%", humidity);

        // Desenha as strings no buffer do display
        ssd1306_draw_string(ssd_buffer, 5, 5, temp_str);
        ssd1306_draw_string(ssd_buffer, 5, 20, hum_str);

        // Lógica de Alerta
        if (humidity > 70.0f || temperature < 20.0f) {
            ssd1306_draw_line(ssd_buffer, 0, 38, 127, 38, true);
            ssd1306_draw_string(ssd_buffer, 5, 45, "ALERTA!");
        }

        // Envia o buffer atualizado para a tela
        render_on_display(ssd_buffer, &frame_area);

        sleep_ms(2000);
    }
    return 0;
}