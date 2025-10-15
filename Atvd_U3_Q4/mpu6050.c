#include <stdio.h>
#include <string.h>
#include <math.h> 
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/pwm.h"

// Includes para o driver OLED da pasta 'inc'
#include "ssd1306.h"
#include "ssd1306_i2c.h"

// --- Configuração dos Pinos ---
#define MPU6050_I2C_PORT i2c0
#define MPU6050_SDA_PIN 0
#define MPU6050_SCL_PIN 1

#define OLED_I2C_PORT i2c1
#define OLED_SDA_PIN 14
#define OLED_SCL_PIN 15

#define SERVO_PIN 2

// --- Configuração MPU6050 (Registradores) ---
const int MPU6050_ADDR = 0x68;
const int MPU6050_PWR_MGMT_1 = 0x6B;
const int MPU6050_ACCEL_XOUT_H = 0x3B;
const int MPU6050_GYRO_CONFIG = 0x1B;
const int MPU6050_ACCEL_CONFIG = 0x1C;

// --- Configuração Servo Motor ---
#define SERVO_MIN_US 500
#define SERVO_MAX_US 2500
#define MAX_SERVO_ANGLE 30.0f // NOVO: Limita o movimento do servo para +/- 30 graus

// --- Configuração Display OLED ---
static uint8_t ssd_buffer[ssd1306_buffer_length];
static struct render_area frame_area;

// --- Variáveis Globais de Calibração ---
int16_t accel_cal[3] = {0, 0, 0};
int16_t gyro_cal[3] = {0, 0, 0};

// --- Funções do MPU6050 ---
void mpu6050_read_raw(int16_t accel[3], int16_t gyro[3]) {
    uint8_t buffer[14];
    uint8_t reg = MPU6050_ACCEL_XOUT_H;
    i2c_write_blocking(MPU6050_I2C_PORT, MPU6050_ADDR, &reg, 1, true);
    i2c_read_blocking(MPU6050_I2C_PORT, MPU6050_ADDR, buffer, 14, false);

    accel[0] = (buffer[0] << 8) | buffer[1];
    accel[1] = (buffer[2] << 8) | buffer[3];
    accel[2] = (buffer[4] << 8) | buffer[5];
    gyro[0] = (buffer[8] << 8) | buffer[9];
    gyro[1] = (buffer[10] << 8) | buffer[11];
    gyro[2] = (buffer[12] << 8) | buffer[13];
}

void mpu6050_calibrate() {
    printf("Iniciando calibracao do MPU6050... Mantenha o sensor parado e nivelado.\n");
    const int num_samples = 200;
    int32_t accel_sum[3] = {0, 0, 0};
    int32_t gyro_sum[3] = {0, 0, 0};
    int16_t accel_temp[3], gyro_temp[3];

    for (int i = 0; i < num_samples; ++i) {
        mpu6050_read_raw(accel_temp, gyro_temp);
        accel_sum[0] += accel_temp[0];
        accel_sum[1] += accel_temp[1];
        accel_sum[2] += accel_temp[2];
        gyro_sum[0] += gyro_temp[0];
        gyro_sum[1] += gyro_temp[1];
        gyro_sum[2] += gyro_temp[2];
        sleep_ms(5);
    }

    accel_cal[0] = accel_sum[0] / num_samples;
    accel_cal[1] = accel_sum[1] / num_samples;
    accel_cal[2] = accel_sum[2] / num_samples;
    // O eixo Z do acelerômetro deve sentir a gravidade, então ajustamos o offset para isso
    accel_cal[2] -= 16384; // 1g para a escala padrão de +/- 2g

    gyro_cal[0] = gyro_sum[0] / num_samples;
    gyro_cal[1] = gyro_sum[1] / num_samples;
    gyro_cal[2] = gyro_sum[2] / num_samples;
    
    printf("Calibracao finalizada.\n");
    printf("Offsets Accel (X,Y,Z): %d, %d, %d\n", accel_cal[0], accel_cal[1], accel_cal[2]);
    printf("Offsets Gyro (X,Y,Z): %d, %d, %d\n\n", gyro_cal[0], gyro_cal[1], gyro_cal[2]);
    sleep_ms(1000);
}

void mpu6050_init() {
    uint8_t buf[2];
    // Acorda o MPU6050
    buf[0] = MPU6050_PWR_MGMT_1;
    buf[1] = 0x00;
    i2c_write_blocking(MPU6050_I2C_PORT, MPU6050_ADDR, buf, 2, false);
    
    // Configura giroscópio para +/- 250 dps (LSB/dps = 131.0)
    buf[0] = MPU6050_GYRO_CONFIG;
    buf[1] = 0x00;
    i2c_write_blocking(MPU6050_I2C_PORT, MPU6050_ADDR, buf, 2, false);

    // Configura acelerômetro para +/- 2g (LSB/g = 16384)
    buf[0] = MPU6050_ACCEL_CONFIG;
    buf[1] = 0x00;
    i2c_write_blocking(MPU6050_I2C_PORT, MPU6050_ADDR, buf, 2, false);

    mpu6050_calibrate();
}

// --- Funções do Servo Motor ---
void servo_init() {
    gpio_set_function(SERVO_PIN, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(SERVO_PIN);
    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv(&config, 125.0f); 
    pwm_config_set_wrap(&config, 20000 - 1); 
    pwm_init(slice_num, &config, true);
}

void servo_set_angle(float angle) {
    // Mapeia o ângulo (-90 a 90) para o novo limite (-MAX_SERVO_ANGLE a +MAX_SERVO_ANGLE)
    if (angle > MAX_SERVO_ANGLE) angle = MAX_SERVO_ANGLE;
    if (angle < -MAX_SERVO_ANGLE) angle = -MAX_SERVO_ANGLE;
    
    uint16_t pulse_width = (angle + 90.0f) * (SERVO_MAX_US - SERVO_MIN_US) / 180.0f + SERVO_MIN_US;
    pwm_set_gpio_level(SERVO_PIN, pulse_width);
}

// --- Funções do Display OLED ---
void oled_setup() {
    ssd1306_init();
    frame_area.start_column = 0;
    frame_area.end_column = ssd1306_width - 1;
    frame_area.start_page = 0;
    frame_area.end_page = ssd1306_n_pages - 1;
    calculate_render_area_buffer_length(&frame_area);
}

int main() {
    stdio_init_all();
    sleep_ms(3000);

    // --- Inicialização do Hardware ---
    i2c_init(MPU6050_I2C_PORT, 400 * 1000);
    gpio_set_function(MPU6050_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(MPU6050_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(MPU6050_SDA_PIN);
    gpio_pull_up(MPU6050_SCL_PIN);
    
    i2c_init(OLED_I2C_PORT, 400 * 1000);
    gpio_set_function(OLED_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(OLED_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(OLED_SDA_PIN);
    gpio_pull_up(OLED_SCL_PIN);

    mpu6050_init(); // Já inclui a calibração
    servo_init();
    oled_setup();
    
    float angle_roll = 0.0f;
    absolute_time_t last_time = get_absolute_time();
    char angle_str[20];

    while (true) {
        int16_t accel_raw[3], gyro_raw[3];
        mpu6050_read_raw(accel_raw, gyro_raw);

        // Aplica a calibração
        float accel_corr[3], gyro_corr[3];
        accel_corr[0] = accel_raw[0] - accel_cal[0];
        accel_corr[1] = accel_raw[1] - accel_cal[1];
        accel_corr[2] = accel_raw[2] - accel_cal[2];
        gyro_corr[0] = gyro_raw[0] - gyro_cal[0];
        
        float dt = absolute_time_diff_us(last_time, get_absolute_time()) / 1000000.0f;
        last_time = get_absolute_time();

        float accel_angle_roll = atan2f(accel_corr[1], accel_corr[2]) * 180.0f / M_PI;
        angle_roll = 0.98f * (angle_roll + (gyro_corr[0] / 131.0f) * dt) + 0.02f * accel_angle_roll;

        servo_set_angle(angle_roll);

        // --- Saída Serial Detalhada ---
        printf("--- Leitura ---\n");
        printf("RAW  -> Accel X: %6d, Y: %6d, Z: %6d | Gyro X: %6d\n", accel_raw[0], accel_raw[1], accel_raw[2], gyro_raw[0]);
        printf("CALIB-> Accel X: %6.0f, Y: %6.0f, Z: %6.0f | Gyro X: %6.0f\n", accel_corr[0], accel_corr[1], accel_corr[2], gyro_corr[0]);
        printf("ANGULO FINAL: %.2f graus\n\n", angle_roll);
        
        memset(ssd_buffer, 0, ssd1306_buffer_length);
        snprintf(angle_str, sizeof(angle_str), "Angulo: %.1f", angle_roll);
        ssd1306_draw_string(ssd_buffer, 5, 10, angle_str);

        if (fabs(angle_roll) > 45.0f) {
            ssd1306_draw_line(ssd_buffer, 0, 38, 127, 38, true);
            ssd1306_draw_string(ssd_buffer, 5, 45, "INCLINACAO EXCEDIDA!");
        }

        render_on_display(ssd_buffer, &frame_area);

        sleep_ms(20);
    }
    return 0;
}