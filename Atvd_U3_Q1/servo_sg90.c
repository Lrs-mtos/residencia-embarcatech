#include <stdio.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "hardware/i2c.h"

/* ============================ CONSTANTES ============================ */

// --- Pinos ---
const uint SERVO_PIN   = 2;  // GP2 -> Sinal do Servo
const uint BTN_A_PIN   = 5;  // GP5 -> Botão A 
const uint BTN_B_PIN   = 6;  // GP6 -> Botão B 
const uint I2C_SDA_PIN = 0;  // GP0 -> I2C SDA
const uint I2C_SCL_PIN = 1;  // GP1 -> I2C SCL

// --- Servo ---
#define SERVO_NEUTRAL_US 1500.0f
#define SERVO_RANGE_US   1000.0f

// --- Sensor BH1750 ---
const int BH1750_ADDR = 0x23;
const uint8_t BH1750_POWER_ON = 0x01;
const uint8_t BH1750_CONTINUOUS_HIGH_RES = 0x10;

// --- LÓGICA DE CONTROLE (NOVAS REGRAS) ---
#define MAX_SERVO_SPEED 40         // Limite máximo de velocidade (-40 a 40)
#define MAX_LUX_FOR_SPEED 40.0f    // O valor de Lux que corresponde à velocidade máxima
#define SENSOR_READ_INTERVAL_MS 200 // Ler o sensor 5 vezes por segundo
#define DEBOUNCE_MS 250

/* ============================ VARIÁVEIS GLOBAIS DE ESTADO ============================ */

static volatile int8_t g_current_speed = 0;
static volatile bool g_is_active = false;             // Motor começa inativo
static volatile int8_t g_direction_multiplier = 1;    // 1 para direto, -1 para inverso
static volatile uint32_t last_press_time = 0;

/* ============================ FUNÇÕES DE HARDWARE E UTILITÁRIAS ============================ */

long map_value(long x, long in_min, long in_max, long out_min, long out_max) {
    // Evita divisão por zero
    if (in_max == in_min) return out_min;
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void servo_set_speed(int8_t new_speed) {
    // Aplica o limite global de velocidade
    if (new_speed < -MAX_SERVO_SPEED) new_speed = -MAX_SERVO_SPEED;
    if (new_speed > MAX_SERVO_SPEED)  new_speed = MAX_SERVO_SPEED;

    if (new_speed == g_current_speed) return;
    
    g_current_speed = new_speed;
    
    float us = SERVO_NEUTRAL_US + (g_current_speed / 100.0f) * SERVO_RANGE_US;
    
    uint slice = pwm_gpio_to_slice_num(SERVO_PIN);
    pwm_set_chan_level(slice, pwm_gpio_to_channel(SERVO_PIN), (uint16_t)us);
}

float bh1750_read_lux() {
    uint8_t buffer[2];
    if (i2c_read_blocking(i2c0, BH1750_ADDR, buffer, 2, false) < 2) return -1.0f;
    uint16_t raw_value = (buffer[0] << 8) | buffer[1];
    return raw_value / 1.2f;
}

/* ============================ LÓGICA DE CONTROLE PRINCIPAL ============================ */

void update_servo_from_light() {
    if (!g_is_active) return; // Se o sistema não estiver ativo, não faz nada

    float lux = bh1750_read_lux();
    if (lux < 0) {
        printf("Falha ao ler o sensor!\n");
        return;
    }

    // Garante que o lux não passe do limite para o cálculo
    if (lux > MAX_LUX_FOR_SPEED) {
        lux = MAX_LUX_FOR_SPEED;
    }

    // Calcula a magnitude da velocidade (0 a MAX_SERVO_SPEED) baseada na luz
    int8_t speed_magnitude = (int8_t)map_value(lux, 0, MAX_LUX_FOR_SPEED, 0, MAX_SERVO_SPEED);
    
    // Aplica a direção e atualiza o servo
    int8_t final_speed = speed_magnitude * g_direction_multiplier;
    servo_set_speed(final_speed);

    printf("Luz: %.2f Lux -> Velocidade: %d\n", lux, final_speed);
}

// NOVA LÓGICA PARA OS BOTÕES
void gpio_irq_handler(uint gpio, uint32_t events) {
    uint32_t now = to_ms_since_boot(get_absolute_time());
    if (now - last_press_time < DEBOUNCE_MS) return;
    last_press_time = now;

    // Se o motor já está ativo, qualquer botão o desliga
    if (g_is_active) {
        g_is_active = false;
        servo_set_speed(0);
        printf("\n--- MOTOR PARADO ---\n");
    } 
    // Se está parado, um botão o ativa e define a direção
    else {
        g_is_active = true;
        if (gpio == BTN_A_PIN) {
            g_direction_multiplier = 1; // Mais luz -> velocidade positiva
            printf("\n--- MOTOR ATIVADO (Direção Direta) ---\n");
        } else if (gpio == BTN_B_PIN) {
            g_direction_multiplier = -1; // Mais luz -> velocidade negativa
            printf("\n--- MOTOR ATIVADO (Direção Inversa) ---\n");
        }
    }
}

/* ============================ INICIALIZAÇÃO E LOOP PRINCIPAL ============================ */

void init_hardware() {
    stdio_init_all();
    sleep_ms(2000);

    // Servo PWM
    gpio_set_function(SERVO_PIN, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(SERVO_PIN);
    pwm_config cfg = pwm_get_default_config();
    pwm_config_set_clkdiv(&cfg, 125.0f);
    pwm_config_set_wrap(&cfg, 20000 - 1);
    pwm_init(slice, &cfg, true);
    servo_set_speed(0); // Garante que o motor comece parado

    // I2C Sensor
    i2c_init(i2c0, 100 * 1000);
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);
    i2c_write_blocking(i2c0, BH1750_ADDR, &BH1750_POWER_ON, 1, false);
    sleep_ms(10);
    i2c_write_blocking(i2c0, BH1750_ADDR, &BH1750_CONTINUOUS_HIGH_RES, 1, false);
    sleep_ms(120);

    // Botões com Interrupção
    gpio_init(BTN_A_PIN);
    gpio_set_dir(BTN_A_PIN, GPIO_IN);
    gpio_pull_up(BTN_A_PIN);
    gpio_init(BTN_B_PIN);
    gpio_set_dir(BTN_B_PIN, GPIO_IN);
    gpio_pull_up(BTN_B_PIN);
    gpio_set_irq_enabled_with_callback(BTN_A_PIN, GPIO_IRQ_EDGE_FALL, true, &gpio_irq_handler);
    gpio_set_irq_enabled(BTN_B_PIN, GPIO_IRQ_EDGE_FALL, true);
}

int main() {
    init_hardware();

    printf("\n--- Sistema de Controle de Servo v2 ---\n");
    printf("O motor está parado. Aperte um botão para começar.\n");
    printf("Botão A: Ativa/Para. Se ativar, mais luz = velocidade positiva.\n");
    printf("Botão B: Ativa/Para. Se ativar, mais luz = velocidade negativa.\n");

    absolute_time_t last_sensor_read_time = get_absolute_time();

    while (true) {
        // A lógica principal só roda se o sistema estiver ativo
        if (g_is_active) {
            if (absolute_time_diff_us(last_sensor_read_time, get_absolute_time()) > SENSOR_READ_INTERVAL_MS * 1000) {
                last_sensor_read_time = get_absolute_time();
                update_servo_from_light();
            }
        } else {
            // Se inativo, dorme por um tempo para não desperdiçar ciclos de CPU
            sleep_ms(20);
        }
    }
}