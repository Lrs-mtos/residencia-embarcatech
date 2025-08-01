/*
 * Descrição: Este código lê dados de um sensor de cor I2C
 * e os publica em um tópico MQTT via Wi-Fi.
 */

#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "hardware/i2c.h"
#include "lwip/apps/mqtt.h"
#include "lwip/ip_addr.h"
#include "lwip/dns.h"

// --- Wi-Fi ---
#define WIFI_SSID "TP-LINK_52D0"
#define WIFI_PASSWORD "92383104"

// --- MQTT ---
#define MQTT_BROKER "broker.hivemq.com"
#define MQTT_BROKER_PORT 1883
#define MQTT_TOPIC "embarca/sensor/color" 
#define MQTT_CLIENT_ID "pico_w_color_sensor"

// --- Sensor I2C ---
#define I2C_PORT i2c0
#define I2C_SDA_PIN 0 
#define I2C_SCL_PIN 1 
#define SENSOR_ADDR 0x29 // Endereço I2C padrão 

// Intervalo (em milissegundos)
#define PUBLISH_INTERVAL_MS 1000

static mqtt_client_t *mqtt_client;
static ip_addr_t broker_ip;
static bool mqtt_connected = false;

// Dados de cor
typedef struct {
    uint16_t r, g, b, c;
} color_data_t;

// Protótipos de funções
static void mqtt_connection_callback(mqtt_client_t *client, void *arg, mqtt_connection_status_t status);
void publish_color_data(color_data_t data);
void dns_check_callback(const char *name, const ip_addr_t *ipaddr, void *callback_arg);
void init_color_sensor();
void read_color_sensor(color_data_t *data);

int main() {
    stdio_init_all();
    sleep_ms(2000); // Aguarda a inicialização do terminal serial
    printf("\n=== Iniciando Sensor de Cor MQTT ===\n");

    // Inicializa Wi-Fi
    if (cyw43_arch_init()) {
        printf("ERRO: Falha ao inicializar o Wi-Fi.\n");
        return -1;
    }
    cyw43_arch_enable_sta_mode();
    printf("[Wi-Fi] Conectando a '%s'...\n", WIFI_SSID);
    if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 30000)) {
        printf("ERRO: Falha ao conectar ao Wi-Fi.\n");
        return -1;
    }
    printf("[Wi-Fi] Conectado com sucesso!\n");

    // Inicializa o sensor de cor
    init_color_sensor();

    // Inicializa cliente MQTT
    mqtt_client = mqtt_client_new();
    if (mqtt_client == NULL) {
        printf("ERRO: Falha ao criar cliente MQTT.\n");
        return -1;
    }

    // Resolve o endereço do broker MQTT via DNS
    printf("[DNS] Resolvendo o IP para '%s'...\n", MQTT_BROKER);
    err_t err = dns_gethostbyname(MQTT_BROKER, &broker_ip, dns_check_callback, NULL);
    if (err == ERR_OK) {
        // O endereço já estava em cache, conecta diretamente
        dns_check_callback(MQTT_BROKER, &broker_ip, NULL);
    } else if (err != ERR_INPROGRESS) {
        printf("ERRO: Falha imediata no DNS: %d\n", err);
    }

    // Loop principal
    while (true) {
        cyw43_arch_poll(); // Mantém a conexão de rede ativa

        if (mqtt_connected) {
            color_data_t current_color;
            read_color_sensor(&current_color);
            publish_color_data(current_color);
        }
        
        sleep_ms(PUBLISH_INTERVAL_MS);
    }
}

// Funções do sensor de cor

void init_color_sensor() {
    // Inicializa I2C
    i2c_init(I2C_PORT, 100 * 1000); // 100 kHz
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);
    printf("[Sensor] I2C inicializado.\n");

    // Liga o sensor 
    uint8_t txdata[] = {0x80, 0x03}; // Endereço do registrador de Enable + valor (PON | AEN)
    if(i2c_write_blocking(I2C_PORT, SENSOR_ADDR, txdata, 2, false) != 2) {
        printf("ERRO: Não foi possível ligar o sensor.\n");
    } else {
        printf("[Sensor] Sensor de cor ativado.\n");
    }
    sleep_ms(10);
}

void read_color_sensor(color_data_t *data) {
    uint8_t reg = 0x94; // Endereço inicial dos dados de cor (CDATAL, RDATAL, etc.)
    uint8_t buffer[8];

    i2c_write_blocking(I2C_PORT, SENSOR_ADDR, &reg, 1, true);
    int bytes_read = i2c_read_blocking(I2C_PORT, SENSOR_ADDR, buffer, 8, false);

    if (bytes_read == 8) {
        data->c = (buffer[1] << 8) | buffer[0];
        data->r = (buffer[3] << 8) | buffer[2];
        data->g = (buffer[5] << 8) | buffer[4];
        data->b = (buffer[7] << 8) | buffer[6];
        printf("[Sensor] Leitura: R=%d, G=%d, B=%d, C=%d\n", data->r, data->g, data->b, data->c);
    } else {
        printf("ERRO: Falha ao ler dados do sensor I2C. Lidos %d bytes.\n", bytes_read);
        data->r = data->g = data->b = data->c = 0;
    }
}

// Callback de conexão MQTT
static void mqtt_connection_callback(mqtt_client_t *client, void *arg, mqtt_connection_status_t status) {
    if (status == MQTT_CONNECT_ACCEPTED) {
        printf("[MQTT] Conectado ao broker!\n");
        mqtt_connected = true;
    } else {
        printf("ERRO: Falha na conexão MQTT. Código: %d\n", status);
        mqtt_connected = false;
    }
}

// Publica os dados do sensor
void publish_color_data(color_data_t data) {
    if (!mqtt_connected) {
        printf("[MQTT] Não conectado, publicação cancelada.\n");
        return;
    }

    char message[100];
    // Formata a mensagem como um objeto JSON
    snprintf(message, sizeof(message), "{\"r\":%d,\"g\":%d,\"b\":%d,\"c\":%d}", data.r, data.g, data.b, data.c);

    printf("[MQTT] Publicando em '%s': %s\n", MQTT_TOPIC, message);

    err_t err = mqtt_publish(mqtt_client, MQTT_TOPIC, message, strlen(message), 1, 0, NULL, NULL); // QoS 1
    if (err != ERR_OK) {
        printf("ERRO: Falha ao publicar mensagem MQTT: %d\n", err);
    }
}

// Callback do DNS
void dns_check_callback(const char *name, const ip_addr_t *ipaddr, void *callback_arg) {
    if (ipaddr) {
        broker_ip = *ipaddr;
        printf("[DNS] Resolvido: %s -> %s\n", name, ipaddr_ntoa(ipaddr));

        struct mqtt_connect_client_info_t ci = {
            .client_id = MQTT_CLIENT_ID,
            .keep_alive = 60,
        };

        printf("[MQTT] Conectando ao broker...\n");
        mqtt_client_connect(mqtt_client, &broker_ip, MQTT_BROKER_PORT, mqtt_connection_callback, NULL, &ci);
    } else {
        printf("ERRO: Falha ao resolver DNS para %s\n", name);
    }
}