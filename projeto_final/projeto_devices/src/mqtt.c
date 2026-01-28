
#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"

#include "FreeRTOS.h"
#include "lwipopts.h"
#include "lwip/apps/mqtt.h"
#include "lwip/ip_addr.h"
#include "lwip/dns.h"
#include "queue.h"
#include "task.h"

#include "mqtt.h"
#include <stdio.h>
#include <string.h>
#include "common.h"


// Credenciais e Configurações
#define WIFI_SSID "TP-LINK_52D0"
#define WIFI_PASSWORD "92383104"
#define MQTT_BROKER "broker.hivemq.com"
#define MQTT_TOPIC "embarca/estacao/monitoramento"

extern QueueHandle_t xStationQueue;

static mqtt_client_t *mqtt_client;
static ip_addr_t broker_ip;
static bool mqtt_connected = false;
static bool wifi_connected = false;

// Callback de conexão MQTT
static void mqtt_connection_callback(mqtt_client_t *client, void *arg, mqtt_connection_status_t status) {
    if (status == MQTT_CONNECT_ACCEPTED) {
        printf("[MQTT] Conectado ao broker!\n"); //
        mqtt_connected = true;
    } else {
        printf("[MQTT] Falha na conexão: %d\n", status); //
        mqtt_connected = false;
    }
}

// Callback do DNS para resolver o IP do Broker
static void dns_callback(const char *name, const ip_addr_t *ipaddr, void *arg) {
    if (ipaddr) {
        broker_ip = *ipaddr;
        printf("[DNS] Resolvido: %s\n", ipaddr_ntoa(ipaddr)); //
        
        struct mqtt_connect_client_info_t ci = {
            .client_id = "pico_station_01",
            .keep_alive = 60
        };
        mqtt_client_connect(mqtt_client, &broker_ip, MQTT_PORT, mqtt_connection_callback, NULL, &ci); //
    }
}

// Task Principal de Comunicação
void vMqttTask(void *pvParameters) {
    station_data_t data;
    char payload[150];

    // Aguarda WiFi estar conectado antes de iniciar MQTT
    printf("[MQTT] Aguardando WiFi conectar...\n");
    while (!wifi_connected) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    printf("[MQTT] WiFi conectado, iniciando MQTT...\n");

    // 2. Inicializa Cliente MQTT
    mqtt_client = mqtt_client_new();
    dns_gethostbyname(MQTT_BROKER, &broker_ip, dns_callback, NULL);

    while (true) {
        cyw43_arch_poll(); // Mantém a pilha de rede viva

        if (mqtt_connected && xQueueReceive(xStationQueue, &data, 0)) {
            // Formata os dados em JSON para o Node-RED
            snprintf(payload, sizeof(payload), 
                     "{\"temp\":%.1f,\"hum\":%.1f,\"roll\":%.1f,\"open\":%s,\"alert\":%s}",
                     data.temperature, data.humidity, data.roll, 
                     data.drawer_open ? "true" : "false",
                     data.alert_active ? "true" : "false");

            mqtt_publish(mqtt_client, MQTT_TOPIC, payload, strlen(payload), 1, 0, NULL, NULL);
            printf("[MQTT] Publicado: %s\n", payload);
        }

        vTaskDelay(pdMS_TO_TICKS(5000)); // Publica a cada 5 segundos
    }
}

// ============================================================
// ============================================================
// WiFi Task - Reconexão e Polling (inicialização feita no main)
// ============================================================
void vWifiTask(void *pvParameters) {
    printf("[WiFi] WiFi Task iniciada - hardware já inicializado no main\n");

    // Loop de reconexão com timeout não-bloqueante
    int reconnect_attempts = 0;
    while (true) {
        if (!wifi_connected) {
            reconnect_attempts++;
            printf("[WiFi] Tentativa #%d de conexão a %s...\n", reconnect_attempts, WIFI_SSID);
            
            // 30 segundos de timeout
            int result = cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 30000);
            
            if (result == 0) {
                wifi_connected = true;
                printf("[WiFi] ✓ Conectado com sucesso!\n");
                reconnect_attempts = 0;
            } else {
                printf("[WiFi] ✗ Falha na conexão (código: %d)\n", result);
                
                // Após 3 tentativas, aguarda mais tempo
                if (reconnect_attempts >= 3) {
                    printf("[WiFi] Aguardando 10s antes de próxima tentativa...\n");
                    vTaskDelay(pdMS_TO_TICKS(10000));
                } else {
                    vTaskDelay(pdMS_TO_TICKS(2000));
                }
            }
        } else {
            // Mantém WiFi vivo com polling
            cyw43_arch_poll();
            vTaskDelay(pdMS_TO_TICKS(50)); // Poll a cada 50ms
        }
    }
}