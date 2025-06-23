#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "hardware/adc.h"
#include "lwip/pbuf.h"
#include "lwip/udp.h"
#include <string.h>
#include <stdio.h>

// --- CONFIGURAÇÕES - ALTERE AQUI ---
#define WIFI_SSID "xxxxx"
#define WIFI_PASSWORD "xxxxx"
#define SERVER_IP     "192.168.1.106" // COLOQUE O IP DO SEU COMPUTADOR
#define SERVER_PORT   5555
#define SEND_INTERVAL_MS 500 // Enviar dados a cada meio segundo
// --- FIM DAS CONFIGURAÇÕES ---

// --- PINOS DO JOYSTICK ---
#define JOYSTICK_X_PIN 27 // Ligado ao ADC1
#define JOYSTICK_Y_PIN 26 // Ligado ao ADC0

// Limiares para a rosa dos ventos (0 a 4095)
#define THRESHOLD_LOW  1000 // Valor baixo para considerar "movido"
#define THRESHOLD_HIGH 3000 // Valor alto para considerar "movido"
// --- FIM DAS CONFIGURAÇÕES ---

// Variáveis globais para a conexão UDP
static struct udp_pcb *udp_pcb;
static ip_addr_t server_addr;

/*
função (get_compass_direction):
  Recebe os valores X e Y do joystick e retorna a string da direção
  correspondente na rosa dos ventos.
entrada:
  - x_val: Valor do eixo X (0-4095)
  - y_val: Valor do eixo Y (0-4095)
saída:
  - const char*: Ponteiro para a string da direção (ex: "Norte").
*/
const char* get_compass_direction(uint16_t x_val, uint16_t y_val) {
    bool up = y_val > THRESHOLD_HIGH;
    bool down = y_val < THRESHOLD_LOW;
    bool left = x_val < THRESHOLD_LOW;
    bool right = x_val > THRESHOLD_HIGH;

    if (up && right) return "Nordeste";
    if (up && left) return "Noroeste";
    if (down && right) return "Sudeste";
    if (down && left) return "Sudoeste";
    if (up) return "Norte";
    if (down) return "Sul";
    if (left) return "Oeste";
    if (right) return "Leste";

    return "Centro";
}

/*
função (send_data_callback):
  É chamada periodicamente. Lê o joystick, calcula a direção e envia os
  dados formatados via UDP para o servidor.
entrada:
  - t: Ponteiro para a estrutura repeating_timer (não utilizado).
saída:
  - bool: Retorna 'true' para manter o timer ativo.
*/
bool send_data_callback(struct repeating_timer *t) {
    // 1. Leitura dos valores crus X e Y do Joystick
    adc_select_input(1); // Eixo X está no ADC1 (GP27)
    uint16_t x_pos = adc_read();

    adc_select_input(0); // Eixo Y está no ADC0 (GP26)
    uint16_t y_pos = adc_read();

    // 2. Obter a direção da rosa dos ventos
    const char* direction = get_compass_direction(x_pos, y_pos);

    // 3. Montagem da mensagem (payload)
    char payload[128];
    snprintf(payload, sizeof(payload), "{\"x\": %d, \"y\": %d, \"direcao\": \"%s\"}",
             x_pos, y_pos, direction);

    // 4. Envio dos dados via UDP
    struct pbuf *p = pbuf_alloc(PBUF_TRANSPORT, strlen(payload), PBUF_RAM);
    if (p != NULL) {
        memcpy(p->payload, payload, strlen(payload));
        udp_sendto(udp_pcb, p, &server_addr, SERVER_PORT);
        pbuf_free(p);
        printf("Enviado: %s\n", payload);
    }

    return true;
}

/*
função (main):
  Função principal. Inicializa tudo (joystick, Wi-Fi), configura o cliente
  UDP e agenda o timer para o envio periódico dos dados.
entrada:
  - Nenhuma.
saída:
  - int: Código de retorno.
*/
int main() {
    stdio_init_all();

    // --- Inicialização do Joystick ---
    printf("Inicializando ADC para o joystick...\n");
    adc_init();
    adc_gpio_init(JOYSTICK_Y_PIN); // Habilita o pino para função de ADC
    adc_gpio_init(JOYSTICK_X_PIN); // Habilita o pino para função de ADC

    // Inicializa e conecta ao Wi-Fi
    cyw43_arch_init();
    cyw43_arch_enable_sta_mode();
    printf("Conectando a '%s'...\n", WIFI_SSID);
    if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 30000)) {
        printf("Falha ao conectar.\n");
        return -1;
    }
    printf("Conectado! IP: %s\n", ip4addr_ntoa(netif_ip4_addr(netif_default)));

    // Configura o cliente UDP
    udp_pcb = udp_new();
    ipaddr_aton(SERVER_IP, &server_addr);

    // Agenda o timer para enviar os dados
    struct repeating_timer timer;
    add_repeating_timer_ms(-SEND_INTERVAL_MS, send_data_callback, NULL, &timer);

    // Loop infinito para manter o programa rodando
    while (true) {
        tight_loop_contents();
    }
    
    return 0;
}