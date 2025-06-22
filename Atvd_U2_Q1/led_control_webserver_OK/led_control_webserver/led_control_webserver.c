#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "hardware/adc.h"
#include "lwip/pbuf.h"
#include "lwip/udp.h"
#include <string.h>
#include <stdio.h>

// --- CONFIGURAÇÕES - ALTERE AQUI ---
#define WIFI_SSID "TP-LINK_52D0"
#define WIFI_PASSWORD "92383104"

// IP e Porta do servidor que receberá os dados
#define SERVER_IP     "192.168.1.106"
#define SERVER_PORT   4444

// Pino do botão (conecte um botão entre o pino GP5 e o GND)
#define BUTTON_PIN    5

// Intervalo para enviar os dados (em milissegundos)
#define SEND_INTERVAL_MS 1000
// --- FIM DAS CONFIGURAÇÕES ---

// Variáveis globais para a conexão UDP
static struct udp_pcb *udp_pcb;
static ip_addr_t server_addr;

/*
Função (send_data_callback): É chamada periodicamente pelo timer. Lê o status do botão e do sensor de
temperatura, formata os dados em uma string e os envia via UDP para o servidor.

Entrada: arg: Um ponteiro para uma estrutura repeating_timer (não utilizado aqui).

Saída: bool: Retorna 'true' para que o timer continue a ser chamado.
*/
bool send_data_callback(struct repeating_timer *t) {
    // 1. Leitura do sensor de temperatura interno
    adc_select_input(4); // O canal 4 do ADC é o sensor de temperatura
    uint16_t raw_adc = adc_read();
    const float conversion_factor = 3.3f / (1 << 12);
    float voltage = raw_adc * conversion_factor;
    float temperature = 27.0f - (voltage - 0.706f) / 0.001721f;

    // 2. Leitura do status do botão
    // Usamos '!' pois o pino está em pull-up. Ele lê '0' quando pressionado.
    bool button_is_pressed = !gpio_get(BUTTON_PIN);

    // 3. Montagem da mensagem (payload)
    char payload[128];
    snprintf(payload, sizeof(payload), "{\"button_pressed\": %s, \"temperature_c\": %.2f}",
             button_is_pressed ? "true" : "false",
             temperature);

    // 4. Envio dos dados via UDP
    struct pbuf *p = pbuf_alloc(PBUF_TRANSPORT, strlen(payload), PBUF_RAM);
    if (p != NULL) {
        memcpy(p->payload, payload, strlen(payload));
        udp_sendto(udp_pcb, p, &server_addr, SERVER_PORT);
        pbuf_free(p);
        printf("Dados enviados: %s\n", payload);
    } else {
        printf("Erro ao alocar pbuf para envio UDP\n");
    }

    return true; // Continua o timer
}


int main() {
    stdio_init_all();

    // Inicializa a conexão Wi-Fi
    if (cyw43_arch_init()) {
        printf("Falha ao inicializar Wi-Fi\n");
        return -1;
    }
    cyw43_arch_enable_sta_mode();
    printf("Conectando ao Wi-Fi '%s'...\n", WIFI_SSID);

    if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 30000)) {
        printf("Falha ao conectar.\n");
        return -1;
    }
    printf("Conectado com sucesso!\n");
    printf("IP do dispositivo: %s\n", ip4addr_ntoa(netif_ip4_addr(netif_default)));

    // Inicializa o botão
    gpio_init(BUTTON_PIN);
    gpio_set_dir(BUTTON_PIN, GPIO_IN);
    gpio_pull_up(BUTTON_PIN); // Habilita resistor de pull-up interno

    // Inicializa o ADC para o sensor de temperatura
    adc_init();
    adc_set_temp_sensor_enabled(true);

    // Configura o cliente UDP
    udp_pcb = udp_new();
    if (udp_pcb) {
        printf("PCB UDP criado com sucesso!\n");
    }else{
        printf("Falha ao criar PCB UDP\n");
        return -1;
    }
    ipaddr_aton(SERVER_IP, &server_addr);

    // Cria um timer para chamar a função de envio de dados repetidamente
    struct repeating_timer timer;
    add_repeating_timer_ms(SEND_INTERVAL_MS, send_data_callback, NULL, &timer);

    // Loop infinito para manter o programa rodando
    while (true) {
        tight_loop_contents();
    }

    cyw43_arch_deinit();
    return 0;
}
