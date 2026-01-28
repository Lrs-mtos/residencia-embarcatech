#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h" // Necessário para controle de baixo nível do Wi-Fi
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "common.h"
#include "sensors.h"
#include "display.h"
#include "mpu6050.h"
#include "aht10.h"
#include "mqtt.h"
#include "ui.h"

// Handles Globais
QueueHandle_t xStationQueue = NULL;
SemaphoreHandle_t xI2C1Mutex = NULL;
SemaphoreHandle_t xI2C0Mutex = NULL;

void vStartupTask(void *pvParameters) {
    printf("\n[RTOS] Startup iniciado com sucesso!\n");

    // Recursos de Comunicação
    xStationQueue = xQueueCreate(1, sizeof(station_data_t));
    xI2C0Mutex = xSemaphoreCreateMutex();
    xI2C1Mutex = xSemaphoreCreateMutex();

    if (xStationQueue == NULL || xI2C0Mutex == NULL || xI2C1Mutex == NULL) {
        printf("[ERRO] Falha ao alocar Filas/Mutex!\n");
        while(1);
    }

    // PASSO 3: Inicialização de Sensores Físicos
    printf("\n--- INICIALIZANDO SENSORES ---\n");
    mpu_init();
    aht_init();
    
    // PASSO 4: Criar apenas as tarefas básicas (Monitoramento Local)
    printf("[SISTEMA] Criando tarefas de monitoramento...\n");
    
    // Reduzidas ao necessário para liberar espaço para WiFi
    xTaskCreate(vSensorTask,  "SENS", 512, NULL, 2, NULL);
    xTaskCreate(vDisplayTask, "DISP", 512, NULL, 1, NULL);
    xTaskCreate(vUITask,      "UI",   256, NULL, 1, NULL);

    /* * PASSO 5: MQTT (COMENTADO POR ENQUANTO)
     * Habilitar depois que WiFi estiver estável
     */
    // printf("[SISTEMA] Criando MQTT task...\n");
    // xTaskCreate(vMqttTask, "MQTT", 1024, NULL, 1, NULL);

    printf("[SISTEMA] Startup finalizado. Deletando tarefa Start.\n");
    vTaskDelete(NULL); 
}

int main() {
    // Inicializa USB e aguarda para você poder ler o terminal
    stdio_init_all();
    for(int i = 0; i < 30; i++) {
        if (stdio_usb_connected()) break;
        sleep_ms(100);
    }

    printf("\n--- MODO DE TESTE: MONITORAMENTO LOCAL ---\n");

    printf("\n--- INICIALIZANDO PINOS AHT10 ---\n");
    // Hardware I2C (Pinos 0 e 1)
    i2c_init(i2c0, 100 * 1000);
    gpio_set_function(AHT10_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(AHT10_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(AHT10_SDA_PIN);
    gpio_pull_up(AHT10_SCL_PIN);

    printf("\n--- INICIALIZANDO PINOS MPU6050 ---\n");
    // Hardware I2C (Pinos 2 e 3)
    i2c_init(i2c1, 100 * 1000);
    gpio_set_function(MPU6050_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(MPU6050_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(MPU6050_SDA_PIN);
    gpio_pull_up(MPU6050_SCL_PIN);

    // Inicializa WiFi ANTES do RTOS (com toda memória disponível)
    // COMENTADO: cyw43_arch_init() está travando - requer debug com openocd/gdb
    // printf("[WiFi] Inicializando hardware CYW43 antes do RTOS...\n");
    // printf("[WiFi] Chamando cyw43_arch_init()...\n");
    // fflush(stdout);
    // 
    // int wifi_init_result = cyw43_arch_init();
    // 
    // printf("[WiFi] cyw43_arch_init() retornou: %d\n", wifi_init_result);
    // fflush(stdout);
    // 
    // if (wifi_init_result) {
    //     printf("[ERRO] Falha ao iniciar WiFi - continuando sem WiFi\n");
    // } else {
    //     printf("[WiFi] Hardware CYW43 inicializado com sucesso!\n");
    //     printf("[WiFi] Ativando modo STA...\n");
    //     fflush(stdout);
    //     
    //     cyw43_arch_enable_sta_mode();
    //     printf("[WiFi] Modo STA ativado\n");
    //     fflush(stdout);
    // }

    // Cria a tarefa de Startup para orquestrar o resto
    // Stack reduzida pois deleta a si mesma após inicializar
    xTaskCreate(vStartupTask, "Start", 512, NULL, 3, NULL);

    // Inicia o RTOS
    vTaskStartScheduler();

    while (true) {

    }
}