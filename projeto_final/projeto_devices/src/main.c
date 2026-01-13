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
SemaphoreHandle_t xI2C0Mutex = NULL;

// --- TAREFA DE STARTUP (O Cérebro do Boot) ---
void vStartupTask(void *pvParameters) {
    printf("\n[RTOS] Startup iniciado com sucesso!\n");

    /* * PASSO 1: WIFI (COMENTADO PARA TESTE)
     * Vamos pular o Wi-Fi por enquanto para evitar o 'PANIC size > 0'
     */
    /*
    printf("[SISTEMA] Inicializando hardware Wi-Fi...\n");
    if (cyw43_arch_init()) {
        printf("[ERRO] Falha no Wi-Fi!\n");
    } else {
        printf("[SISTEMA] Wi-Fi pronto!\n");
    }
    */

    // PASSO 2: Recursos de Comunicação
    xStationQueue = xQueueCreate(1, sizeof(station_data_t));
    xI2C0Mutex = xSemaphoreCreateMutex();

    if (xStationQueue == NULL || xI2C0Mutex == NULL) {
        printf("[ERRO] Falha ao alocar Filas/Mutex!\n");
        while(1);
    }

    // PASSO 3: Inicialização de Sensores Físicos
    mpu_init();
    aht_init();

    // PASSO 4: Criar apenas as tarefas básicas (Monitoramento Local)
    printf("[SISTEMA] Criando tarefas de monitoramento...\n");
    
    // Criamos com pilhas seguras, mas não exageradas
    xTaskCreate(vSensorTask,  "SENS", 1024, NULL, 2, NULL);
    xTaskCreate(vDisplayTask, "DISP", 1024, NULL, 1, NULL);
    xTaskCreate(vUITask,      "UI",   512,  NULL, 1, NULL);

    /* * PASSO 5: MQTT (COMENTADO)
     * Só reativaremos quando os sensores estiverem estáveis no terminal.
     */
    // xTaskCreate(vMqttTask, "MQTT", 2048, NULL, 1, NULL);

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

    // Hardware I2C (Pinos 0 e 1)
    i2c_init(i2c0, 100 * 1000);
    gpio_set_function(0, GPIO_FUNC_I2C);
    gpio_set_function(1, GPIO_FUNC_I2C);
    gpio_pull_up(0);
    gpio_pull_up(1);

    // Cria a tarefa de Startup para orquestrar o resto
    // Usamos 2048 aqui pois é a única tarefa rodando no início
    xTaskCreate(vStartupTask, "Start", 2048, NULL, 3, NULL);

    // Inicia o RTOS
    vTaskStartScheduler();

    while (true) {
        // O código nunca deve chegar aqui
    }
}