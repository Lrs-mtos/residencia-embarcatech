#include <stdio.h>
#include "pico/stdlib.h"
#include "FreeRTOS.h"
#include "task.h"
#include "ui.h"

// Pinos baseados no seu projeto
#define JOYSTICK_SEL_PIN 22
#define LED_FEEDBACK_PIN 16 

void vUITask(void *pvParameters) {
    // Inicializa o botão (entrada) e o LED (saída)
    gpio_init(JOYSTICK_SEL_PIN);
    gpio_set_dir(JOYSTICK_SEL_PIN, GPIO_IN);
    gpio_pull_up(JOYSTICK_SEL_PIN);

    gpio_init(LED_FEEDBACK_PIN);
    gpio_set_dir(LED_FEEDBACK_PIN, GPIO_OUT);

    while (true) {
        // Verifica se o botão do joystick foi pressionado
        if (!gpio_get(JOYSTICK_SEL_PIN)) {
            printf("[UI] Dispositivo Selecionado! Piscando LED...\n");

            // Requisito: Piscar exatamente 4 vezes
            for (int i = 0; i < 4; i++) {
                gpio_put(LED_FEEDBACK_PIN, 1);
                vTaskDelay(pdMS_TO_TICKS(200)); 
                gpio_put(LED_FEEDBACK_PIN, 0);
                vTaskDelay(pdMS_TO_TICKS(200));
            }
            vTaskDelay(pdMS_TO_TICKS(500)); // Debounce
        }
        vTaskDelay(pdMS_TO_TICKS(100)); // Polling suave
    }
}