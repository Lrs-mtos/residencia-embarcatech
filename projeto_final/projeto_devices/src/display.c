#include <stdio.h>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "common.h"
#include "display.h"

extern QueueHandle_t xStationQueue;

void vDisplayTask(void *pvParameters) {
    station_data_t data_to_show;
    while (true) {
        // Recebe dados da fila de monitoramento da mini-estação
        if (xStationQueue != NULL && xQueueReceive(xStationQueue, &data_to_show, portMAX_DELAY)) {
            printf("[DISPLAY] Temp: %.1f C | Gaveta: %s\n", 
                    data_to_show.temperature, 
                    data_to_show.drawer_open ? "ABERTA" : "FECHADA");
        }
    }
}