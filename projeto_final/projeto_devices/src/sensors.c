#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "common.h"
#include "aht10.h"
#include "mpu6050.h"
#include "sensors.h"

extern QueueHandle_t xStationQueue;
extern SemaphoreHandle_t xI2C0Mutex;

void vSensorTask(void *pvParameters) {
    station_data_t current_status;
    while (true) {
        // Proteção do barramento I2C0 (GP0/GP1)
        if (xI2C0Mutex != NULL && xSemaphoreTake(xI2C0Mutex, portMAX_DELAY) == pdTRUE) {
            aht_read_data(&current_status.temperature, &current_status.humidity);
            mpu_read_data(&current_status.roll, &current_status.drawer_open);
            xSemaphoreGive(xI2C0Mutex);
        }

        // Verifica a "saúde" interna da mini-estação
        current_status.alert_active = (current_status.temperature > 40.0f || current_status.drawer_open);
        
        if (xStationQueue != NULL) {
            xQueueOverwrite(xStationQueue, &current_status);
        }
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}