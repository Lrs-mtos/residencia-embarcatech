#ifndef MQTT_H
#define MQTT_H

#include "common.h"

void mqtt_task_init();
void vMqttTask(void *pvParameters);
void vWifiTask(void *pvParameters);
void wifi_connect();

#endif