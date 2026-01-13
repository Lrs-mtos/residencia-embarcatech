#ifndef COMMON_H
#define COMMON_H

#include <stdbool.h>

// Estrutura única para telemetria da estação
typedef struct {
    float temperature;
    float humidity;
    float roll;          // Inclinação lateral
    bool drawer_open;    // Status da gaveta
    bool alert_active;   // Alerta de saúde/segurança
} station_data_t;

#endif // COMMON_H