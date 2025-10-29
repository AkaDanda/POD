#ifndef TANK_H
#define TANK_H

#include <stdint.h>
#include <stdbool.h>
#include "driver/adc.h"

typedef struct {
    uint8_t id;

    float pressure_measured;
    float pressure_filtered;
    float pressure_max_limit;
    float pressure_min_limit;

    // Canale ADC assegnato (es: ADC1_CHANNEL_5)
    adc1_channel_t adc_channel_pressure;

    bool state;
} Tank;

// Inizializza il tank con canale ADC e ID
void tank_init(Tank *tank, uint8_t id, adc1_channel_t adc_channel);

// Aggiorna pressione (da raw ADC)
void tank_pressure_update(Tank *tank);

#endif // TANK_H
