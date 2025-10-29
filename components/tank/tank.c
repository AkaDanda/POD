#include tank.h
#include "driver/adc.h"

Tank tank = {
    .id = 0,
    .pressure_measured = 0.0f,
    .pressure_filtered = 0.0f,
    .pressure_max_limit = 8.0f, //expressed in bar
    .pressure_min_limit = 5.0f,   
    .adc_channel_pressure = ADC1_CHANNEL_5, //GPIO6
    .state = false
};

void tank_pressure_update(Tank *tank) {
    if (tank == NULL) return;  // safety check
}