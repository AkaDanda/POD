#include "piston.h"
#include "driver/adc.h"


Piston piston_r = {
    .id = 0,
    .pin_scl_stoke = 11,
    .pin_sda_stoke = 10,
    .D23231_ADDRESS = 0x58,
    .adc_channel_stroke = ADC1_CHANNEL_6,
    .state = false,

    .stroke_measured = 0.0f,
    .stroke_target = 0.0f
};

Piston piston_l = {
    .id = 1,
    .pin_scl_stoke = 16,
    .pin_sda_stoke = 15,
    .D23231_ADDRESS = 0x58,
    .adc_channel_stroke = ADC1_CHANNEL_1,
    .state = false,

    .stroke_measured = 0.0f,
    .stroke_target = 0.0f
};

void piston_update_stroke_measured(Piston* piston, float new_value) {
    if (piston == NULL) return;  // sicurezza
    // TODO: add reading from ADC 
    piston->stroke_measured = new_value;
}