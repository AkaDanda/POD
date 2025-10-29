#ifndef PISTON_H
#define PISTON_H

#include <stdint.h>
#include <stdbool.h>
#include "driver/adc.h"

typedef struct {
    
    uint8_t id;
    
    //PINOUT
    uint8_t pin_scl_stoke;
    uint8_t pin_sda_stoke;
    uint8_t  D23231_ADDRESS;
    int adc_channel_stroke;
    
    //
    bool state;
    float stroke_measured;
    float stroke_target;
} Piston;

extern Piston piston_r;
extern Piston piston_l;

//Stroke Measurement Update
void piston_update_stroke_measured(Piston* piston, float new_value);
void piston_set_stroke_target(Piston* piston, float target); // TODO: Implement this function


#endif // PISTONE_H