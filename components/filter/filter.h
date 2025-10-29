#ifndef FILTER_H
#define FILTER_H

#include <stdint.h>
#include <stdbool.h>

#define WINDOW_SIZE 20 // Size of the moving average window

typedef struct
{
    float buffer[WINDOW_SIZE];
    float somma;
    int index;
    int count;
} moving_avg_filter_t;

float media_mobile_custom(moving_avg_filter_t *filter, float nuovo_valore);

extern moving_avg_filter_t pressure_filter; 
extern moving_avg_filter_t stroke_filter_R;
extern moving_avg_filter_t stroke_filter_L; 

#endif // PISTONE_H