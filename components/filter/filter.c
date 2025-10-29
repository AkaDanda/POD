#include "filter.h"

moving_avg_filter_t pressure_filter = {0};
moving_avg_filter_t stroke_filter_R = {0};
moving_avg_filter_t stroke_filter_L = {0};

float media_mobile_custom(moving_avg_filter_t *filter, float nuovo_valore)
{
    filter->somma -= filter->buffer[filter->index];
    filter->buffer[filter->index] = nuovo_valore;
    filter->somma += nuovo_valore;

    filter->index = (filter->index + 1) % WINDOW_SIZE;
    if (filter->count < WINDOW_SIZE)
        filter->count++;

    return filter->somma / filter->count;
}