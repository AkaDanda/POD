#ifndef ADC_CONFIG_H
#define ADC_CONFIG_H

#include "driver/adc.h"
#include "esp_adc_cal.h"

#define DEFAULT_VREF 1100 // o il tuo valore
#define ADC_ATTEN    ADC_ATTEN_DB_11
#define ADC_WIDTH    ADC_WIDTH_BIT_12

void adc_init(esp_adc_cal_characteristics_t **adc_chars, adc1_channel_t channel);

#endif // ADC_CONFIG_H