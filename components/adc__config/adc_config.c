#include "adc_config.h"
#include <stdlib.h>

void adc_init(esp_adc_cal_characteristics_t **adc_chars, adc1_channel_t channel) {
    // Configura larghezza ADC
    adc1_config_width(ADC_WIDTH);

    // Configura attenuazione
    adc1_config_channel_atten(channel, ADC_ATTEN);

    // Alloca e caratterizza
    *adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN, ADC_WIDTH, DEFAULT_VREF, *adc_chars);
}

//TODO: Include in the code the ADC initialization function