#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "driver/uart.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include <stdlib.h>
#include "driver/ledc.h"
#include "esp_err.h"
#include <time.h>
#include "driver/i2c_master.h"
#include "math.h"
#include "cJSON.h"
#include "../components/piston/piston.h"
#include "../components/filter/filter.h"


/*THR PID SETTINGS ARE THE BASIC FROM STACCATO APP AND ARTE THE FOLLOWING
GAIN = 2
FREQ FROM 50Hz TO 100Hz
MAX SPEED = 1000 mm/s

FILTER WINDOW SIZE = 20
*/

// I2C
#define D23231_ADDRESS 0x58 // I2C address of the D23231 device
#define SDA_PIN_2 8
#define SCL_PIN_2 9

// Compressors
#define COMP1_PIN 18
#define COMP2_PIN 17
#define MAX_PRESS 6
#define MIN_PRESS 4

// UART
#define UART_NUM UART_NUM_1
#define UART_TX_PIN 4
#define UART_RX_PIN 5

// Low-pass filter settings
#define ALPHA 0.1                  // Smoothing factor (0.0 < ALPHA < 1.0)
#define MIN_MOVEMENT_THRESHOLD 2.0 // Minimum change to react (adjust based on your sensor's noise level)

// Variable to store the last filtered sensor reading
static float filtered_value = 0.0;

typedef struct
{
    uint16_t D_offset;     // DAC code for 4mA (e.g. ~655)
    uint16_t D_full_scale; // DAC code for 20mA (e.g. ~3276)
} dac_calibration_t;

dac_calibration_t dac_calibration = {
    .D_offset = 655,     // Example value for 4mA calibration
    .D_full_scale = 3276 // Example value for 20mA calibration
};


// UART port used for communication
const uart_port_t uart_num = UART_NUM_2;

// Global variables to store the pressure and stroke values
float pressure_value = 0.0;

// Reference voltage used for ADC calibration (in mV)
#define DEFAULT_VREF 1100 // Used if eFuse Vref is not available
#define NO_OF_SAMPLES 64  // Number of samples for multisampling (averaging)

// Window size for moving average filter
#define WINDOW_SIZE 20

// Buffers and indices for moving average calculation (pressure and stroke)
float buffer[WINDOW_SIZE];
int buffer_index = 0;
int count = 0;

// Tag used for logging
static const char *TAG = "ADC_STROKE";
static const char *TAG_1 = "ADC_PRESSURE";

// ADC Configuration
#define ADC_CHANNEL_STROKE_R ADC1_CHANNEL_6
#define ADC_CHANNEL_STROKE_L ADC1_CHANNEL_1 // GPIO7 (on ESP32-S3, ADC1_CHANNEL_6 is GPIO7)
#define ADC_CHANNEL_PRESSURE ADC1_CHANNEL_5 // GPIO6
#define ADC_WIDTH ADC_WIDTH_BIT_12          // 12-bit ADC resolution (0 - 4095)
#define ADC_ATTEN ADC_ATTEN_DB_12           // 12dB attenuation, full voltage input range (~3.3V)
#define ADC_UNIT ADC_UNIT_1                 // Use ADC unit 1

// Pointer to store ADC calibration characteristics
static esp_adc_cal_characteristics_t *adc_chars = NULL;

float current_command(int stroke_read, i2c_master_dev_handle_t dev_handel_core, dac_calibration_t *calib)
{
    if (stroke_read < 0)
        stroke_read = 0;
    if (stroke_read > 800)
        stroke_read = 800;

    // Map stroke_read (0–800mm) to current (4–20mA)
    float current_mA = 4.0f + ((float)stroke_read / 800.0f) * 16.0f;

    // Calculate DAC digital code with calibration
    uint16_t _dac_4 = calib->D_offset;      // Calibrated DAC code for 4mA
    uint16_t _dac_20 = calib->D_full_scale; // Calibrated DAC code for 20mA

    uint16_t digital_code;

    if (stroke_read == 0)
    {
        digital_code = _dac_4;
    }
    else if (stroke_read == 800)
    {
        digital_code = _dac_20;
    }
    else
    {
        digital_code = _dac_4 + (uint16_t)(((current_mA - 4.0f) * (_dac_20 - _dac_4)) / 16.0f + 0.5f);
    }

    uint8_t write_buffer[3];
    write_buffer[0] = 0x02;                       // Register address
    write_buffer[1] = (digital_code << 4) & 0xF0; // Upper nibble is upper 4 bits of digital code
    write_buffer[2] = (digital_code >> 4) & 0xFF; // Lower byte is lower 8 bits of digital code

    esp_err_t err = i2c_master_transmit(dev_handel_core, write_buffer, sizeof(write_buffer), pdMS_TO_TICKS(100));
    if (err != ESP_OK)
    {
        printf("I2C transmit failed: %s\n", esp_err_to_name(err));
    }

    vTaskDelay(pdMS_TO_TICKS(10));

    return current_mA;
}

float adc_to_pressure(int adc_value)
{
    return 0.00415 * adc_value - 2.28;
}

void Keyboard_task(void *parameters)
{
    const uart_port_t uart_console = UART_NUM_0; // UART0 è collegato alla console via USB
    char input[32];
    int index = 0;

    // Configurazione UART0
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE};
    uart_param_config(uart_console, &uart_config);
    uart_driver_install(uart_console, 1024, 0, 0, NULL, 0);

    printf("\n[Keyboard] Enter new stroke target (mm): ");
    fflush(stdout);

    while (1)
    {
        uint8_t ch;
        int len = uart_read_bytes(uart_console, &ch, 1, pdMS_TO_TICKS(100));
        if (len > 0)
        {
            if (ch == '\r' || ch == '\n')
            {
                input[index] = '\0';
                float value = atof(input);
                if (value >= 0 && value <= 800)
                {
                    piston_r.stroke_target = value;
                    printf("\n[Keyboard] stroke_target updated to: %.2f mm\n", piston_r.stroke_target);
                }
                else
                {
                    printf("\n[Keyboard] Invalid value. Enter a value between 0 and 800.\n");
                }
                index = 0;
                printf("\n[Keyboard] Enter new stroke target (mm): ");
                fflush(stdout);
            }
            else if (index < sizeof(input) - 1)
            {
                input[index++] = ch;
                uart_write_bytes(uart_console, (const char *)&ch, 1); // Echo del carattere
            }
        }
    }
}

void pressure_sensor_task(void *parameters)
{
    // Configure ADC width (bit resolution)
    adc1_config_width(ADC_WIDTH);

    // Configure ADC channel attenuation
    adc1_config_channel_atten(ADC_CHANNEL_PRESSURE, ADC_ATTEN);

    // Allocate memory for ADC calibration characteristics
    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));

    // Characterize the ADC to get accurate voltage values
    esp_adc_cal_characterize(ADC_UNIT, ADC_ATTEN, ADC_WIDTH, DEFAULT_VREF, adc_chars);

    // Configure the GPIO pin
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << COMP1_PIN) | (1ULL << COMP2_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE};
    gpio_config(&io_conf);

    while (1)
    {
        int adc_reading = adc1_get_raw(ADC_CHANNEL_PRESSURE);

        float raw_pressure = adc_to_pressure(adc_reading);

        pressure_value = media_mobile_custom(&pressure_filter, raw_pressure);

        // ESP_LOGI(TAG_1, "Raw: %d\tRaw Pressure: %fBar\tPressure: %f Bar", adc_reading, raw_pressure, pressure_value);

        if (pressure_value < MIN_PRESS)
        {
            gpio_set_level(COMP1_PIN, 1);
            gpio_set_level(COMP2_PIN, 1);
        }
        else if (pressure_value >= MAX_PRESS)
        {
            gpio_set_level(COMP1_PIN, 0);
            gpio_set_level(COMP2_PIN, 0);
        }

        vTaskDelay(100 / portTICK_PERIOD_MS); // Delay 100ms
    }
    vTaskDelete(NULL);
}

void I2C_Position_Target(void *parameters)
{
    i2c_master_bus_config_t i2c_master_bus_config_2 = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_NUM_1,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
        .scl_io_num = SCL_PIN_2,
        .sda_io_num = SDA_PIN_2};
    i2c_master_bus_handle_t i2c_master_bus_handle_2;
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_master_bus_config_2, &i2c_master_bus_handle_2));

    i2c_device_config_t i2c_device_config_2 = {
        .scl_speed_hz = 100000,
        .device_address = D23231_ADDRESS};
    i2c_master_dev_handle_t dev_handle_target;
    ESP_ERROR_CHECK(i2c_master_bus_add_device(i2c_master_bus_handle_2, &i2c_device_config_2, &dev_handle_target));
    while (1)
    {
        current_command((int)piston_r.stroke_target, dev_handle_target, &dac_calibration);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void I2C_Position_FDBK_R(void *parameters)
{
    i2c_master_bus_config_t i2c_master_bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_NUM_0,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
        .scl_io_num = piston_r.pin_scl_stoke,
        .sda_io_num = piston_r.pin_sda_stoke};
    i2c_master_bus_handle_t i2c_master_bus_handle;
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_master_bus_config, &i2c_master_bus_handle));

    i2c_device_config_t i2c_device_config = {
        .scl_speed_hz = 100000,
        .device_address = D23231_ADDRESS};
    i2c_master_dev_handle_t dev_handel;
    i2c_master_bus_add_device(i2c_master_bus_handle, &i2c_device_config, &dev_handel);
    
    // Configure ADC width (bit resolution)
    adc1_config_width(ADC_WIDTH);

    // Configure ADC channel attenuation
    adc1_config_channel_atten(piston_r.adc_channel_stroke, ADC_ATTEN);

    // Allocate memory for ADC calibration characteristics
    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));

    // Characterize the ADC to get accurate voltage values
    esp_adc_cal_characterize(ADC_UNIT, ADC_ATTEN, ADC_WIDTH, DEFAULT_VREF, adc_chars);

    while (true)
    {
        int adc_reading = adc1_get_raw(piston_r.adc_channel_stroke);
        // Convert the raw ADC reading to a voltage value in millivolts
        int voltage = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars); // This line didn't give accurate result, esp_adc_cal_raw_to_voltage() this function has to be checked carefully with the ESP32-S3.
        
        float stroke_raw = adc_reading * 800 / 4096;
        piston_update_stroke_measured(&piston_r, media_mobile_custom(&stroke_filter_R, stroke_raw)); // Update the stroke measured value in the piston structure
        current_command((int)piston_r.stroke_measured, dev_handel, &dac_calibration); // I chganged the function to take the stroke_target as an argument
        vTaskDelay(pdMS_TO_TICKS(0));
    }
}

void I2C_Position_FDBK_L(void *parameters)
{
    i2c_master_bus_config_t i2c_master_bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_NUM_0,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
        .scl_io_num = piston_l.pin_scl_stoke,
        .sda_io_num = piston_l.pin_sda_stoke};
    i2c_master_bus_handle_t i2c_master_bus_handle;
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_master_bus_config, &i2c_master_bus_handle));

    i2c_device_config_t i2c_device_config = {
        .scl_speed_hz = 100000,
        .device_address = D23231_ADDRESS};
    i2c_master_dev_handle_t dev_handel;
    i2c_master_bus_add_device(i2c_master_bus_handle, &i2c_device_config, &dev_handel);

    /*adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC1_CHANNEL_7, ADC_ATTEN_DB_11);*/

    // ADC Initialization
    // moving_avg_filter_t stroke_filter = {0}; // Inizializzazione a zero

    // Configure ADC width (bit resolution)
    adc1_config_width(ADC_WIDTH);

    // Configure ADC channel attenuation
    adc1_config_channel_atten(piston_l.adc_channel_stroke, ADC_ATTEN);

    // Allocate memory for ADC calibration characteristics
    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));

    // Characterize the ADC to get accurate voltage values
    esp_adc_cal_characterize(ADC_UNIT, ADC_ATTEN, ADC_WIDTH, DEFAULT_VREF, adc_chars);

    while (true)
    {
        int adc_reading = adc1_get_raw(piston_l.adc_channel_stroke);
        // Convert the raw ADC reading to a voltage value in millivolts
        int voltage = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars); // This line didn't give accurate result, esp_adc_cal_raw_to_voltage() this function has to be checked carefully with the ESP32-S3.
        /**
         * Custom conversion from raw ADC reading to a "stroke" value.
         * Assuming:
         *   - 4096 raw value corresponds to 800 mm
         *   - Linear relationship between raw and mm
         *   - stroke = (adc_reading / 4096) * 800
         */
        float stroke_raw = adc_reading * 800 / 4096;
        piston_update_stroke_measured(&piston_l, media_mobile_custom(&stroke_filter_L, stroke_raw)); //!!!!!!!!!1 USA UN ALTRO FILTRO PER QUEST MEDIA !!!!!!!!!!
        printf("Stroke Filtered: %.2f mm\n", piston_l.stroke_measured);
        current_command((int)piston_l.stroke_measured, dev_handel, &dac_calibration); // I chganged the function to take the stroke_target as an argument
        vTaskDelay(pdMS_TO_TICKS(0));
    }
}

void monitor_task(void *parameters)
{
    while (true)
    {
        printf("MONITOR YOUR VARIABLES HERE\n");
        printf("Stroke Filtered: %.2f mm\n", piston_l.stroke_measured);
        vTaskDelay(pdMS_TO_TICKS(50)); // Delay 1 second
    }
}

void task_uart_send(void *pvParameters)
{
    while (1) {
        cJSON *root = cJSON_CreateObject();
        cJSON *esp = cJSON_AddObjectToObject(root, "ESP");

        cJSON *cylinders = cJSON_AddArrayToObject(esp, "cylinders");

        cJSON *cyl0 = cJSON_CreateObject();
        cJSON_AddNumberToObject(cyl0, "id", 0);
        cJSON_AddNumberToObject(cyl0, "stroke_target", 5.80);
        cJSON_AddNumberToObject(cyl0, "stroke_position", 4.12);
        cJSON_AddItemToArray(cylinders, cyl0);

        cJSON *cyl1 = cJSON_CreateObject();
        cJSON_AddNumberToObject(cyl1, "id", 1);
        cJSON_AddNumberToObject(cyl1, "stroke_target", 5.75);
        cJSON_AddNumberToObject(cyl1, "stroke_position", 4.00);
        cJSON_AddItemToArray(cylinders, cyl1);

        cJSON *cyl2 = cJSON_CreateObject();
        cJSON_AddNumberToObject(cyl2, "id", 2);
        cJSON_AddNumberToObject(cyl2, "stroke_target", 5.90);
        cJSON_AddNumberToObject(cyl2, "stroke_position", 4.25);
        cJSON_AddItemToArray(cylinders, cyl2);

        cJSON *tank = cJSON_AddObjectToObject(esp, "tank");
        cJSON_AddNumberToObject(tank, "pressure", 8.5);

        cJSON *compressor = cJSON_AddArrayToObject(esp, "compressor");

        cJSON *comp0 = cJSON_CreateObject();
        cJSON_AddNumberToObject(comp0, "id", 0);
        cJSON_AddNumberToObject(comp0, "status", 1);
        cJSON_AddItemToArray(compressor, comp0);

        cJSON *comp1 = cJSON_CreateObject();
        cJSON_AddNumberToObject(comp1, "id", 1);
        cJSON_AddNumberToObject(comp1, "status", 0);
        cJSON_AddItemToArray(compressor, comp1);

        char *json_str = cJSON_PrintUnformatted(root);

        uart_write_bytes(UART_NUM, json_str, strlen(json_str));
        uart_write_bytes(UART_NUM, "\n", 1);  // newline come delimitatore

        cJSON_Delete(root);
        free(json_str);

        vTaskDelay(pdMS_TO_TICKS(1000)); // invia ogni secondo
    }
}

void task_uart_receive(void *pvParameters)
{
    uint8_t rx_buffer[128];

    while (1) {
        int len = uart_read_bytes(UART_NUM, rx_buffer, sizeof(rx_buffer) - 1, pdMS_TO_TICKS(200));
        if (len > 0) {
            rx_buffer[len] = 0; // terminatore stringa
            printf("Ricevuto: %s\n", rx_buffer);

            float value = atof((char *)rx_buffer);
            if (value >= 0 && value <= 800) {
                piston_r.stroke_target = value;
                printf("stroke_target aggiornato a: %.2f mm\n", piston_r.stroke_target);
            } else {
                printf("Valore non valido. Inserire un valore tra 0 e 800.\n");
            }
        }
    }
}


void app_main(void)
{
    //xTaskCreate(task_uart_send, "task_uart_send", 4096, NULL, 10, NULL);
    //xTaskCreate(task_uart_receive, "task_uart_receive", 4096, NULL, 9, NULL);
    // Create the ADC reading task
    //xTaskCreate(uart_task, "uart_task", 4096, NULL, 5, NULL);
    // xTaskCreate(pressure_sensor_task, "pressure_sensor_task", 4096, NULL, 5, NULL);
    //xTaskCreatePinnedToCore(Keyboard_task, "keyboard_task", 4096, NULL, 5, NULL, 1);
    xTaskCreatePinnedToCore(I2C_Position_FDBK_R, "I2C_Position_FDBK", 4096, NULL, 10, NULL, 0);
    //xTaskCreatePinnedToCore(I2C_Position_Target, "I2C_Position_Target", 4096, NULL, 5, NULL, 1);
    // xTaskCreate(monitor_task, "monitor_task", 2048, NULL, 1, NULL);
    //xTaskCreatePinnedToCore(I2C_Position_FDBK_L, "I2C_Position_FDBK", 4096, NULL, 10, NULL, 0);

}
