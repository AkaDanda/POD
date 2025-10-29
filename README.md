# POD
Embedded system for ESP32-S3 to control pneumatic pistons via ADC, DAC, I2C, and UART. Includes FreeRTOS tasks for stroke and pressure feedback, compressor control, and user interaction.


Features

Real-time control of pneumatic pistons
Stroke feedback via ADC and I2C
Pressure monitoring and compressor control
UART interface for command input and JSON data output
I2C communication with DAC for current control
Custom filtering algorithms for signal smoothing
Hardware Requirements

ESP32-S3 microcontroller
Analog stroke sensors
Analog pressure sensor
DAC device (e.g., D23231) with I2C interface
Compressors controlled via GPIO
UART interface for debugging and control
Software Components

ESP-IDF (FreeRTOS, ADC, GPIO, UART, I2C drivers)
cJSON library for JSON formatting
Custom modules:piston.h
filter.h
