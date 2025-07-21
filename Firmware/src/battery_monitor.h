#ifndef BATTERY_MONITOR_H
#define BATTERY_MONITOR_H

#include <zephyr/types.h>
#include <zephyr/device.h>

// Battery monitoring configuration
#define BATTERY_ADC_RESOLUTION      12
#define BATTERY_ADC_GAIN           ADC_GAIN_1_6
#define BATTERY_ADC_REFERENCE      ADC_REF_INTERNAL
#define BATTERY_ADC_ACQ_TIME_US    40

// Voltage divider constants
#define VOLTAGE_DIVIDER_R1         1500000  // 1.5MΩ
#define VOLTAGE_DIVIDER_R2         220000   // 220kΩ
#define VOLTAGE_DIVIDER_RATIO      ((float)(VOLTAGE_DIVIDER_R2) / (float)(VOLTAGE_DIVIDER_R1 + VOLTAGE_DIVIDER_R2))

// Battery voltage range
#define BATTERY_VOLTAGE_MIN        3.0f     // 3.0V
#define BATTERY_VOLTAGE_MAX        4.2f     // 4.2V

// Thread configuration
#define BATTERY_THREAD_PRIORITY    5
#define BATTERY_THREAD_STACK_SIZE  1024
#define BATTERY_SAMPLE_INTERVAL_MS 30000    // 30 seconds
#define BATTERY_SERVICE_UPDATE_INTERVAL_MS 30000 // 30 seconds

// Function prototypes
int battery_monitor_init(void);
uint8_t battery_get_soc(void);
float battery_get_voltage(void);
uint16_t battery_get_raw_adc(void);
int battery_read_now(void);

#endif /* BATTERY_MONITOR_H */