#include "battery_monitor.h"
#include <zephyr/kernel.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/logging/log.h>
#include <math.h>

LOG_MODULE_REGISTER(battery_monitor, LOG_LEVEL_INF);

// Battery lookup table entry
typedef struct {
    uint16_t adc_value;    // 12-bit ADC reading
    uint8_t soc_percent;   // State of charge percentage (0-100)
} battery_lut_entry_t;

// Placeholder lookup table - REPLACE WITH REAL DATA AFTER CHARACTERIZATION
static const battery_lut_entry_t battery_voltage_soc_lut[] = {
    {557, 100},  // 4.2V = 100% SoC
    {540, 95},   // ~4.1V = 95% SoC
    {525, 90},   // ~4.0V = 90% SoC
    {510, 80},   // ~3.88V = 80% SoC
    {495, 70},   // ~3.75V = 70% SoC
    {480, 60},   // ~3.63V = 60% SoC
    {465, 50},   // ~3.51V = 50% SoC
    {460, 40},   // ~3.39V = 40% SoC
    {445, 30},   // ~3.27V = 30% SoC
    {430, 20},   // ~3.16V = 20% SoC
    {420, 10},   // ~3.08V = 10% SoC
    {412, 5},    // ~3.02V = 5% SoC
    {408, 0}     // 3.0V = 0% SoC
};

#define BATTERY_LUT_SIZE (sizeof(battery_voltage_soc_lut) / sizeof(battery_voltage_soc_lut[0]))

// ADC related variables
static const struct device *adc_dev = NULL;
static struct adc_channel_cfg channel_cfg = {
    .gain = BATTERY_ADC_GAIN,
    .reference = BATTERY_ADC_REFERENCE,
    .acquisition_time = ADC_ACQ_TIME(ADC_ACQ_TIME_MICROSECONDS, BATTERY_ADC_ACQ_TIME_US),
    .channel_id = 2,  // AIN2
    .input_positive = SAADC_CH_PSELP_PSELP_AnalogInput2,
};

static struct adc_sequence sequence = {
    .channels = BIT(2),  // Channel 2
    .buffer = NULL,      // Will be set before read
    .buffer_size = 0,    // Will be set before read
    .resolution = BATTERY_ADC_RESOLUTION,
    .oversampling = 4,   // 4x oversampling
};

// Battery state variables
static uint16_t last_adc_value = 0;
static uint8_t current_soc = 0;
static float current_voltage = 0.0f;
static struct k_mutex battery_mutex;
static bool battery_initialized = false;

// Moving average filter
#define FILTER_SIZE 8
static uint16_t adc_filter_buffer[FILTER_SIZE];
static uint8_t filter_index = 0;
static bool filter_initialized = false;

// Work queue for battery monitoring
static struct k_work_delayable battery_work;

/**
 * @brief Convert ADC value to battery voltage
 * @param adc_value 12-bit ADC reading
 * @return Battery voltage in volts
 */
static float adc_to_voltage(uint16_t adc_value)
{
    // Calculate ADC voltage based on reference (0.6V) and gain (1/6)
    float adc_voltage = (adc_value * 0.6f * 6.0f) / 4095.0f;
    
    // Calculate actual battery voltage using divider ratio
    float battery_voltage = adc_voltage / VOLTAGE_DIVIDER_RATIO;
    
    return battery_voltage;
}

/**
 * @brief Convert ADC reading to State of Charge percentage using LUT
 * @param adc_value 12-bit ADC reading
 * @return SoC percentage (0-100), or 255 if error
 */
static uint8_t adc_to_soc(uint16_t adc_value)
{
    // Bounds checking
    if (adc_value < battery_voltage_soc_lut[BATTERY_LUT_SIZE - 1].adc_value) {
        return 0;  // Below minimum battery voltage
    }
    
    if (adc_value >= battery_voltage_soc_lut[0].adc_value) {
        return 100;  // At or above maximum battery voltage
    }
    
    // Linear interpolation between lookup table points
    for (int i = 0; i < BATTERY_LUT_SIZE - 1; i++) {
        if (adc_value >= battery_voltage_soc_lut[i + 1].adc_value) {
            uint16_t adc_high = battery_voltage_soc_lut[i].adc_value;
            uint16_t adc_low = battery_voltage_soc_lut[i + 1].adc_value;
            uint8_t soc_high = battery_voltage_soc_lut[i].soc_percent;
            uint8_t soc_low = battery_voltage_soc_lut[i + 1].soc_percent;
            
            // Linear interpolation
            uint8_t soc = soc_low + 
                ((adc_value - adc_low) * (soc_high - soc_low)) / 
                (adc_high - adc_low);
            return soc;
        }
    }
    
    return 255;  // Error case
}

/**
 * @brief Apply moving average filter to ADC value
 * @param new_value New ADC reading
 * @return Filtered ADC value
 */
static uint16_t apply_filter(uint16_t new_value)
{
    // Initialize filter buffer on first use
    if (!filter_initialized) {
        for (int i = 0; i < FILTER_SIZE; i++) {
            adc_filter_buffer[i] = new_value;
        }
        filter_initialized = true;
    }
    
    // Add new value to buffer
    adc_filter_buffer[filter_index] = new_value;
    filter_index = (filter_index + 1) % FILTER_SIZE;
    
    // Calculate average
    uint32_t sum = 0;
    for (int i = 0; i < FILTER_SIZE; i++) {
        sum += adc_filter_buffer[i];
    }
    
    return (uint16_t)(sum / FILTER_SIZE);
}

/**
 * @brief Read battery ADC and update values
 * @return 0 on success, negative errno on error
 */
static int battery_read_adc(void)
{
    int16_t adc_buffer;
    int ret;
    
    if (!battery_initialized || !adc_dev) {
        return -ENODEV;
    }
    
    sequence.buffer = &adc_buffer;
    sequence.buffer_size = sizeof(adc_buffer);
    
    ret = adc_read(adc_dev, &sequence);
    if (ret < 0) {
        LOG_ERR("ADC read failed: %d", ret);
        return ret;
    }
    
    // Apply filter
    uint16_t filtered_value = apply_filter((uint16_t)adc_buffer);
    
    k_mutex_lock(&battery_mutex, K_FOREVER);
    last_adc_value = filtered_value;
    current_voltage = adc_to_voltage(filtered_value);
    current_soc = adc_to_soc(filtered_value);
    k_mutex_unlock(&battery_mutex);
    
    LOG_INF("Battery: ADC=%d, Voltage=%.2fV, SoC=%d%%", 
            filtered_value, current_voltage, current_soc);
    
    return 0;
}

/**
 * @brief Battery monitoring work handler
 */
static void battery_work_handler(struct k_work *work)
{
    battery_read_adc();
    
    // Reschedule the work
    k_work_reschedule(&battery_work, K_MSEC(BATTERY_SAMPLE_INTERVAL_MS));
}

/**
 * @brief Initialize battery monitoring
 * @return 0 on success, negative errno on error
 */
int battery_monitor_init(void)
{
    int ret;
    
    if (battery_initialized) {
        LOG_WRN("Battery monitor already initialized");
        return 0;
    }
    
    k_mutex_init(&battery_mutex);
    
    // Get ADC device
    adc_dev = DEVICE_DT_GET(DT_NODELABEL(adc));
    if (!device_is_ready(adc_dev)) {
        LOG_ERR("ADC device not ready");
        return -ENODEV;
    }
    
    // Configure ADC channel
    ret = adc_channel_setup(adc_dev, &channel_cfg);
    if (ret < 0) {
        LOG_ERR("ADC channel setup failed: %d", ret);
        return ret;
    }
    
    // Initialize work item
    k_work_init_delayable(&battery_work, battery_work_handler);
    
    battery_initialized = true;
    
    // Perform initial reading after a short delay to ensure system stability
    k_work_reschedule(&battery_work, K_MSEC(1000));
    
    LOG_INF("Battery monitor initialized successfully");
    
    return 0;
}

/**
 * @brief Get current battery state of charge
 * @return SoC percentage (0-100)
 */
uint8_t battery_get_soc(void)
{
    uint8_t soc;
    if (!battery_initialized) {
        return 0;
    }
    k_mutex_lock(&battery_mutex, K_FOREVER);
    soc = current_soc;
    k_mutex_unlock(&battery_mutex);
    return soc;
}

/**
 * @brief Get current battery voltage
 * @return Battery voltage in volts
 */
float battery_get_voltage(void)
{
    float voltage;
    if (!battery_initialized) {
        return 0.0f;
    }
    k_mutex_lock(&battery_mutex, K_FOREVER);
    voltage = current_voltage;
    k_mutex_unlock(&battery_mutex);
    return voltage;
}

/**
 * @brief Get raw ADC value
 * @return 12-bit ADC value
 */
uint16_t battery_get_raw_adc(void)
{
    uint16_t adc;
    if (!battery_initialized) {
        return 0;
    }
    k_mutex_lock(&battery_mutex, K_FOREVER);
    adc = last_adc_value;
    k_mutex_unlock(&battery_mutex);
    return adc;
}

/**
 * @brief Trigger immediate battery reading
 * @return 0 on success, negative errno on error
 */
int battery_read_now(void)
{
    if (!battery_initialized) {
        return -ENODEV;
    }
    
    // Cancel any pending work and execute immediately
    k_work_cancel_delayable(&battery_work);
    return battery_read_adc();
}