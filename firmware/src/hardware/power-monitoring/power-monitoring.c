/*******************************************************************************
 * @file    power-monitoring.c
 * @brief   Drivers for monitoring the battery voltage and current consumption.
 ******************************************************************************/

#include "power-monitoring.h"

#include <stdbool.h>

#include "adc.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "soc/adc_channel.h"

/** Configuration for the battery sensing ADC. Needs to read a range of
 * [0.4...1] V*/
static struct ADCChannel voltage_adc = {
    .adc = ADCPeripheral1,
    .channel2 = ADC1_GPIO36_CHANNEL,
    .attenuation = ADC_ATTEN_DB_11,
};

/** Configuration for the current sensing ADC. Needs to read a range of
 * [0...2.5] V */
static struct ADCChannel current_adc = {
    .adc = ADCPeripheral1,
    .channel1 = ADC1_GPIO35_CHANNEL,
    .attenuation = ADC_ATTEN_DB_11,
};

/** Tag used in logging. */
static const char *TAG = "power-monitoring";

void power_monitoring_task(void *pvParameters) {

    // Set up the two ADCs that are used for the battery voltage and current
    // consumption readings.
    adc_channel_configure(&voltage_adc);
    adc_channel_configure(&current_adc);

    while (true) {

        uint16_t battery_sens = adc_channel_sample(&voltage_adc);
        uint16_t current_sens = adc_channel_sample(&current_adc);

        // Convert to raw battery voltage
        // Step 1: Divide by 2, previous buffer had gain 2
        // Step 2: Reverse the voltage divider (499k+499k vs. 110k)
        float battery_voltage_mv =
            battery_sens / 2.0f * 1.0f / (110.0f / (499.0f + 499.0f + 110.0f));

        // Current is 1 mV / mA
        float current_consumption_ma = current_sens * 1.0f - 40;

        ESP_LOGI(TAG, "%f mV, %f mA", battery_voltage_mv,
                 current_consumption_ma);

        // In a future commit, publish to host computer
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
