/*******************************************************************************
 * @file    steer-identification.c
 * @brief   Finds the limits of the potentiometer for the car the PCB is on.
 ******************************************************************************/

#include "steer-identification.h"

#include <stdbool.h>

#include "actuators.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sensors.h"

/* Local variables ---------------------------------------------------------- */

static const char *TAG = "steer-id";

/* Public function implementation ------------------------------------------- */

bool steer_identification_find_limits(struct PotentiometerLimits *limits) {
    if (!limits)
        return false;

    // Start steering left continuously, let the angle settle
    apply_steer(+0.55);
    vTaskDelay(pdMS_TO_TICKS(1000));

    unsigned long adc_sum = 0;
    for (unsigned int i = 0; i < 100; i++) {
        adc_sum += adc_sample();
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    limits->upper = adc_sum / (100UL);

    // Start steering right continously, let the angle settle
    apply_steer(-0.55);
    vTaskDelay(pdMS_TO_TICKS(1000));

    adc_sum = 0;
    for (unsigned int i = 0; i < 100; i++) {
        adc_sum += adc_sample();
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    limits->lower = adc_sum / (100UL);
    ESP_LOGI(TAG, "Lower/upper poti limit: %u/%u", limits->lower,
             limits->upper);

    apply_steer(0.0);

    return true;
}
