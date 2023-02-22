/*******************************************************************************
 * @file    sensors.c
 * @brief   Driver for talking to sensors.
 ******************************************************************************/

#include "sensors.h"

#include <stdbool.h>

#include "adc.h"
#include "bmi088.h"
#include "bmi08x.h"
#include "driver/adc.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "spi-master.h"

/* Local macros ------------------------------------------------------------- */

/** Chip Select pin to talk to the accelerometer part of the BMI088. */
#define BMI088_ACCEL_CS_PIN (gpio_num_t) CONFIG_SENSOR_ACCEL_CS_PIN
/** Chip Select pin to talk to the gyroscope part of the BMI088. */
#define BMI088_GYRO_CS_PIN (gpio_num_t) CONFIG_SENSOR_GYRO_CS_PIN

#define SPI_CLOCK_PIN (gpio_num_t) CONFIG_SENSOR_SPI_CLK_PIN
#define SPI_MOSI_PIN (gpio_num_t) CONFIG_SENSOR_SPI_MOSI_PIN
#define SPI_MISO_PIN (gpio_num_t) CONFIG_SENSOR_SPI_MISO_PIN
#define SPI_CLOCK_SPEED (gpio_num_t) CONFIG_SPI_BUS_SPEED

/* Local type definitions --------------------------------------------------- */

/**
 * Type that combines a SPI bus device and a corresponding CS pin.
 *
 * This combination is needed in callbacks that happen before and after a
 * transaction happens, when the CS pins need to be pulled manually to satisfy
 * timing constraints.
 */
struct spi_bus_cs_combination {
    spi_device_handle_t dev;
    gpio_num_t cs;
};

typedef struct spi_bus_cs_combination *spi_bus_cs_t;

/* Local variables ---------------------------------------------------------- */

/** Tag used in logging */
static const char *TAG = "sensors";

/** Struct identifying the BMI088 intertial measurement unit. */
static struct BMI088 bmi088 = {
    .bus_read = (bmi_bus_read_function)&spi_master_bus_read,
    .bus_write = (bmi_bus_write_function)&spi_master_bus_write,
};

static struct ADCChannel steering_fb_adc = {
    .adc = ADCPeripheral1,
    .channel1 = ADC1_CHANNEL_6,
    .attenuation = ADC_ATTEN_11db,
};

/* Public function implementation ------------------------------------------- */

void sensors_setup(void) {

    adc_channel_configure(&steering_fb_adc);

    // Set up the SPI bus where most of the sensors are at
    struct SPIMaster master_cfg = {
        .mosi = (gpio_num_t)CONFIG_SENSOR_SPI_MOSI_PIN,
        .miso = (gpio_num_t)CONFIG_SENSOR_SPI_MISO_PIN,
        .clk = (gpio_num_t)CONFIG_SENSOR_SPI_CLK_PIN,
    };

    if (!spi_master_init(&master_cfg)) {
        ESP_LOGE(TAG, "Failed to set up SPI master!");
        return;
    }

    // Add two SPI devices: accelerometer and gyroscope. Their handles are
    // stored directly by the sensor struct, which can then pass them to the
    // bus read/write hfunctions
    struct SPIDevice accel_cfg = {
        .cs = (gpio_num_t)CONFIG_SENSOR_ACCEL_CS_PIN,
        .clock_speed = SPI_CLOCK_SPEED,
    };

    struct SPIDevice gyro_cfg = {
        .cs = (gpio_num_t)CONFIG_SENSOR_GYRO_CS_PIN,
        .clock_speed = SPI_CLOCK_SPEED,
    };

    bmi088.intf_ptr_accel = spi_master_add_bus_device(&master_cfg, &accel_cfg);
    bmi088.intf_ptr_gyro = spi_master_add_bus_device(&master_cfg, &gyro_cfg);

    ESP_LOGI(TAG, "SPI2 master set up, initializing BMI088...");

    // bmi.accel_fsr = FSR_3G;
    // bmi.gyro_fsr = FSR_250DPS;

    if (!bmi088_init(&bmi088)) {
        ESP_LOGE(TAG, "Failed to set up BMI088 sensor!");
    }

    ESP_LOGI(TAG, "Reached end of sensor initialization!");
}

bool sensors_sample(InertialMeasurement *meas) {
    return bmi088_sample(&bmi088, meas);
}

uint16_t adc_sample() { return adc_channel_sample(&steering_fb_adc); }
