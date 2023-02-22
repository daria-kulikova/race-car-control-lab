/*******************************************************************************
 * @file    spi-master.c
 * @brief   SPI bus master driver
 ******************************************************************************/

#include "spi-master.h"

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_log.h"

/* Private type declarations ------------------------------------------------ */

/**
 * @brief Device handle combining the CS pin and the device handle from the
 * underlying driver.
 */
struct SPIDeviceHandle {
    spi_device_handle_t drive_dev_handle;
    gpio_num_t cs;
};

/* Private function declaration --------------------------------------------- */

/**
 * @brief Callback for pre-transaction actions on the SPI driver.
 *
 * This function is needed to fulfill the timing requirements of the BMI088
 * sensor: between the CS falling edge and the first CLK edge, there have to be
 * at least 40ns (the ESP32 does them at the same time). This function manually
 * puts CS low and then delays for >= 40ns before returning.
 * @note The pin that will be pulled low is set by the @link{spi_bus_write} or
 *  @link{spi_bus_read} function that is called by the BMI088 driver. This is
 *  NOT thread-safe, so the SPI interactions with the BMI088 can only come from
 *  a single thread! The pin to be pulled low is stored in @link{active_cs_pin}.
 * @param trans parameter from the SPI driver with metadata of the transaction
 */
void IRAM_ATTR pull_cs_low(spi_transaction_t *trans);

/**
 * @brief Counterpart callback for after SPI transaction has happened.
 *
 * Pulls CS back up after a transaction on the bus.
 * @note The pin that will be pulled low is set by the @link{spi_bus_write} or
 *  @link{spi_bus_read} function that is called by the BMI088 driver. This is
 *  NOT thread-safe, so the SPI interactions with the BMI088 can only come from
 *  a single thread! The pin to be pulled low is stored in @link{active_cs_pin}.
 * @param trans parameter from the SPI driver with metadata of the transaction
 */
void IRAM_ATTR pull_cs_high(spi_transaction_t *trans);

/* Local variable declaration ----------------------------------------------- */

static const char *TAG = "spi-master";

/* Public function implementation ------------------------------------------- */

bool spi_master_init(struct SPIMaster *master_cfg) {
    if (!master_cfg)
        return false;

    spi_bus_config_t bus_config = {
        .mosi_io_num = master_cfg->mosi,
        .miso_io_num = master_cfg->miso,
        .sclk_io_num = master_cfg->clk,
        .quadwp_io_num = -1, // don't use write protect or
        .quadhd_io_num = -1, // hold signals
    };

    esp_err_t ret = ESP_OK;

    // Initialize on SPI2 host, without DMA (set dma_chan to 0)
    ret = spi_bus_initialize(SPI2_HOST, &bus_config, 0);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI bus could not be initialized. Reason: %s",
                 esp_err_to_name(ret));
        return false;
    }

    return true;
}

struct SPIDeviceHandle *spi_master_add_bus_device(struct SPIMaster *master,
                                                  struct SPIDevice *dev) {

    spi_device_interface_config_t dev_config = {
        .command_bits = 0,
        .address_bits = 8,
        .dummy_bits = 0,
        .mode = 0,
        .spics_io_num = -1, // don't use CS from SPI driver
        .clock_speed_hz = dev->clock_speed,
        .flags = 0,
        .queue_size = 1,
        .pre_cb = &pull_cs_low,
        .post_cb = &pull_cs_high,
    };

    // Allocate space for a new SPI device, since we can't know in this driver
    // how many devices are goin gto be added
    struct SPIDeviceHandle *handle = malloc(sizeof(struct SPIDeviceHandle));

    if (!handle) {
        ESP_LOGE(TAG, "Failed to allocate memory for new SPI device!");
        return false;
    }

    // Here the device gets added and a handle is returned, which is then in
    // turn also returned to the user of this driver
    esp_err_t ret =
        spi_bus_add_device(SPI2_HOST, &dev_config, &handle->drive_dev_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Could not add bus device! Reason: %s",
                 esp_err_to_name(ret));
        free(handle);
        return false;
    }

    // Initialize CS pin
    gpio_reset_pin(dev->cs);
    gpio_set_direction(dev->cs, GPIO_MODE_OUTPUT);
    gpio_set_level(dev->cs, 1);

    handle->cs = dev->cs;

    ESP_LOGV(TAG, "SPI device successfully added to bus!");
    return handle;
}

int8_t spi_master_bus_write(uint8_t reg_addr, const uint8_t *data, uint32_t len,
                            struct SPIDeviceHandle *handle) {
    // Set up new transaction, chip select pin to be pulled low is passed as the
    // user field and then read in the .pre and .post callbacks
    spi_transaction_t trans = {
        .addr = reg_addr,
        .rx_buffer = NULL,
        .tx_buffer = data,
        .length = len * 8,
        .user = (void *)(handle->cs),
    };

    esp_err_t ret = spi_device_transmit(handle->drive_dev_handle, &trans);
    ESP_LOGD(TAG, "Write (%u B) @ %#02x: %#02x (%d)", len, reg_addr, *data,
             ret);

    return ret == ESP_OK ? 0 : -1;
}

int8_t spi_master_bus_read(uint8_t reg_addr, uint8_t *data, uint32_t len,
                           struct SPIDeviceHandle *handle) {
    // Set up new transaction, chip select pin to be pulled low is passed as the
    // user field and then read in the .pre and .post callbacks
    spi_transaction_t trans = {
        .addr = reg_addr,
        .rx_buffer = data,
        .tx_buffer = NULL,
        .length = len * 8,
        .user = (void *)(handle->cs),
    };

    esp_err_t ret = spi_device_transmit(handle->drive_dev_handle, &trans);
    ESP_LOGD(TAG, "Read (%u B) @ %#02x: %#02x (%d)", len, reg_addr, *data, ret);

    return ret == ESP_OK ? 0 : -1;
}

/* Private function implementation ------------------------------------------ */

void IRAM_ATTR pull_cs_low(spi_transaction_t *trans) {
    gpio_num_t cs = (gpio_num_t)(trans->user);
    if (cs != -1) {
        gpio_set_level(cs, 0);
        // BMI088 requires >= 40ns before clock starts
        // TODO: reduce this to 40ns, currently 1µs
        ets_delay_us(1);
    }
}

void IRAM_ATTR pull_cs_high(spi_transaction_t *trans) {
    gpio_num_t cs = (gpio_num_t)(trans->user);
    if (cs != -1) {
        gpio_set_level(cs, 1);
    }
};
