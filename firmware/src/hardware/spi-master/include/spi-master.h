/*******************************************************************************
 * @file    spi-master.h
 * @brief   SPI bus master driver
 ******************************************************************************/

#pragma once

#include <stdbool.h>

#include "driver/gpio.h"

/* Public type declarations ------------------------------------------------- */

/**
 * @brief Configuration for an SPI master.
 *
 * The SPI master bus can be configured as follows:
 * - set the pin numbers required in this struct
 * - call spi_master_init()
 * - add devices on the bus by specifying their CS pins and clock speeds
 *      by calling spi_master_add_bus_device()
 * - store the SPIDevice structs and pass them to spi_bus_write and spi_bus_read
 *      functions to talk to a specific device
 */
struct SPIMaster {
    gpio_num_t mosi; ///< Master Out Slave In Pin
    gpio_num_t miso; ///< Master In Slave Out Pin
    gpio_num_t clk;  ///< Clock Pin
};

/**
 * @brief Configuration of a device on an SPI bus.
 *
 * Call spi_master_add_bus_device with this struct and an initialized SPI Master
 * to connect them. In return you will receive an (SPIDeviceHandle *) that you
 * have to store and pass to spi_bus_write() or spi_bus_read() to initiate
 * communication with this device.
 */
struct SPIDevice {
    gpio_num_t cs;        ///< Chip Select pin for this device
    uint32_t clock_speed; ///< Desired clock speed
};

/** A handle returned by the spi_master_add_bus_device() function that can be
 *  used to talk to a device on the SPI bus. */
struct SPIDeviceHandle;

/* Public function declarations --------------------------------------------- */

/**
 * @brief Initialize the SPI master.
 * @returns true if the initialization was successful, false otherwise.
 */
bool spi_master_init(struct SPIMaster *master_cfg);

/**
 * @brief Add an SPI device on the bus.
 *
 * First initialize the master using spi_master_init(). If successful, devices
 * can be added on the bus using this function.
 * @note The handle that is returned must be stored. Transactions on the bus
 *  using spi_master_bus_read() or spi_master_bus_write() must be passed these
 *  handles to know which device to talk to.
 * @param master An already initialized SPI master.
 * @param dev The bus device to register with the SPI master.
 * @return A struct SPIDeviceHandle* handle to initiate communication with the
 * device using spi_master_bus_write() or spi_master_bus_read().
 */
struct SPIDeviceHandle *spi_master_add_bus_device(struct SPIMaster *master,
                                                  struct SPIDevice *dev);

/**
 * @brief Write data to a register address of a device on the SPI bus.
 *
 * @param reg_addr The 8-bit register address targeted, including R/W bit.
 * @param data The data to write on the bus after the register address
 * @param len Number of bytes contained in @p data
 * @param handle Handle obtained from spi_master_add_bus_device() for this
 * device.
 * @note Make sure that the R/W bit (MSB) is set appropriately to what the
 * device expects. It can be set through (reg_addr | 0x80) to read for some
 * devices (e.g. BMI088).
 * @return Zero in case of success, non-zero otherwise.
 */
int8_t spi_master_bus_write(uint8_t reg_addr, const uint8_t *data, uint32_t len,
                            struct SPIDeviceHandle *handle);

/**
 * @brief Read data from a register address of a device on the SPI bus.
 *
 * @param reg_addr The 8-bit register address targeted, including R/W bit.
 * @param data Pointer to where the data read from the bus should be written.
 * @param len Number of bytes to read from bus and to write to @p data.
 * @param handle Handle obtained from spi_master_add_bus_device() for this
 * device.
 * @note Make sure that the R/W bit (MSB) is set appropriately to what the
 * device expects. It can be set through (reg_addr | 0x80) to read for some
 * devices (e.g. BMI088).
 * @return Zero in case of success, non-zero otherwise.
 */
int8_t spi_master_bus_read(uint8_t reg_addr, uint8_t *data, uint32_t len,
                           struct SPIDeviceHandle *handle);
