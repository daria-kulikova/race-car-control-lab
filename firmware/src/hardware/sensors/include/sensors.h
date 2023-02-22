/*******************************************************************************
 * @file    sensors.h
 * @brief   Driver for talking to sensors.
 ******************************************************************************/

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "control-types.h"

/* Public type definitions -------------------------------------------------- */
/* Public function definitions ---------------------------------------------- */

/**
 * @brief Set up the sensors.
 */
void sensors_setup(void);

/**
 * @brief Sample the onboard sensors.
 *
 * Takes a sample of the onboard IMU sensor and writes the result into the
 * memory address pointed at by @p meas.
 * @param meas Memory address where the IMU sample should be stored.
 * @returns true if the sensor sampling was successful, false otherwise
 */
bool sensors_sample(InertialMeasurement *meas);

/**
 * @brief Sample the steering feedback sensor.
 */
uint16_t adc_sample();
