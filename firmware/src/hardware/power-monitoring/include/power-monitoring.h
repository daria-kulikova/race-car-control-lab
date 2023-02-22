/*******************************************************************************
 * @file    power-monitoring.h
 * @brief   Drivers for monitoring the battery voltage and current consumption.
 ******************************************************************************/

#pragma once

/**
 * @brief Enters the power monitoring task.
 * The power monitoring task continuously monitors the battery voltage and
 * current consumption.
 * @param pvParameters Arguments that can be passed on task instantiation.
 */
void power_monitoring_task(void *pvParameters);
