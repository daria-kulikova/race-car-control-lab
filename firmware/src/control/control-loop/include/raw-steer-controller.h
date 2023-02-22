/*******************************************************************************
 * @file    raw-steer-controller.h
 * @brief   Steer controller that tracks the raw potentiometer measurement
 ******************************************************************************/

#pragma once

#include "control.h"

/**
 * @brief Iterate the controller that tracks a raw feedback voltage reference.
 *
 * Instead of tracking a steering angle, the raw feedback voltage controller
 * tracks the feedback voltage of the potentiometer and does not apply any model
 * knowledge or limits on what the minimum/maximum reference is that can be
 * tracked.
 *
 * Internally, this raw controller is a P controller with a gain of 1/(1000mV).
 *
 * @param ref The reference voltage to be tracked
 * @param y The current feedback voltage of the steering potentiometer
 * @param u The normalized actuator input that should be applied
 */
void control_raw_steer_step(uint16_t ref, uint16_t y, float *u);
