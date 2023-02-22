/*******************************************************************************
 * @file    actuators.h
 * @brief   Bridge between the DC motor drivers and the control loop.
 ******************************************************************************/

#pragma once

#include <stdint.h>

/** @defgroup actuators Actuators */

/**
 * @addtogroup actuators
 * @brief Module containing the conversion from control references to actuator
 *        inputs and drivers for interfacing with the motor controller chips.
 *
 * @{
 */

/* Public function declaration ---------------------------------------------- */

/**
 * @brief Set up all peripherals for interaction with the actuators.
 *
 * This setup configures two PWM channels, four digital output pins and an
 * analog input pin to steer the steering and driving motors and to get steering
 * feedback. The configuration is according to the values in the sdkconfig file,
 * which can be edited by using the `idf.py menuconfig` command.
 * @note The pins specified in the configuration must be able to do the required
 *  tasks (ADC input, PWM output), otherwise this function causes undefined
 *  behaviour.
 */
void actuator_setup(void);

/**
 * @brief Change the torque continuously applied to the steering actuator.
 *
 * When this function is called, the steering actuator starts changing the
 * current steering angle to steer more to the left or to the right. Note that
 * this corresponds to a CHANGE in angle and not the ABSOLUTE angle. Direction
 * and power (=> speed) of angle change can be specified. To hold the steering
 * angle where it is, specify power = 0.0.
 *
 * The steering input is specified in a range of [-1, 1]. The sign denotes the
 * direction the actuator should turn: if @p steer_input > 0, the steering
 * actuator is powered so that the steering angle is changed in the positive
 * direction, which by convention would be to steer left. If @p steer_input < 0
 * on the other hand, the actuators start turning right.
 *
 * @param steer_input The percentage of power [-1, 1] applied to the steering
 * actuators.
 */
void apply_steer(float steer_input);

/**
 * @brief Change the torque continously applied to the throttle actuator.
 *
 * When this function is called, the driving actuators will be driven at a
 * percentage of the maximum torque. They will apply torque indefinitely until
 * a new input is given, i.e., the function is called again. To apply no torque,
 * set power = 0.0. To apply maximum torque, set power = 1.0. The sign of @p
 * throttle_input denotes the direction where the car should drive:
 * @p throttle_input > 0 propels the car forward, @p throttle_input < 0
 * reverses.
 * @param throttle_input The percentage of power [-1, 1] applied to the torque
 * actuator.
 */
void apply_torque(float throttle_input);

/** @} */
