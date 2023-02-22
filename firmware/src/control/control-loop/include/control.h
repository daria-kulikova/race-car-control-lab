/*******************************************************************************
 * @file    control.h
 * @brief   Low-level control loop task for steering and driving
 ******************************************************************************/

#pragma once

#include <stdbool.h>

#include "actuators.h"
#include "freertos/FreeRTOS.h"

/** @defgroup control Control Loop */

/**
 * @addtogroup control
 * @brief The control loop which samples the sensors and calculates the control
 * input, which is then applied to the actuators.
 *
 * The control component contains the control loop, which can be started and
 * stopped. When it is started, the control loop always runs according to the
 * following schedule:
 *
 * 1. All onboard sensors are sampled.
 * 2. The new control reference is fetched from the global variable.
 * 3. The active controller iteration functions are invoked, which are provided
 *    with the current reference and the sensor measurements.
 * 4. The return value of the invoked controller functions, the plant inputs,
 *    are applied to the actuators.
 * 5. The sensor measurements and the control input are sent to the host
 *    computer.
 * 6. The control loop task delays until the sampling period has elapsed.
 *
 * The control loop exposes an API that can be used by the rest of the system to
 * manage its state: it can be started and stopped, and the reference it tries
 * to track can be changed. New references are received via Wi-Fi by different
 * components of the software, and they may use control_update_reference() to
 * make the control loop track a new one.
 * @{
 */

/* Public type declaration -------------------------------------------------- */

/**
 * @brief A steering reference that the car should track.
 *
 * A steering reference normally consists of the angle of the steering wheels.
 * However, in certain (mostly system identification) cases, a reference may
 * also be given in terms of the raw voltage of the feedback potentiometer to be
 * tracked.
 */
struct SteeringReference {
    /** Type of the reference, i.e., which of the fields of the union is valid*/
    enum ReferenceType {
        SteeringAngleReference = 0,
        RawVoltageReference = 1
    } type;

    union {
        /**
         * @brief The steering angle of a vehicle in radians.
         *
         * The steering angle is measured in radians is defined as a positive
         * rotation around the yaw axis of the vehicle. This means that
         *
         * - angle > 0: car is steering to the left when driving forward
         * - angle < 0: car is steering to the right when driving forward
         */
        float steer_angle;      ///< [rad/s] angle reference
        uint16_t steer_voltage; ///< [mV] potentiometer reference
    };
};

/**
 * @brief A struct representing a single reference that a combination of
 * low-level controllers should track.
 */
struct Reference {
    /**
     * @brief The normalized torque in the range [-1, 1] applied on the throttle
     *
     * The throttle torque is normalized to the range [-1, 1], because the PWM
     * signal used to drive the actuators through the actuator driver chips can
     * linearly vary the voltage applied to the motors, which corresponds to the
     * torque. The normalization to the range [-1, 1] denotes a percentage of
     * the maximum torque, which is applied.
     *
     * The negative range is reserved for reversing the car, while the positive
     * range is used to propel the car forward.
     */
    float throttle;

    struct SteeringReference steer_ref;
};

/* Public function declaration ---------------------------------------------- */

/**
 * @brief Entry function of the control loop task.
 *
 * Commands the actuator setup and starts listening for control commands from
 * the host computer.
 * @note Never call directly, only invoke using xTaskCreate().
 * @param pvParameters Argument to be passed on task instantiation. Unused.
 */
void control_loop_task(void *pvParameters);

/**
 * @brief Sets a new reference to track.
 * @param new_ref Pointer to a non-null reference struct.
 * @param ticks_to_wait Maximum number of ticks to wait until timeout.
 * @returns true if the reference could be updated, false on error or timeout
 */
bool control_update_reference(struct Reference *new_ref,
                              TickType_t ticks_to_wait);

/**
 * @brief Start iterative execution of the control loop.
 *
 * The control loop can be started using control_loop_start() and stopped using
 * control_loop_stop(). It can be paused and resumed as often as required.
 */
void control_loop_start(void);

/**
 * @brief Stop iterative execution of the control loop.
 *
 * The control loop can be started using control_loop_start() and stopped using
 * control_loop_stop(). It can be paused and resumed as often as required.
 */
void control_loop_stop(void);

// The following two function definitions are not implemented in control-loop.c,
// but are free to be implemented in a user-provided file. Multiple
// implementations can be written, but only one can be linked against the
// control-loop component at any given time.

/**
 * @brief Iterate the steering controller.
 * @param r_steer The steering reference to follow
 * @param y_steer The current steering angle in radians
 * @param out Struct to write the next steering input to
 */
void control_steer_step(float r_steer, float y_steer, float *u_steer);

/**
 * @brief Iterate the driving controller.
 * @param r_drive The driving reference to follow
 * @param out Struct to write the next driving input to
 */
void control_drive_step(float r_drive, float *u_throttle);

/** @} */
