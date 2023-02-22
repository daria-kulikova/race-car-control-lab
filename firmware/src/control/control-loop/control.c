/*******************************************************************************
 * @file    control.c
 * @brief   Low-level control loop task for steering and driving
 ******************************************************************************/

#include "control.h"

#include <math.h>
#include <stdbool.h>
#include <string.h>

#include "actuators.h"
#include "adc-steering-estimator.h"
#include "adc.h"
#include "bang-bang.h"
#include "communication-layer.h"
#include "control-types.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "math-util.h"
#include "nvs-manager.h"
#include "pid-steer.h"
#include "raw-steer-controller.h"
#include "sensors.h"
#include "state-machine.h"

/* Local macros ------------------------------------------------------------- */

/** The bit that is set in the control loop's task notification if the timer
 * fired and another iteration of the control loop should be executed. */
#define CONTROL_LOOP_TIMER_FIRED (1 << 9)

/** Frequency of control loop in Hertz */
#define CONTROL_LOOP_FREQUENCY (CONFIG_CONTROL_LOOP_RATE)

/** Steering range that the car's wheels can reach (one-sided). */
#define STEERING_RANGE (global_config.steer_angle_deg / 180.0f * M_PI)
/** Lowest voltage [mV] that is measured by the poti (right-most position) */
#define DISCRETE_LOWER_BOUND (global_config.steer_limit_lower)
/** Highest voltage [mV] that is measured by the poti (left-most position) */
#define DISCRETE_UPPER_BOUND (global_config.steer_limit_upper)

/* Local variable declaration ----------------------------------------------- */

//! Current reference to track
static struct Reference ref = {
    .throttle = 0.0f,
    .steer_ref =
        {
            .type = SteeringAngleReference,
            .steer_angle = 0.0f,
        },
};

//! Last time the reference got updated.
TickType_t last_update_tick = 0;

//! Tag used in logging.
static const char *TAG = "control-loop";

/* Local RTOS objects ------------------------------------------------------- */

// The following are handles for RTOS objects semaphores used by this task.
// Also, static buffers are used to instantiate these objects.

/** Task handle used to send notifications upon the timer firing. */
static TaskHandle_t control_loop_task_handle;

/** Semaphore that has to be taken before writing to @link{ref} */
static SemaphoreHandle_t ref_semphr = NULL;
static StaticSemaphore_t ref_semphr_buf;

/** Timer that fires each time the control loop should run. */
static TimerHandle_t ctrl_loop_timer_handle = NULL;
static StaticTimer_t ctrl_loop_timer_buf;

/* Private function declaration --------------------------------------------- */

/**
 * @brief Set up the RTOS functionality of the control loop task.
 *
 * The control loop uses a timer and a semaphore from FreeRTOS. This function
 * sets up the global variables @c ctrl_loop_timer_handle and @c ref_semphr.
 */
static void control_loop_rtos_setup(void);

/**
 * @brief Callback that is executed when the control loop timer fires.
 *
 * This function runs on the timer task's runtime and only notifies the control
 * loop task that it's time to execute another loop.
 * @param timer The handle of the timer that fired.
 */
static void control_loop_timer_fired(TimerHandle_t timer);

/**
 * @brief Log all measurements to the console (if verbose logging is enabled).
 * @param state The state containing the measurements
 */
static void log_measurements(const CarState *state);

/* Public function implementation ------------------------------------------- */

void control_loop_task(void *pvParameters) {

    control_loop_rtos_setup();

    Packet p = Packet_init_zero;
    p.which_contents = Packet_car_state_tag;
    CarState *state = &(p.contents.car_state);

    struct Reference current_ref = {
        .throttle = 0.0f,
        .steer_ref =
            {
                .type = SteeringAngleReference,
                .steer_angle = 0.0f,
            },
    };
    float steer_input = 0.0f;
    float throttle_input = 0.0f;
    InertialMeasurement imu_meas = {};
    struct LinearAcceleration linear_accel = {};
    struct AngularVelocity angular_vel = {};

    SteeringAngle steer_angle = 0.0f;

    while (true) {

        // Wait for the timer to fire, i.e. time to execute another loop.
        uint32_t not_val = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        // Other notifications could be sent to control loop task as well,
        // ignore them & wait again
        if ((not_val & CONTROL_LOOP_TIMER_FIRED) == 0)
            continue;

        // Grab current reference and store it locally, so that for this
        // controller iteration, there is no mismatch between global & local if
        // the reference is updated during loop execution (=> race condition)
        if (xSemaphoreTake(ref_semphr, pdMS_TO_TICKS(1)) == pdTRUE) {
            memcpy(&current_ref, &ref, sizeof(struct Reference));
            xSemaphoreGive(ref_semphr);
        } else {
            ESP_LOGE(TAG, "Control loop failed to obtain semaphore"
                          " after 1 tick (!)");
        }

        // Detect if reference has not been updated in a while - and tell the
        // FSM that it is time to go to power-save mode
        if (xTaskGetTickCount() - last_update_tick >
            pdMS_TO_TICKS(CONFIG_DISCONNECT_THRESHOLD_MS)) {
            current_ref.steer_ref.steer_angle = 0.0f;
            current_ref.steer_ref.type = SteeringAngleReference;
            current_ref.throttle = 0.0f;
            apply_steer(current_ref.steer_ref.steer_angle);
            apply_torque(current_ref.throttle);
            state_machine_input(PauseOperation);

            // Skip this control loop iteration, since it would override
            // stopping the motors
            continue;
        }

        // Constrain reference to the range the car can reach
        if (current_ref.steer_ref.type == SteeringAngleReference) {
            current_ref.steer_ref.steer_angle =
                clampf(current_ref.steer_ref.steer_angle, -STEERING_RANGE,
                       STEERING_RANGE);
        }

        // Sample sensors and current steering position
        ADCMeasurement discrete_steer_measurement = adc_sample();
        sensors_sample(&imu_meas);
        linear_accel = (struct LinearAcceleration)imu_meas.linear_accel;
        angular_vel = (struct AngularVelocity)imu_meas.angular_vel;

        steer_angle =
            estimate_steer(DISCRETE_UPPER_BOUND, DISCRETE_LOWER_BOUND,
                           STEERING_RANGE, discrete_steer_measurement);

        // Calculate next input to the actuators
        if (current_ref.steer_ref.type == SteeringAngleReference) {
            if (global_config.pid_enabled) {
                control_steer_step_pid(current_ref.steer_ref.steer_angle,
                                       steer_angle, &steer_input);
            } else {
                control_steer_step_bangbang(current_ref.steer_ref.steer_angle,
                                            steer_angle, &steer_input);
            }
            control_drive_step(current_ref.throttle, &throttle_input);
        } else {
            control_raw_steer_step(current_ref.steer_ref.steer_voltage,
                                   discrete_steer_measurement, &steer_input);
            control_drive_step(current_ref.throttle, &throttle_input);
        }

        // Update input PWM to motor drivers
        apply_steer(steer_input);
        apply_torque(throttle_input);

        // Send system state to host: sensor samples, inputs, etc.
        state->steer_data.adc_meas = discrete_steer_measurement;
        state->steer_data.steer_rad = steer_angle;

        state->steer_motor_input.power = steer_input;
        state->drive_motor_input.power = throttle_input;

        state->imu_data.linear_acceleration = (Vector3){
            linear_accel.x,
            linear_accel.y,
            linear_accel.z,
        };
        state->imu_data.angular_velocity = (Vector3){
            angular_vel.x,
            angular_vel.y,
            angular_vel.z,
        };
        state->current_reference.torque_ref = current_ref.throttle;
        if (current_ref.steer_ref.type == SteeringAngleReference) {
            state->current_reference.steer_input.which_input =
                SteerInput_steer_angle_tag;
            state->current_reference.steer_input.input.steer_angle =
                current_ref.steer_ref.steer_angle;
        } else {
            state->current_reference.steer_input.which_input =
                SteerInput_steer_voltage_tag;
            state->current_reference.steer_input.input.steer_voltage =
                current_ref.steer_ref.steer_voltage;
        }
        // for compatibility with old versions
        state->current_reference.steer_ref = current_ref.steer_ref.steer_angle;

        communication_layer_send(&p);
        log_measurements(state);
    }
}

bool control_update_reference(struct Reference *new_ref,
                              TickType_t ticks_to_wait) {
    if (!new_ref || !ref_semphr)
        return false;

    if (xTimerIsTimerActive(ctrl_loop_timer_handle) == pdFALSE) {
        state_machine_input(Operate);
    }

    if (xSemaphoreTake(ref_semphr, ticks_to_wait) == pdTRUE) {
        // got the semaphore. copy new reference and return it
        memcpy(&ref, new_ref, sizeof(struct Reference));
        last_update_tick = xTaskGetTickCount();
        xSemaphoreGive(ref_semphr);
        return true;
    } else {
        // probably timeout
        return false;
    }
}

void control_loop_start(void) {
    if (ctrl_loop_timer_handle) {
        xTimerStart(ctrl_loop_timer_handle, 0);
    }
}

void control_loop_stop(void) {
    if (ctrl_loop_timer_handle) {
        xTimerStop(ctrl_loop_timer_handle, 0);
    }
}

/* Private function implementation ------------------------------------------ */

static void control_loop_rtos_setup(void) {
    // Task handle is used by the timer callback function to notify control loop
    control_loop_task_handle = xTaskGetCurrentTaskHandle();

    // Instantiate semaphore to protect current control reference
    ref_semphr = xSemaphoreCreateMutexStatic(&ref_semphr_buf);
    configASSERT(ref_semphr);

    // Create software timer that can be started/stopped to control if the
    // control loop runs. Always succeeds because of static memory allocation.
    ctrl_loop_timer_handle = xTimerCreateStatic(
        "ctrl-timer",                                 // name
        pdMS_TO_TICKS(1000 / CONTROL_LOOP_FREQUENCY), // trigger frequency
        pdTRUE,                                       // repeating timer
        NULL,                                         // argument to callback
        &control_loop_timer_fired,                    // callback function
        &ctrl_loop_timer_buf                          // memory for timer
    );
    configASSERT(ctrl_loop_timer_handle);
}

static void control_loop_timer_fired(TimerHandle_t timer) {
    if (control_loop_task_handle) {
        xTaskNotify(control_loop_task_handle, CONTROL_LOOP_TIMER_FIRED,
                    eSetBits);
    }
}

static void log_measurements(const CarState *state) {
    ESP_LOGV(TAG, "w = %u", state->steer_data.adc_meas);
    ESP_LOGV(TAG, "ax, ay, az = %.2f, %.2f, %.2f",
             state->imu_data.linear_acceleration.x,
             state->imu_data.linear_acceleration.y,
             state->imu_data.linear_acceleration.z);
    ESP_LOGV(TAG, "wx, wy, wz = %.2f, %.2f, %.2f",
             state->imu_data.angular_velocity.x,
             state->imu_data.angular_velocity.y,
             state->imu_data.angular_velocity.z);
}
