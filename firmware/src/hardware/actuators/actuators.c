/*******************************************************************************
 * @file    actuators.c
 * @brief   Bridge between the DC motor drivers and the control loop.
 ******************************************************************************/

#include "actuators.h"

#include "dc-motor.h"
#include "driver/gpio.h"
#include "driver/mcpwm.h"
#include "esp_log.h"
#include "math-util.h"

/* Local macros ------------------------------------------------------------- */

/**
 * @brief Enable / fault pin of the steering BLDC.
 *
 * Must be high for the motor to turn. Can be pulled low to disable the motor,
 * or will be pulled low in case of overcurrent.
 */
#define STEER_EN_PIN 25

/**
 * @brief PWM pin of the steering motor.
 *
 * The STSPIN250 motor driver, which supplies current to the BLDC, measures the
 * duty cycle and supplies power proportional to the duty cycle of the PWM on
 * this pin. If duty cycle = 100%, then it uses max power, if 0% it doesn't
 * power the motor.
 */
#define STEER_PWM_PIN 33

/** Steering phase pin. Sets the turning direction of the steering BLDC.
                            Low for turning right, high for turning left. */
#define STEER_PH_PIN 32
/** The pin that is connected to the steering feedback signal. */
#define STEERING_FB_PIN CONFIG_STEERING_FB_PIN

/**
 * @brief Enable / fault pin of the driving BLDC.
 *
 * Must be high for the motor to turn. Can be pulled low to disable the motor,
 * or will be pulled low in case of overcurrent.
 */
#define DRIVE_EN_PIN 16

/**
 * @brief PWM pin of the driving motor.
 *
 * The STSPIN250 motor driver, which supplies current to the BLDC, measures the
 * duty cycle and supplies power proportional to the duty cycle of the PWM on
 * this pin. If duty cycle = 100%, then it uses max power, if 0% it doesn't
 * power the motor.
 */
#define DRIVE_PWM_PIN 27

/** Driving phase pin. Sets the turning direction of the driving BLDC.
 *                                     High for forward, low for reverse. */
#define DRIVE_PH_PIN 26

/**
 * The actuators of the car are capable of running at a much higher torque than
 * is sustainable. We don't know what limitations the manufacturer set.
 * Conservatively, we only use 50% currently.
 */
#define MAX_TORQUE 100.0f

/**
 * The actuators of the car are capable of running at a much higher torque than
 * is sustainable. We don't know what limitations the manufacturer set.
 * Conservatively, we only use 50% currently.
 */
#define MAX_STEER_TORQUE 100.0f

/* Private function declaration --------------------------------------------- */

/**
 * @brief Callback that is invoked if the motors encounter a fault.
 * @param pwm_unit The PWM unit that encountered the fault.
 */
static void IRAM_ATTR actuator_fault_callback(mcpwm_unit_t pwm_unit);

/* Local variables ---------------------------------------------------------- */

/** Logging tag for the ESP_LOG* functions */
static const char *TAG = "actuators";

struct DCMotor drive_motor = {
    .pwm_pin = DRIVE_PWM_PIN,
    .phase_pin = DRIVE_PH_PIN,
    .en_fault_pin = DRIVE_EN_PIN,

    .mcpwm_unit = MCPWM_UNIT_0,
    .mcpwm_timer = MCPWM_TIMER_0,

    .pwm_freq = 1000,
    .auto_coasting = true,
    .fault_callback = &actuator_fault_callback,
};

struct DCMotor steer_motor = {
    .pwm_pin = STEER_PWM_PIN,
    .phase_pin = STEER_PH_PIN,
    .en_fault_pin = STEER_EN_PIN,

    .mcpwm_unit = MCPWM_UNIT_1,
    .mcpwm_timer = MCPWM_TIMER_1,

    .pwm_freq = 1000,
    .auto_coasting = false,
    .fault_callback = &actuator_fault_callback,
};

/* Public function implementation ------------------------------------------- */

void actuator_setup(void) {

    if (!dcmotor_configure(&drive_motor)) {
        ESP_LOGE(TAG, "Error during driving actuator configuration!");
    }

    if (!dcmotor_configure(&steer_motor)) {
        ESP_LOGE(TAG, "Error during steering actuator configuration!");
    }

    dcmotor_enable(&steer_motor);
}

void apply_steer(float steer_input) {
    steer_input = clampf(steer_input, -1.0f, 1.0f) * MAX_STEER_TORQUE;
    dcmotor_set_power(&steer_motor, steer_input);

    return;
}

void apply_torque(float throttle_input) {
    throttle_input = clampf(throttle_input, -1.0f, 1.0f) * MAX_TORQUE;
    dcmotor_set_power(&drive_motor, throttle_input);

    return;
}

/* Private function implementation ------------------------------------------ */

static void IRAM_ATTR actuator_fault_callback(mcpwm_unit_t pwm_unit) {
    // nothing happens currently
    return;
}
