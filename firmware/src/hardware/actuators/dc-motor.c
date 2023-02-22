/*******************************************************************************
 * @file    dc-motor.c
 * @brief   Drivers for a DC motor, based on the MCPWM drivers of ESP-IDF
 ******************************************************************************/

#include "dc-motor.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "driver/mcpwm.h"
#include "esp_log.h"

/* Local macros ------------------------------------------------------------- */

/** Timer unit used to set the duty cycle of the PWM pin. */
#define POWER_TIMER MCPWM_TIMER_0
/** Timer unit used for phase of motor. A PWM unit is abused for this.
 * (Constantly held low or high <=> 0% or 100% duty cycle. )*/
#define PHASE_TIMER MCPWM_TIMER_1
/** For this flag, see the gpio_example of ESP-IDF */
#define ESP_INTR_FLAG_DEFAULT 0

/** Tag used in logging. */
const char *TAG = "dc-motor";

/* Public function implementation ------------------------------------------- */

bool dcmotor_configure(struct DCMotor *motor) {
    if (!motor)
        return false;

    // Reject invalid configurations that exceed number of available MCPWM units
    if (motor->mcpwm_unit == MCPWM_UNIT_MAX ||
        motor->mcpwm_timer == MCPWM_TIMER_MAX)
        return false;

    // Spaghetti code here is a bit unavoidable, since one needs to know the
    // macros associated with each unit.
    mcpwm_io_signals_t pwm_signal_route;
    mcpwm_io_signals_t pha_signal_route;

    switch (motor->mcpwm_unit) {
    case MCPWM_UNIT_0:
        pwm_signal_route = MCPWM0A;
        pha_signal_route = MCPWM0B;
        break;
    case MCPWM_UNIT_1:
        pwm_signal_route = MCPWM1A;
        pha_signal_route = MCPWM1B;
        break;
    default:
        ESP_LOGE(TAG, "Tried to configure unsupported MCPWM unit. Only MCPWM 0 "
                      "and 1 supported currently!");
        return false;
    }

    // Configure the pins
    if (mcpwm_gpio_init(motor->mcpwm_unit, pwm_signal_route, motor->pwm_pin) !=
        ESP_OK)
        return false;

    if (mcpwm_gpio_init(motor->mcpwm_unit, pha_signal_route,
                        motor->phase_pin) != ESP_OK)
        return false;

    // Configure the PWM frequency and initial conditions
    mcpwm_config_t pwm_config = {
        .frequency = motor->pwm_freq,
        // both PWM channels default to zero duty cycle (phase 0, no power)
        .cmpr_a = 0.0,
        .cmpr_b = 0.0,
        .counter_mode = MCPWM_UP_COUNTER,
        .duty_mode = MCPWM_DUTY_MODE_0,
    };

    if (mcpwm_init(motor->mcpwm_unit, motor->mcpwm_timer, &pwm_config) !=
        ESP_OK)
        return false;

    // Configure EN/FAULT pin as open drain output
    gpio_config_t en_cfg = {
        .pin_bit_mask = (1ULL << motor->en_fault_pin),
        .mode = GPIO_MODE_INPUT_OUTPUT_OD,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE,
    };

    gpio_config(&en_cfg);

    // Pull down enable pins to disable motor initialls
    if (motor->fault_callback != NULL) {
        gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
        gpio_isr_handler_add(motor->en_fault_pin,
                             (void (*)(void *))(motor->fault_callback),
                             (void *)motor->mcpwm_unit);
    }

    return dcmotor_disable(motor);
}

bool dcmotor_enable(struct DCMotor *motor) {
    // To re-enable the motor, just let go of the pin and it will be pulled high
    // by the external pull-up resistor. Since it is open-drain, this lets go.
    gpio_set_level(motor->en_fault_pin, 1);
    ets_delay_us(1);
    gpio_set_intr_type(motor->en_fault_pin, GPIO_INTR_NEGEDGE);
    gpio_intr_enable(motor->en_fault_pin);

    motor->enabled = true;

    return true;
}

bool dcmotor_disable(struct DCMotor *motor) {
    // The EN/FAULT pin is only ever driven LOW, never high. If driving high and
    // the motor driver chip encounters a fault condition, you got yourself a
    // nice short.

#if CONFIG_DISABLE_DC_COASTING
#else
    motor->enabled = false;
    gpio_set_intr_type(motor->en_fault_pin, GPIO_INTR_DISABLE);
    gpio_set_level(motor->en_fault_pin, 0);
#endif

    return true;
}

bool dcmotor_set_power(struct DCMotor *motor, float power) {
    if (fabs(power) > 100.0f)
        return false;

    // the PWM signal for power is on output A of the MCPWM, phase is output B
    mcpwm_set_duty(motor->mcpwm_unit, motor->mcpwm_timer, MCPWM_GEN_A,
                   fabs(power));

    if (motor->reverse_phase)
        power = -power;

    if (power < 0.0f) {
        mcpwm_set_signal_high(motor->mcpwm_unit, motor->mcpwm_timer,
                              MCPWM_GEN_B);
    } else {
        mcpwm_set_signal_low(motor->mcpwm_unit, motor->mcpwm_timer,
                             MCPWM_GEN_B);
    }

    if (motor->auto_coasting) {
        if (fabs(power) <= 0.01 && motor->enabled)
            dcmotor_disable(motor);
        else if (fabs(power) > 0.01 && !motor->enabled)
            dcmotor_enable(motor);
    }

    return true;
}
