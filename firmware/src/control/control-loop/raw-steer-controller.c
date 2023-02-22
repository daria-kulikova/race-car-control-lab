/*******************************************************************************
 * @file    raw-steer-controller.c
 * @brief   Steer controller that tracks the raw potentiometer measurement
 ******************************************************************************/

#include "raw-steer-controller.h"

#include "control.h"

void control_raw_steer_step(uint16_t ref, uint16_t y, float *u) {
    if (u != NULL) {
        // k_p = 1.0 / (1000 mV)
        *u = ((float)ref - (float)y) / 1000.0f;
    }
}
