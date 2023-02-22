/*******************************************************************************
 * @file    drive-controller.c
 * @brief   Standard driving controllers that apply the torque from CRS directly
 ******************************************************************************/

#include "control.h"
#include "sdkconfig.h"

#if CONFIG_CONTROL_DRIVE_UNITY_GAIN

void control_drive_step(float r_drive, float *u_throttle) {
    *u_throttle = r_drive;
}

#endif

#if CONFIG_CONTROL_DRIVE_INCREASED_GAIN

void control_drive_step(float r_drive, float *u_throttle) {
    *u_throttle = 3 * r_drive;
}

#endif
