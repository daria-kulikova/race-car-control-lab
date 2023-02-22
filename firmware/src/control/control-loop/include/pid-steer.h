/*******************************************************************************
 * @file    pid-steer.h
 * @brief   PID controller for the steering angle of the car.
 ******************************************************************************/

#pragma once

void control_steer_step_pid(float r_steer, float y_steer, float *u_steer);
