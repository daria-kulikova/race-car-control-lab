/*******************************************************************************
 * @file    pid-steer.c
 * @brief   PID controller for the steering angle of the car.
 ******************************************************************************/

#include "pid-steer.h"

#include <float.h>

#include "control.h"
#include "nvs-manager.h"

/* Local macros ------------------------------------------------------------- */

void control_steer_step_pid(float r_steer, float y_steer, float *u_steer) {

    // Normalize input
    static const float theta_0 = 0.1f;

    // Memory variables
    static float e_1 = 0; ///< e[n - 1]
    static float i = 0;   ///< sum of e[k] where k = 0...n

    const float kp = global_config.pid_kp;

    const float Ti =
        global_config.pid_Ti != 0.0 ? global_config.pid_Ti : FLT_MAX;
    const float Td = global_config.pid_Td;
    // const float N = global_config.pid_N != 0 ? global_config.pid_N : 1;
    const float Ts = 0.004;

    const float r = r_steer / theta_0;
    const float y = y_steer / theta_0;
    const float e = r - y;

    // Propagate integral gain
    i += e * Ts;

    *u_steer = kp * (e + 1 / Ti * i + Td * (e - e_1));

    // Propagate error
    e_1 = e;

    return;
}
