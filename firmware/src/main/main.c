/*******************************************************************************
 * @file    main.c
 * @brief   Main entry point for the CRS WiFi Car Firmware
 ******************************************************************************/

#include "main.h"

#include <stdio.h>

#include "actuators.h"
#include "adc.h"
#include "communication-layer.h"
#include "control.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs-manager.h"
#include "power-monitoring.h"
#include "sdkconfig.h"
#include "sensors.h"
#include "state-machine.h"
#include "udp-client.h"
#include "wifi.h"

/* Static stack allocation for tasks ---------------------------------------- */

// To reduce dynamic memory allocation, the task's stacks are allocated during
// linking. If they don't fit into memory, a build error instead of a runtime
// error will occur.

/** Task Control Block for UDP task. */
StaticTask_t udp_task_buffer;
/** Task Control Block for control loop task. */
StaticTask_t control_loop_task_buffer;
/** Task Control Block for power monitoring task. */
StaticTask_t power_monitoring_task_buffer;
/** Task Control Block for state machine task. */
StaticTask_t state_machine_task_buffer;

/* Stack of UDP task. */
StackType_t udp_stack_buffer[CONFIG_TASK_STACK_SIZE_UDP];
/* Stack of control loop task. */
StackType_t control_loop_stack_buffer[CONFIG_TASK_STACK_SIZE_CONTROL_LOOP];
/* Stack of Power Monitoring task. */
StackType_t
    power_monitoring_stack_buffer[CONFIG_TASK_STACK_SIZE_POWER_MONITORING];
/** Stack of Finite State Machine task. */
StackType_t state_machine_stack_buffer[CONFIG_TASK_STACK_SIZE_STATE_MACHINE];

/* Local variables ---------------------------------------------------------- */

/** Tag used in logging */
static const char *TAG = "main";
/** Handle to UDP client task that handles the UDP socket communication with the
 * CRS host. */
static TaskHandle_t udp_task_handle;
/** Handle to control loop task, which runs the control algorithm. */
static TaskHandle_t control_loop_task_handle;
/** Handle to power monitoring task, which measures battery voltage and current
 * consumption. */
static TaskHandle_t power_monitoring_task_handle;
/** Handle to the finite state machine task. */
static TaskHandle_t state_machine_task_handle;

/* Local macros ------------------------------------------------------------- */

/** Specifies that a task can run on core 0 only. "PRO CPU". */
#define CORE_0 0
/** Specifies that a task can run on core 1 only. "APP CPU". */
#define CORE_1 1
/** Specifies that a task can run on both cores. */
#define CORE_BOTH tskNO_AFFINITY

/* Public function implementation ------------------------------------------- */

void app_main(void) {

    // Initialize non-volatile storage. This enables reading the stored
    // settings, Wi-Fi calibration data etc.
    nvs_init();
    nvs_load_config();
    ESP_LOGI(TAG, "Config ip: '%s', port: %u, ssid: '%s', pwd: '%s'",
             global_config.host_ip, global_config.host_port, global_config.ssid,
             global_config.pwd);

    adc_setup();

    // First task to get instantiated is the finite state machine task.
    state_machine_task_handle = xTaskCreateStaticPinnedToCore(
        &state_machine_task,                  // entry function of the task
        "state machine",                      // task name
        CONFIG_TASK_STACK_SIZE_STATE_MACHINE, // stack size
        NULL,                                 // arguments to pass
        CONFIG_TASK_PRIORITY_STATE_MACHINE,   // priority
        state_machine_stack_buffer,           // static buffer used as stack
        &state_machine_task_buffer,           // static buffer for TCB
        CORE_BOTH // core the task is bound to (none)
    );

    // Next task to get instantiated is the power management because it should
    // also monitor the power-up phase
    power_monitoring_task_handle = xTaskCreateStaticPinnedToCore(
        &power_monitoring_task,                  // entry function of the task
        "power monitor",                         // task name
        CONFIG_TASK_STACK_SIZE_POWER_MONITORING, // stack size
        NULL,                                    // arguments to pass
        CONFIG_TASK_PRIORITY_POWER_MONITORING,   // priority
        power_monitoring_stack_buffer,           // static buffer used as stack
        &power_monitoring_task_buffer,           // static buffer for TCB
        CORE_BOTH // core the task is bound to (none)
    );

    // First, initialize all internal physical components, so that they are
    // ready when tasks launch.
    sensors_setup();
    actuator_setup();

    // Internal stuff is set up. Now, a Wi-Fi connection can be established,
    // communication with other hosts can be started.
    wifi_init();

    // Start tasks that deal with the Wi-Fi: UDP to receive data and the control
    // loop to run the car. Both depend on a working Wi-Fi connection.
    udp_task_handle = xTaskCreateStaticPinnedToCore(
        &udp_client_task,           // entry function of the task
        "udp client",               // task name
        CONFIG_TASK_STACK_SIZE_UDP, // stack size
        NULL,                       // arguments to pass
        CONFIG_TASK_PRIORITY_UDP,   // priority
        udp_stack_buffer,           // static buffer used as stack
        &udp_task_buffer,           // static buffer for TCB
        CORE_0                      // core the task is bound to
    );

    control_loop_task_handle = xTaskCreateStaticPinnedToCore(
        &control_loop_task,                  // entry function of the task
        "control loop",                      // task name
        CONFIG_TASK_STACK_SIZE_CONTROL_LOOP, // stack size
        NULL,                                // arguments to pass
        CONFIG_TASK_PRIORITY_CONTROL_LOOP,   // priority
        control_loop_stack_buffer,           // static buffer used as stack
        &control_loop_task_buffer,           // static buffer for TCB
        CORE_1                               // core the task is bound to
    );

    if (!wifi_connect_to_ap(pdMS_TO_TICKS(20000))) {
        // If the Wi-Fi connection fails, this usually means that the
        // credentials are bad. In that case, deviate from the launch and start
        // the configuration server so that new credentials can be entered.
        state_machine_input(StartupFailure);
    }

    if (udp_task_handle == NULL || control_loop_task_handle == NULL) {
        ESP_LOGE(TAG, "Failed launching tasks. Likely a configuration error!");
    }

    communication_layer_setup();

    // Set up an interrupt on pressing the BOOT button: if it is pressed, enter
    // the configuration mode
    // Configure EN/FAULT pin as open drain output
    gpio_config_t gpio0_cfg = {
        .pin_bit_mask = (1ULL << 0),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE,
    };
    gpio_config(&gpio0_cfg);
    gpio_install_isr_service(0);
    gpio_isr_handler_add((gpio_num_t)0, (void (*)(void *))(state_machine_input),
                         (void *)Configure);

    // As per the ESP-IDF documentation, it is allowed to return from app_main
    // once the setup work is done. The task's resources will get cleaned up.
    state_machine_input(StartupCompleted);
    ESP_LOGI(TAG, "Finished setup of system. Returning from main task.");
    return;
}
