/*******************************************************************************
 * @file    nvs-manager.c
 * @brief   Initializes and manages the non-volatile storage (NVS).
 ******************************************************************************/

#include "nvs-manager.h"

#include <string.h>

#include "nvs_flash.h"
#include "sdkconfig.h"

/* Global variable declaration ---------------------------------------------- */

struct Configuration global_config = {0};

/* Local variables ---------------------------------------------------------- */

/** Handle to the storage where the vehicle config is stored. */
nvs_handle_t vehicle_config_handle = 0;

/* Public function implementation ------------------------------------------- */

void nvs_init(void) {

    // Call ESP-IDF initialization. If it fails, need to erase flash and
    // re-init.
    esp_err_t err = nvs_flash_init();

    if (err == ESP_ERR_NVS_NO_FREE_PAGES ||
        err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // NVS partition was truncated and needs to be erased. If it fails,
        // abort() the boot, because it is a more serious issue.
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }

    // Check again if it worked. If it didn't, abort.
    ESP_ERROR_CHECK(err);

    // Open the container used to store the configuration
    err = nvs_open("vehicle-storage", NVS_READWRITE, &vehicle_config_handle);
    ESP_ERROR_CHECK(err);

    return;
}

void nvs_load_config(void) {
    size_t ip_len = sizeof(global_config.host_ip);
    esp_err_t ret = nvs_get_str(vehicle_config_handle, "ip",
                                global_config.host_ip, &ip_len);

    switch (ret) {
    case ESP_OK:
        break;
    case ESP_ERR_NVS_NOT_FOUND:
        strncpy(global_config.host_ip, CONFIG_UDP_HOST_IP,
                sizeof(global_config.host_ip));
        // Zero-terminate in case the value in the config would be an overflow.
        // What will happen is that it just doesn't work, but instead can still
        // be set via the web interface.
        global_config.host_ip[sizeof(global_config.host_ip) - 1] = '\0';
        break;
    default:
        ESP_ERROR_CHECK(ret);
    }

    size_t ssid_len = sizeof(global_config.ssid);
    ret = nvs_get_str(vehicle_config_handle, "ssid", global_config.ssid,
                      &ssid_len);

    switch (ret) {
    case ESP_OK:
        break;
    case ESP_ERR_NVS_NOT_FOUND:
        strncpy(global_config.ssid, CONFIG_ESP_WIFI_SSID,
                sizeof(global_config.ssid));
        global_config.ssid[sizeof(global_config.ssid) - 1] = '\0';
        break;
    default:
        ESP_ERROR_CHECK(ret);
    }

    size_t pwd_len = sizeof(global_config.pwd);
    ret =
        nvs_get_str(vehicle_config_handle, "pwd", global_config.pwd, &pwd_len);

    switch (ret) {
    case ESP_OK:
        break;
    case ESP_ERR_NVS_NOT_FOUND:
        strncpy(global_config.pwd, CONFIG_ESP_WIFI_PASSWORD,
                sizeof(global_config.pwd));
        global_config.pwd[sizeof(global_config.pwd) - 1] = '\0';
        break;
    default:
        ESP_ERROR_CHECK(ret);
    }

    ret = nvs_get_u16(vehicle_config_handle, "port", &global_config.host_port);
    if (ret == ESP_ERR_NVS_NOT_FOUND) {
        global_config.host_port = CONFIG_UDP_HOST_PORT;
    }

    ret = nvs_get_u16(vehicle_config_handle, "adcl",
                      &global_config.steer_limit_lower);
    if (ret == ESP_ERR_NVS_NOT_FOUND) {
        global_config.steer_limit_lower = CONFIG_STEER_ADC_LOWER_LIMIT;
    }
    ret = nvs_get_u16(vehicle_config_handle, "adch",
                      &global_config.steer_limit_upper);
    if (ret == ESP_ERR_NVS_NOT_FOUND) {
        global_config.steer_limit_upper = CONFIG_STEER_ADC_UPPER_LIMIT;
    }

    ret = nvs_get_u8(vehicle_config_handle, "stang",
                     &global_config.steer_angle_deg);
    if (ret == ESP_ERR_NVS_NOT_FOUND) {
        global_config.steer_angle_deg = CONFIG_STEER_RANGE_DEG;
    }

    ret = nvs_get_u32(vehicle_config_handle, "pidp",
                      (uint32_t *)&global_config.pid_kp);
    if (ret == ESP_ERR_NVS_NOT_FOUND) {
        global_config.pid_kp = CONFIG_STEER_DEFAULT_KP / 1000.0f;
    }

    ret = nvs_get_u32(vehicle_config_handle, "pidi",
                      (uint32_t *)&global_config.pid_Ti);
    if (ret == ESP_ERR_NVS_NOT_FOUND) {
        global_config.pid_Ti = CONFIG_STEER_DEFAULT_TI / 1000.0f;
    }

    ret = nvs_get_u32(vehicle_config_handle, "pidd",
                      (uint32_t *)&global_config.pid_Td);
    if (ret == ESP_ERR_NVS_NOT_FOUND) {
        global_config.pid_Td = CONFIG_STEER_DEFAULT_TD / 1000.0f;
    }

    ret = nvs_get_u8(vehicle_config_handle, "pidn", &global_config.pid_N);
    if (ret == ESP_ERR_NVS_NOT_FOUND) {
        global_config.pid_N = CONFIG_STEER_DEFAULT_N;
    }

    ret = nvs_get_u32(vehicle_config_handle, "bbdb",
                      (uint32_t *)&global_config.bb_deadband);
    if (ret == ESP_ERR_NVS_NOT_FOUND) {
        global_config.bb_deadband = CONFIG_STEER_DEFAULT_DEADBAND;
    }

    ret = nvs_get_u8(vehicle_config_handle, "bbtq", &global_config.bb_torque);
    if (ret == ESP_ERR_NVS_NOT_FOUND) {
        global_config.bb_torque = CONFIG_STEER_DEFAULT_TORQUE;
    }

    uint8_t controller;
    ret = nvs_get_u8(vehicle_config_handle, "ctrl", &controller);
    if (ret == ESP_ERR_NVS_NOT_FOUND) {

        // Fall back to default controller
#if CONFIG_CONTROL_STEER_BANGBANG == 1
        controller = 1;
#elif CONFIG_CONTROL_STEER_PID == 1
        controller = 2;
#else
        controller = 3;
#endif
    }

    if (controller == 2)
        global_config.pid_enabled = true;
    else {
        global_config.pid_enabled = false;
    }
}

void nvs_store_config() {
    ESP_ERROR_CHECK(
        nvs_set_str(vehicle_config_handle, "ip", global_config.host_ip));
    ESP_ERROR_CHECK(
        nvs_set_str(vehicle_config_handle, "ssid", global_config.ssid));
    ESP_ERROR_CHECK(
        nvs_set_str(vehicle_config_handle, "pwd", global_config.pwd));
    ESP_ERROR_CHECK(
        nvs_set_u16(vehicle_config_handle, "port", global_config.host_port));
    ESP_ERROR_CHECK(nvs_set_u16(vehicle_config_handle, "adcl",
                                global_config.steer_limit_lower));
    ESP_ERROR_CHECK(nvs_set_u16(vehicle_config_handle, "adch",
                                global_config.steer_limit_upper));
    ESP_ERROR_CHECK(nvs_set_u8(vehicle_config_handle, "stang",
                               global_config.steer_angle_deg));

    uint32_t pidp;
    uint32_t pidi;
    uint32_t pidd;
    uint8_t pidn;
    uint32_t bbdb;
    uint8_t bbtq;

    memcpy(&pidp, &global_config.pid_kp, 4);
    memcpy(&pidi, &global_config.pid_Ti, 4);
    memcpy(&pidd, &global_config.pid_Td, 4);
    memcpy(&pidn, &global_config.pid_N, 1);
    memcpy(&bbdb, &global_config.bb_deadband, 4);
    memcpy(&bbtq, &global_config.bb_torque, 1);

    uint8_t ctrl = (global_config.pid_enabled) ? 2 : 0;

    ESP_ERROR_CHECK(nvs_set_u32(vehicle_config_handle, "pidp", pidp));
    ESP_ERROR_CHECK(nvs_set_u32(vehicle_config_handle, "pidi", pidi));
    ESP_ERROR_CHECK(nvs_set_u32(vehicle_config_handle, "pidd", pidd));
    ESP_ERROR_CHECK(nvs_set_u8(vehicle_config_handle, "pidn", pidn));
    ESP_ERROR_CHECK(nvs_set_u32(vehicle_config_handle, "bbdb", bbdb));
    ESP_ERROR_CHECK(nvs_set_u8(vehicle_config_handle, "bbtq", bbtq));
    ESP_ERROR_CHECK(nvs_set_u8(vehicle_config_handle, "ctrl", ctrl));

    ESP_ERROR_CHECK(nvs_commit(vehicle_config_handle));
}

/* Compile-time asserts ----------------------------------------------------- */

// These asserts are needed because floats are stored as uint32_t's in NVS as
// they're assumed to be the same size. If this assumption is broken, stop
// compilation.
_Static_assert(sizeof(float) == sizeof(uint32_t), "float is not four bytes");

// Assert that the
_Static_assert(CONFIG_STEER_DEFAULT_N != 0,
               "PID derivative filter divisor N can not be zero!");
