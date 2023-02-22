/*******************************************************************************
 * @file    udp-client.c
 * @brief   Implementation of the UDP client task.
 ******************************************************************************/

#include "udp-client.h"

#include <lwip/netdb.h>
#include <stdbool.h>
#include <string.h>
#include <sys/param.h>

#include "esp_err.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/ringbuf.h"
#include "freertos/task.h"
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "nvs-manager.h"

/* Local macros ------------------------------------------------------------- */

#define HOST_IP_ADDR (global_config.host_ip)
#define HOST_PORT (global_config.host_port)

// Maximum UDP packet size. 508B + 60B IP header + 8B UDP header = 576B
// ("minimum maximum reassembly buffer size")
#define UDP_MAX_SIZE 508

/**
 * Frequency with which the UDP socket is polled for new updates. Should be
 * larger than the tick rate (1 kHz). Configure in menuconfig.
 * @note This frequency can probably only be met if the log level is lower than
 *  debug because the logging will limit throughput.
 */
#define UDP_POLL_FREQUENCY CONFIG_UDP_POLL_FREQUENCY

/** Bit indicating that the polling timer fired and the UDP client should poll
        the queue and the socket. */
#define UDP_POLL_TIMER_FIRED (1 << 1)

/* Local variables ---------------------------------------------------------- */

/** Logging tag. */
static const char *TAG = "udp-client";
/**  Callback that can be set via @link{udp_client_register_callback} */
static udp_reception_callback callback = NULL;
/** Ring buffer used to store data to be sent out */
static RingbufHandle_t ring_buffer_out = NULL;
/** Continuous timer providing high-frequency interrupts to start UDP poll */
static esp_timer_handle_t poll_timer = NULL;
/** Task handle that is used by the high-res timer to send notifications. */
static TaskHandle_t udp_task_handle = NULL;

/* Private function declaration --------------------------------------------- */

/**
 * @brief Sets up the high resolution timer used by the UDP task.
 *
 * The UDP task will periodically wake up and check if new data has arrived or
 * should be sent out. This wake-up frequency should be much larger than the
 * desired transmission rate, f_{wakeup} >> f_{transmission}.
 * Therefore, a high-resolution timer that can activate the UDP client at a rate
 * higher than the RTOS tick rate is needed. Configure the update frequency
 * using @link{UDP_POLL_FREQUENCY}.
 */
static bool highres_timer_setup(void);

/**
 * @brief Starts the timer.
 * @returns true if the timer could be started, false otherwise.
 */
static bool highres_timer_start(void);

/**
 * @brief Stops the timer.
 * @return true if the timer could be stopped, false if an error occurred.
 */
static bool highres_timer_stop(void);

/**
 * @brief Callback function that is invoked when the high res timer fires.
 * Unlocks the UDP client task via a direct to task notification.
 * @param arg user-defined argument, not used
 */
static void highres_timer_callback(void *arg);

/* Public function implementation ------------------------------------------- */

void udp_client_task(void *pvParameters) {

    ESP_LOGI(TAG, "Entered UDP Client Task!");

    // Preparation: create ring buffer that can hold up to 2kB of packets and
    // headers. Don't split so that on item acquiring it can directly be sent to
    // the socket
    ring_buffer_out = xRingbufferCreate(2048, RINGBUF_TYPE_NOSPLIT);
    uint8_t rx_buffer[509];

    udp_task_handle = xTaskGetCurrentTaskHandle();

    if (!highres_timer_setup()) {
        ESP_LOGE(TAG, "Failed to start high-res timer, exiting UDP task");
        vTaskDelete(NULL);
    }

    // Outer loop: when the connection breaks, keep trying to reconnect.
    while (true) {

        // Open UDP socket (use SOCK_STREAM for TCP experimentation)
        int sock = socket(PF_INET, SOCK_DGRAM, 0);

        if (sock < 0) {
            // If a socket can't be opened, the task might as well delete itself
            ESP_LOGE(TAG, "Unable to create socket: errno is %d", errno);
            break;
        }

        // Socket address information is filled in outer loop, does not need to
        // be set on each loop since only one host is connected to
        struct sockaddr_in si;
        memset(&si, 0, sizeof(si));
        si.sin_family = AF_INET;
        si.sin_addr.s_addr = inet_addr(HOST_IP_ADDR);
        si.sin_port = htons(HOST_PORT);

        // Make socket reusable, so that if the connection breaks, it can be
        // re-opened immediately.
        int reuse = 1;
        if (setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, (char *)&reuse,
                       sizeof(int)) < 0) {
            ESP_LOGW(TAG, "Could not set socket re-use option!");
        }

        // Set the priority for all packets that are sent to maximum priority
        // see:
        // https://docs.espressif.com/projects/esp-idf/en/latest/esp32/
        // api-guides/wifi.html#esp32-wi-fi-throughput
        const int ip_precedence_vi = 7;
        const int ip_precedence_offset = 5;
        int priority = (ip_precedence_vi << ip_precedence_offset);
        if (setsockopt(sock, IPPROTO_IP, IP_TOS, &priority, sizeof(priority)) <
            0) {
            ESP_LOGW(TAG, "Could not set socket priority option");
        }

        // Mark the socket as non-blocking, so it can be polled
        int flags = fcntl(sock, F_GETFL);
        if (fcntl(sock, F_SETFL, flags | O_NONBLOCK) == -1) {
            ESP_LOGW(TAG, "Unable to set socket non blocking");
        }

        ESP_LOGI(TAG, "Socket created, try to connect to %s:%d", HOST_IP_ADDR,
                 HOST_PORT);

        if (connect(sock, (struct sockaddr *)&si, sizeof(si)) < 0) {
            ESP_LOGE(TAG, "Can't connect to socket: errno is %d (s)", errno);
            // The first send/recv call will fail, and this will then jump to
            // re-initialize the socket. No action/jump required here.
        }

        // Now to actually send and receive data!
        size_t send_sz = 0; ///< Number of bytes to send next
        void *send_buf;     ///< Buffer to send next

        fd_set rx_set;

        highres_timer_start();

        while (true) {

            uint32_t bits = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

            // Task becomes ready at any notification bit, make sure it's the
            // timer-triggered one
            if ((bits & UDP_POLL_TIMER_FIRED) == 0)
                continue;

            // select() will modify the fd_set and the timeout struct, so set
            // each time in this loop
            FD_ZERO(&rx_set);
            FD_SET(sock, &rx_set);

            // First check if data is available to read, then send out if buffer
            // is non-empty
            // TODO: while the sending phase is repeated if multiple items are
            // available, the receiving phase only receives one packet per
            // iteration
            int recv_len = recv(sock, rx_buffer, sizeof(rx_buffer) - 1, 0);
            if (recv_len < 0) {
                if (errno == EAGAIN || errno == EWOULDBLOCK ||
                    errno == EINPROGRESS) {
                    // This fine — if the socket were blocking, it would wait
                    // here since no data is available.
                    ;
                } else {
                    // connection broke, socket closed or other error
                    ESP_LOGE(TAG, "Receival failed. Errno: %u", errno);
                    break;
                }
            } else if (recv_len == 0) {
                // socket may have been closed
                ESP_LOGE(TAG, "recv_len == 0, socket closed? errno %u", errno);
                break;
            } else {
                // we got data! null-terminate and hand over
                rx_buffer[recv_len] = '\0';
                if (callback != NULL) {
                    (*callback)(recv_len, rx_buffer);
                }
            }

            // Send phase is entered as soon as data is available in the ring
            // buffer.Skip if there is none.
            while ((send_buf = xRingbufferReceive(ring_buffer_out, &send_sz,
                                                  0)) != NULL) {

                // new item in queue, now send_buf points into the ring
                // buffer's memory, and we can directly send the data out
                if (send_sz <= UDP_MAX_SIZE) {
                    int sent = send(sock, send_buf, send_sz, 0);

                    if (sent < 0) {
                        ESP_LOGE(TAG,
                                 "Error occurred during sending:"
                                 "errno is %d",
                                 errno);
                        vRingbufferReturnItem(ring_buffer_out, send_buf);
                        break;
                    } else if (sent < send_sz) {
                        // Since the UDP implementation should be able to send
                        // 508B of payload, this "shouldn't" happen.
                        ESP_LOGW(TAG, "Not all bytes sent!");
                    } else {
                        ESP_LOGV(TAG, "Sent %dB", send_sz);
                    }

                } else {
                    ESP_LOGE(TAG, "UDP packet exceeded maximum allowed "
                                  "size. Dropping.");
                }

                vRingbufferReturnItem(ring_buffer_out, send_buf);
            }
        }

        highres_timer_stop();

        if (sock != -1) {
            // We arrive here, because something used break
            // close socket and re-attempt connection
            shutdown(sock, 0);
            close(sock);
        }

        vTaskDelay(100 / portTICK_PERIOD_MS);
    }

    // Goodbye Cruel World
    vTaskDelete(NULL);
}

void udp_client_register_callback(udp_reception_callback cb) { callback = cb; }

bool udp_client_send_packet(uint16_t len, const uint8_t *data) {

    if (ring_buffer_out) {
        if (len > 0 && len <= UDP_MAX_SIZE && data != NULL) {

            BaseType_t ret =
                xRingbufferSend(ring_buffer_out, // buffer to send to
                                (void *)data,    // data (will be copied)
                                len,             // size of data
                                0 // non-blocking, so return if full
                );

            if (ret == pdFALSE) {
                ESP_LOGE(TAG, "Could not queue data to send (buffer full!)");
            }

            // convert to bool
            return ret == pdTRUE;

        } else
            return false;
    } else {
        ESP_LOGE(TAG, "Attempted to send item via UDP before ring"
                      " buffer was created!");
        return false;
    }
}

/* Private function implementation ------------------------------------------ */

static bool highres_timer_setup(void) {
    esp_timer_create_args_t timer_cfg = {
        .callback = &highres_timer_callback,
        .dispatch_method =
            ESP_TIMER_TASK, // ESP-IDF v4.3 doesn't support ISR cb
        .name = "udp-poll-timer",
    };

    esp_err_t ret = esp_timer_create(&timer_cfg, &poll_timer);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create high-res timer. Reason: %s",
                 esp_err_to_name(ret));
        return false;
    }
    return true;
}

static bool highres_timer_start(void) {
    esp_err_t ret = esp_timer_start_periodic(
        poll_timer, (uint64_t)1000000 / UDP_POLL_FREQUENCY);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start high-res timer. Reason: %s",
                 esp_err_to_name(ret));
        return false;
    }
    return true;
}

static bool highres_timer_stop(void) {
    if (esp_timer_is_active(poll_timer)) {
        return esp_timer_stop(poll_timer) == ESP_OK;
    }
    return false;
}

static void highres_timer_callback(void *arg) {
    xTaskNotify(udp_task_handle, UDP_POLL_TIMER_FIRED, eSetBits);
}
