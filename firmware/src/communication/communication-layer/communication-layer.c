/**
 *******************************************************************************
 * @file    communication-layer.c
 * @brief   Set of callbacks that mediate between the control loop and the
 *          network layer.
 *******************************************************************************
 */

#include "communication-layer.h"

#include <pb_decode.h>
#include <pb_encode.h>
#include <string.h>

#include "control.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
#include "udp-client.h"

#include "Packet.pb.h"

/* Private function declaration --------------------------------------------- */

/**
 * @brief Quickly and efficiently handle data received via UDP.
 *
 * This parses the incoming data into a packet struct and then decides which
 * action to take depending on the packet type.
 * @note This callback is called on Core 0, notabene on the runtime of the UDP
 *  client task, and has to be kept short.
 */
void handle_incoming_datagram(uint16_t len, const uint8_t *data);

/* Private variable declaration --------------------------------------------- */

/** Tag used in logging. */
static const char *TAG = "comm-layer";

/* Private function declaration --------------------------------------------- */

/**
 * @brief Sends a discovery packet out to the host upon a timer firing.
 *
 * This discovery packet is needed because the CRS host needs to first receive
 * a datagram from the car before sending its commands to the IP/port it
 * receives traffic from. If the discovery packet is missing, the embedded
 * system can't pair to the host.
 * @param t The timer that fired (will be 100% ignored)
 */
void send_discovery_packet(TimerHandle_t t);

/* Public function implementation ------------------------------------------- */

void communication_layer_setup() {
    // Register as callback for incoming UDP datagrams
    udp_client_register_callback(&handle_incoming_datagram);

    // Create and start timer to send out periodic discovery packets
    TimerHandle_t discovery_timer;
    if ((discovery_timer =
             xTimerCreate("discovery-timer", 5000 / portTICK_PERIOD_MS, pdTRUE,
                          NULL, &send_discovery_packet)) != NULL) {
        if (xTimerStart(discovery_timer, 0) == pdPASS) {
            ESP_LOGI(TAG, "Started discovery timer...");
            return;
        }
    }
    ESP_LOGE(TAG, "Failed to start discovery timer!");
}

/* Private function implementation ------------------------------------------ */

void communication_layer_send(const Packet *p) {
    if (!p)
        return;

    // Buffer where the serialized data goes
    uint8_t buffer[255];

    // Create a stream handle from the buffer, and encode the data into the
    // buffer
    pb_ostream_t ostream =
        pb_ostream_from_buffer((uint_least8_t *)&buffer, sizeof(buffer));
    bool status = pb_encode(&ostream, Packet_fields, p);

    if (!status) {
        // encoding successful
        ESP_LOGE(TAG, "Failed to encode packet. Reason: %s",
                 PB_GET_ERROR(&ostream));
        return;
    }

    udp_client_send_packet(ostream.bytes_written, buffer);
}

/* Private function implementation ------------------------------------------ */

void handle_incoming_datagram(uint16_t len, const uint8_t *data) {
    if (len < 2) {
        // insufficient length to gather type and payload size, reject
        ESP_LOGE(TAG, "Incoming datagram has insufficient length (<2).");
        return; // packet is dropped by UDP client
    }

    Packet p = Packet_init_zero;
    pb_istream_t istream = pb_istream_from_buffer(data, len);

    bool status = pb_decode(&istream, Packet_fields, &p);
    if (!status) {
        ESP_LOGE(TAG, "Failed to decode message. Reason: %s",
                 PB_GET_ERROR(&istream));
        return; // packet is dropped by UDP client
    }

    switch (p.which_contents) {
    case Packet_ping_tag:
        p.contents.ping.should_bounce = false;
        communication_layer_send(&p);
        break;
    case Packet_control_input_tag: {
        struct Reference new_ref = {0};
        new_ref.throttle = p.contents.control_input.torque_ref;

        if (p.contents.control_input.has_steer_input) {
            // Newer version of message format that uses the new oneof format
            if (p.contents.control_input.steer_input.which_input ==
                SteerInput_steer_angle_tag) {
                new_ref.steer_ref.type = SteeringAngleReference;
                new_ref.steer_ref.steer_angle =
                    p.contents.control_input.steer_input.input.steer_angle;
            } else {
                new_ref.steer_ref.type = RawVoltageReference;
                new_ref.steer_ref.steer_voltage =
                    p.contents.control_input.steer_input.input.steer_voltage;
            }
        } else {
            // Fallback to the old version
            new_ref.steer_ref.type = SteeringAngleReference;
            new_ref.steer_ref.steer_angle = p.contents.control_input.steer_ref;
        }

        control_update_reference(&new_ref, 1);
        break;
    }
    default:
        ESP_LOGW(TAG, "Received unknown packet type. Dropping...");
        break;
    }

    return;
}

void send_discovery_packet(TimerHandle_t t) {
    ESP_LOGI(TAG, "Sending a discovery beacon packet...");

    Packet p = Packet_init_zero;
    p.which_contents = Packet_ping_tag;
    p.contents.ping.should_bounce = false;

    communication_layer_send(&p);
}
