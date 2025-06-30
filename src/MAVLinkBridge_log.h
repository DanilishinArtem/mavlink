#ifndef MAVLINKBRIDGELOG_H
#define MAVLINKBRIDGELOG_H

#pragma once

#include "driver/uart.h"
#include "esp_timer.h"
#include "FlightController.h"
#include "mavlink/common/mavlink.h"

#define UART_PORT UART_NUM_1
#define BUF_SIZE (1024)

class MAVLinkBridge {
public:
    FlightController& fc;

    MAVLinkBridge(FlightController& fc) : fc(fc) {
        uart_config_t uart_config = {
            .baud_rate = 57600,
            .data_bits = UART_DATA_8_BITS,
            .parity    = UART_PARITY_DISABLE,
            .stop_bits = UART_STOP_BITS_1,
            .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
        };
        uart_param_config(UART_PORT, &uart_config);
        uart_set_pin(UART_PORT, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
        uart_driver_install(UART_PORT, BUF_SIZE * 2, 0, 0, NULL, 0);
    }

    void heartbeat() {
        mavlink_heartbeat_t hb = {
            .type = MAV_TYPE_QUADROTOR,
            .autopilot = MAV_AUTOPILOT_GENERIC,
            .base_mode = MAV_MODE_MANUAL_ARMED,
            .system_status = MAV_STATE_ACTIVE,
            .mavlink_version = 3
        };

        mavlink_message_t msg;
        uint8_t buf[MAVLINK_MAX_PACKET_LEN];

        mavlink_msg_heartbeat_encode(1, 200, &msg, &hb);
        uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
        uart_write_bytes(UART_PORT, buf, len);
    }

    void send_attitude() {
        auto [roll, pitch, yaw] = fc.getEuler();
        uint32_t time = (uint32_t)(esp_timer_get_time() / 1000);
        float roll_ = roll * float(M_PI) / 180.0f;
        float pitch_ = pitch * float(M_PI) / 180.0f;
        float yaw_ = yaw * float(M_PI) / 180.0f;
        printf("roll: %.4f, pitch: %.4f, yaw: %.4f\n", roll_, pitch_, yaw_);

        mavlink_attitude_t att = {
            .time_boot_ms = (uint32_t)(esp_timer_get_time() / 1000),
            .roll = roll * float(M_PI) / 180.0f,
            .pitch = pitch * float(M_PI) / 180.0f,
            .yaw = yaw * float(M_PI) / 180.0f,
            .rollspeed = 0,
            .pitchspeed = 0,
            .yawspeed = 0
        };

        mavlink_message_t msg;
        uint8_t buf[MAVLINK_MAX_PACKET_LEN];
        mavlink_msg_attitude_encode(1, 200, &msg, &att);
        uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
        uart_write_bytes(UART_PORT, buf, len);
    }

    void receive() {
        uint8_t rx_buf[256];
        int len = uart_read_bytes(UART_PORT, rx_buf, sizeof(rx_buf), 10 / portTICK_PERIOD_MS);
        if (len <= 0) return;

        mavlink_message_t msg;
        mavlink_status_t status;

        for (int i = 0; i < len; ++i) {
            if (mavlink_parse_char(MAVLINK_COMM_0, rx_buf[i], &msg, &status)) {
                // Обработка входящих сообщений по необходимости
                switch (msg.msgid) {
                    case MAVLINK_MSG_ID_HEARTBEAT:
                        printf("💡 Received HEARTBEAT\n");
                        break;
                    default:
                        break;
                }
            }
        }
    }
};

#endif
