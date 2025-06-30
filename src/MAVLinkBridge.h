#ifndef MAVLINKBRIDGE_H
#define MAVLINKBRIDGE_H

#pragma once

#include "driver/uart.h"
#include "esp_timer.h"
#include "FlightController.h"
#include "mavlink/common/mavlink.h"
#include <string>

#define WIFI_SSID "ESP32-MAV"
#define WIFI_PASS "12345678"

class MAVLinkBridge{
public:
    FlightController& fc;
    int sock;
    sockaddr_in dest_addr;
    MAVLinkBridge(FlightController& fc) : fc(fc) {
        fc.init();
        nvs_flash_init();
        // setting up wifi point ...
        esp_netif_init();
        esp_event_loop_create_default();
        esp_netif_create_default_wifi_ap();
        
        wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
        esp_wifi_init(&cfg);
        
        wifi_config_t wifi_config = {};
        strcpy((char*)wifi_config.ap.ssid, WIFI_SSID);
        wifi_config.ap.ssid_len = strlen(WIFI_SSID);
        strcpy((char*)wifi_config.ap.password, WIFI_PASS);
        wifi_config.ap.max_connection = 1;
        wifi_config.ap.authmode = WIFI_AUTH_WPA_WPA2_PSK;

        esp_wifi_set_mode(WIFI_MODE_AP);
        esp_wifi_set_config(WIFI_IF_AP, &wifi_config);
        esp_wifi_start();
        xTaskCreate(&MAVLinkBridge::udp_mavlink_server_task, "mavlink_recv", 4096, this, 5, NULL);
        xTaskCreate(&MAVLinkBridge::send_heartbeat_task_wrapper, "heartbeat_task", 4096, this, 5, NULL);
        // socket initialization ...
        sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
        dest_addr.sin_family = AF_INET;
        dest_addr.sin_port = htons(14550);
        dest_addr.sin_addr.s_addr = inet_addr("192.168.4.2");
    }

    static void udp_mavlink_server_task(void* param) {
        static_cast<MAVLinkBridge*>(param)->udp_mavlink_server();
    }

    static void send_heartbeat_task_wrapper(void* param) {
        static_cast<MAVLinkBridge*>(param)->send_heartbeat_task();
    }

    void send_heartbeat_task(){
        mavlink_message_t msg;
        uint8_t buf[300];

        while (true) {
            mavlink_msg_heartbeat_pack(
                1,
                1,           // component_id (меняем на 1 — autopilot)
                &msg,
                MAV_TYPE_QUADROTOR,
                MAV_AUTOPILOT_GENERIC,
                MAV_MODE_MANUAL_ARMED,
                0,
                MAV_STATE_ACTIVE
            );
            uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
            sendto(sock, buf, len, 0, (struct sockaddr*)&dest_addr, sizeof(dest_addr));
            printf("Sent heartbeat\n");
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    void udp_mavlink_server(){
        int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);

        sockaddr_in addr;
        addr.sin_addr.s_addr = htonl(INADDR_ANY);
        addr.sin_family = AF_INET;
        addr.sin_port = htons(14550);

        bind(sock, (struct sockaddr*)&addr, sizeof(addr));

        uint8_t rx_buf[128];
        while (1) {
            sockaddr_in source_addr;
            socklen_t socklen = sizeof(source_addr);
            int len = recvfrom(sock, rx_buf, sizeof(rx_buf) - 1, 0,
                            (struct sockaddr *)&source_addr, &socklen);

            if (len > 0) {
                printf("Message recieved from qmavlink...\n");
                // Тут ты можешь парсить MAVLink, например:
                // mavlink_message_t msg;
                // mavlink_parse_char(MAVLINK_COMM_0, rx_buf[i], &msg, &status);
            }
        }
    }

    void send_attitude() {
        float roll, pitch, yaw;
        std::tie(roll, pitch, yaw) = fc.getEuler();  // В радианах

        mavlink_attitude_t att_msg = {
            .time_boot_ms = static_cast<uint32_t>(esp_timer_get_time() / 1000),
            .roll = roll * static_cast<float>(M_PI) / 180.0f,
            .pitch = pitch * static_cast<float>(M_PI) / 180.0f,
            .yaw = yaw * static_cast<float>(M_PI) / 180.0f,
            .rollspeed = 0.0f,
            .pitchspeed = 0.0f,
            .yawspeed = 0.0f
        };

        mavlink_message_t msg;
        uint8_t buf[MAVLINK_MAX_PACKET_LEN];
        mavlink_msg_attitude_encode(1, 1, &msg, &att_msg);
        uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

        sendto(sock, buf, len, 0, (struct sockaddr*)&dest_addr, sizeof(dest_addr));
        printf("Sent attitude: roll=%.2f pitch=%.2f yaw=%.2f\n", roll, pitch, yaw);
    }
};

#endif