#ifndef MAVLINKBRIDGE_H
#define MAVLINKBRIDGE_H

#pragma once

#include <string>
#include <tuple>
#include <cstring>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>

#include "esp_timer.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "nvs_flash.h"

#include "driver/uart.h"
#include "FlightController.h"
#include "mavlink/common/mavlink.h"

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
        xTaskCreate(&MAVLinkBridge::send_sys_status_wrapper, "sys_status", 4096, this, 5, NULL);
        xTaskCreate(&MAVLinkBridge::rotter_update_loop_wrapper, "rotter_update_loop", 4096, this, 5, NULL);
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

    static void send_sys_status_wrapper(void* param) {
        static_cast<MAVLinkBridge*>(param)->send_sys_status();
    }

    static void rotter_update_loop_wrapper(void* param) {
        static_cast<MAVLinkBridge*>(param)->rotter_update_loop();
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
            // printf("Sent heartbeat\n");
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    void send_sys_status() {
        mavlink_message_t msg;
        uint8_t buf[MAVLINK_MAX_PACKET_LEN];
        while(true){
            mavlink_msg_sys_status_pack(
                1, 1, &msg,
                // onboard_control_sensors_present
                MAV_SYS_STATUS_SENSOR_3D_GYRO | MAV_SYS_STATUS_SENSOR_3D_ACCEL | MAV_SYS_STATUS_SENSOR_3D_MAG,
                // onboard_control_sensors_enabled
                MAV_SYS_STATUS_SENSOR_3D_GYRO | MAV_SYS_STATUS_SENSOR_3D_ACCEL | MAV_SYS_STATUS_SENSOR_3D_MAG,
                // onboard_control_sensors_health
                MAV_SYS_STATUS_SENSOR_3D_GYRO | MAV_SYS_STATUS_SENSOR_3D_ACCEL | MAV_SYS_STATUS_SENSOR_3D_MAG,
                // load (0–10000 = 0–100%)
                100,  
                // voltage_battery (in mV)
                12000,
                // current_battery (in cA)
                -1,
                // battery_remaining (in %, -1 = unknown)
                -1,
                // drop_rate_comm
                0,
                // errors_comm
                0,
                // errors_count1..4
                0, 0, 0, 0,
                // extended present, enabled, health (MAVLink 2 only)
                0, 0, 0
            );
            uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
            sendto(sock, buf, len, 0, (struct sockaddr*)&dest_addr, sizeof(dest_addr));
            printf("[HIMARK] system status sended...\n");
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    void udp_mavlink_server(){
        // int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);

        // sockaddr_in addr;
        // addr.sin_addr.s_addr = htonl(INADDR_ANY);
        // addr.sin_family = AF_INET;
        // addr.sin_port = htons(14550);

        // bind(sock, (struct sockaddr*)&addr, sizeof(addr));


        uint8_t rx_buf[128];
        mavlink_message_t msg;
        mavlink_status_t status;

        while (true) {
            sockaddr_in source_addr;
            socklen_t socklen = sizeof(source_addr);
            int len = recvfrom(sock, rx_buf, sizeof(rx_buf), 0,
                            (struct sockaddr *)&source_addr, &socklen);

            if (len > 0) {
                for (int i = 0; i < len; ++i) {
                    if (mavlink_parse_char(MAVLINK_COMM_0, rx_buf[i], &msg, &status)) {
                        handle_mavlink_message(msg);
                    }
                }
            }
        }
    }

    void rotter_update_loop() {
        // ждём инициализации ESC
        while (!fc.esc_ready) {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        // и запускаем PWM
        while (true) {
            fc.updateESC();
            vTaskDelay(pdMS_TO_TICKS(50));
        }
    }

    void handle_mavlink_message(const mavlink_message_t& msg) {
        switch (msg.msgid) {
            case MAVLINK_MSG_ID_MANUAL_CONTROL:
                handle_manual_control(msg);
                break;
            case MAVLINK_MSG_ID_COMMAND_LONG:
                printf("[HIMARK] COMMAND_LONG CASE...\n");
                handle_command_long(msg);
                break;
            case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
                printf("[HIMARK] PARAM_REQUEST_LIST CASE...\n");
                handle_param_request_list(msg);
                break;
            case MAVLINK_MSG_ID_MISSION_REQUEST_LIST:
                handle_mission_request_list(msg);
                printf("[HIMARK] MISSION_REQUEST_LIST CASE...\n");
                break;
            default:
                break;
        }
    }

    void handle_manual_control(const mavlink_message_t& msg) {
        mavlink_manual_control_t ctrl;
        mavlink_msg_manual_control_decode(&msg, &ctrl);
        RCInput rc;
        rc.roll = ctrl.x;
        rc.pitch = ctrl.y;
        rc.throttle = ctrl.z;
        rc.yaw = ctrl.r;
        rc.buttons = ctrl.buttons;
        printf("[INFO RC] roll: %d, pitch: %d, yaw: %d, throttle: %d\n", rc.roll, rc.pitch, rc.yaw, rc.throttle);
        fc.setRCInput(rc);
    }

    void handle_command_long(const mavlink_message_t& msg) {
        mavlink_command_long_t cmd;
        mavlink_msg_command_long_decode(&msg, &cmd);

        if (cmd.command == MAV_CMD_REQUEST_MESSAGE && static_cast<uint32_t>(cmd.param1) == MAVLINK_MSG_ID_AUTOPILOT_VERSION) {
            mavlink_message_t out_msg;
            mavlink_autopilot_version_t version = {};
            version.capabilities = MAV_PROTOCOL_CAPABILITY_MISSION_INT |
                                MAV_PROTOCOL_CAPABILITY_PARAM_FLOAT |
                                MAV_PROTOCOL_CAPABILITY_COMMAND_INT;
            version.flight_sw_version = 0x01010101;
            version.middleware_sw_version = 0x01010101;
            version.os_sw_version = 0x01010101;
            version.board_version = 0x00000001;
            version.vendor_id = 42;
            version.product_id = 0;
            version.uid = 12345678;

            mavlink_msg_autopilot_version_encode(1, 1, &out_msg, &version);
            uint8_t buf[MAVLINK_MAX_PACKET_LEN];
            uint16_t len = mavlink_msg_to_send_buffer(buf, &out_msg);
            sendto(sock, buf, len, 0, (struct sockaddr*)&dest_addr, sizeof(dest_addr));
            printf("Ответил на AUTOPILOT_VERSION\n");
        }
    }

    void handle_param_request_list(const mavlink_message_t& msg) {
        mavlink_param_request_list_t req;
        mavlink_msg_param_request_list_decode(&msg, &req);

        // Пример одного параметра — можно сделать массив, если нужно больше
        const char* param_id = "TEST_PARAM";
        float param_value = 42.0f;
        uint8_t param_type = MAV_PARAM_TYPE_REAL32;

        mavlink_message_t out_msg;
        mavlink_msg_param_value_pack(
            1, 1, &out_msg,
            param_id, param_value, param_type,
            1,  // total param count
            0   // index of this param
        );

        uint8_t buf[MAVLINK_MAX_PACKET_LEN];
        uint16_t len = mavlink_msg_to_send_buffer(buf, &out_msg);
        sendto(sock, buf, len, 0, (struct sockaddr*)&dest_addr, sizeof(dest_addr));
        printf("Ответил на PARAM_REQUEST_LIST\n");
    }

    void handle_mission_request_list(const mavlink_message_t& msg) {
        mavlink_mission_request_list_t req;
        mavlink_msg_mission_request_list_decode(&msg, &req);

        mavlink_message_t out_msg;
        mavlink_msg_mission_count_pack(
            1, 1, &out_msg,
            req.target_system,
            req.target_component,
            0,             // count = 0
            MAV_MISSION_TYPE_MISSION, // тип миссии (обычно MISSION)
            0              // opaque_id = 0
        );

        uint8_t buf[MAVLINK_MAX_PACKET_LEN];
        uint16_t len = mavlink_msg_to_send_buffer(buf, &out_msg);
        sendto(sock, buf, len, 0, (struct sockaddr*)&dest_addr, sizeof(dest_addr));
        printf("Ответил на MISSION_REQUEST_LIST (0 missions)\n");
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
        // printf("Sent attitude: roll=%.2f pitch=%.2f yaw=%.2f\n", roll, pitch, yaw);
    }
};

#endif