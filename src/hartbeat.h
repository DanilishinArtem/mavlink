#ifndef HARTBEAT_H
#define HARTBEAT_H
#pragma once

#include "lwip/sockets.h"
#include "mavlink/common/mavlink.h"

void send_heartbeat_task() {
    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);

    sockaddr_in dest_addr = {};
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(14550);
    dest_addr.sin_addr.s_addr = inet_addr("192.168.4.2"); // IP iPhone'а в сети ESP32

    mavlink_message_t msg;
    uint8_t buf[300];

    while (true) {
        mavlink_msg_heartbeat_pack(
            1,           // system_id (оставляем 1)
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

#endif