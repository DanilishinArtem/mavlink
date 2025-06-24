#ifndef SERVER_H
#define SERVER_H
#pragma once

// #include "esp_wifi.h"
// #include "esp_event.h"
// #include "nvs_flash.h"
// #include "esp_netif.h"
#include "lwip/sockets.h"
#include <string.h>

void udp_mavlink_server() {
    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);

    sockaddr_in addr;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_family = AF_INET;
    addr.sin_port = htons(14550);  // Порт, на который QGroundControl шлёт пакеты

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

#endif