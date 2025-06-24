#include <iostream>
#include "wifi.h"
#include "server.h"
#include "hartbeat.h"

// #include "mpu.h"
#include "gyroData.h"

extern "C" void app_main()
{
    // part of communication with qgroundcontrol --->
    // nvs_flash_init();
    // wifi_init_softap();
    // // Задача на приём MAVLink-пакетов
    // xTaskCreate(
    //     [](void*){
    //         udp_mavlink_server();  // функция с while(1)
    //     },
    //     "mavlink_recv",
    //     4096,
    //     NULL,
    //     5,
    //     NULL
    // );

    // // Задача на отправку heartbeat
    // xTaskCreate(
    //     [](void*){
    //         send_heartbeat_task();
    //     },
    //     "heartbeat_task",
    //     4096,
    //     NULL,
    //     5,
    //     NULL
    // );
    // printf("Wi-Fi + MAVLink tasks started.\n");


    // part of displaying whiami function ---->
    // setup();

    // part of displaying gero datas --------->
    // Инициализация I2C
    if (i2c_master_init() != ESP_OK) {
        printf("Critical I2C init error!\n");
        return;
    }
    // Даем время на инициализацию устройств
    vTaskDelay(pdMS_TO_TICKS(1500));
    // Инициализация MPU9250 с диагностикой
    mpu9250_init();
    // Дополнительная задержка перед чтением
    while(true){
        mpu9250_read_test();
    }
}