#include <iostream>
#include "wifi.h"
#include "server.h"
#include "hartbeat.h"

// #include "mpu.h"
#include "FlightController.h"
#include "MAVLinkBridge.h"

extern "C" void app_main()
{
    // part of communication with qgroundcontrol --->
    nvs_flash_init();
    wifi_init_softap();
    // Задача на приём MAVLink-пакетов
    xTaskCreate(
        [](void*){
            udp_mavlink_server();  // функция с while(1)
        },
        "mavlink_recv",
        4096,
        NULL,
        5,
        NULL
    );

    // Задача на отправку heartbeat
    xTaskCreate(
        [](void*){
            send_heartbeat_task();
        },
        "heartbeat_task",
        4096,
        NULL,
        5,
        NULL
    );
    printf("Wi-Fi + MAVLink tasks started.\n");

    FlightController fc;
    fc.init();
    MAVLinkBridge bridge(fc);

    vTaskDelay(pdMS_TO_TICKS(1000)); // Подождать, пока всё инициализируется

    while (true) {
        fc.update();                 // Обновляем сенсоры и фильтр
        bridge.receive();           // Чтение входящих сообщений
        bridge.send_attitude();     // Отправка roll/pitch/yaw

        vTaskDelay(pdMS_TO_TICKS(30));
    }
}