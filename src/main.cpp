#include <iostream>
#include "FlightController.h"
#include "MAVLinkBridge.h"
#include "esc_control.h"

extern "C" void app_main()
{
    esc_init();
    FlightController fc;
    MAVLinkBridge bridge(fc);

    vTaskDelay(pdMS_TO_TICKS(1000)); // Подождать, пока всё инициализируется

    while (true) {
        bridge.fc.update();
        bridge.send_attitude();
        // vTaskDelay(pdMS_TO_TICKS(500));
    }
}