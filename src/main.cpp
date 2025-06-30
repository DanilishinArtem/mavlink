#include <iostream>
#include "FlightController.h"
#include "MAVLinkBridge.h"

extern "C" void app_main()
{
    FlightController fc;
    MAVLinkBridge bridge(fc);

    vTaskDelay(pdMS_TO_TICKS(1000)); // Подождать, пока всё инициализируется

    while (true) {
        bridge.fc.update();
        bridge.send_attitude();
        // vTaskDelay(pdMS_TO_TICKS(50));
    }
}