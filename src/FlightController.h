#ifndef FLIGHTCONTROLLER_H
#define FLIGHTCONTROLLER_H

#pragma once

#include "mpu9250.h"
#include "MadgwickFilter.h"
#include "esp_timer.h"
#include "driver/i2c.h"
#include "esp_log.h"

#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_SDA_IO 9
#define I2C_MASTER_SCL_IO 8
#define I2C_MASTER_FREQ_HZ 100000
#define I2C_MASTER_TX_BUF_DISABLE 0
#define I2C_MASTER_RX_BUF_DISABLE 0

class FlightController {
public:
    MPU9250 mpu;
    MadgwickFilter filter;
    uint64_t last_time_us = 0;
    bool mag_ready = false;

    struct SensorData {
        float ax, ay, az;
        float gx, gy, gz;
        float mx, my, mz;
        float dt;
    } data;
    // 0.6045997880780726
    FlightController() : filter(0.05f) {}

    esp_err_t init() {
        i2c_config_t conf = {};
        conf.mode = I2C_MODE_MASTER;
        conf.sda_io_num = I2C_MASTER_SDA_IO;
        conf.scl_io_num = I2C_MASTER_SCL_IO;
        conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
        conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
        conf.master.clk_speed = I2C_MASTER_FREQ_HZ;

        esp_err_t ret = i2c_param_config(I2C_MASTER_NUM, &conf);
        if (ret != ESP_OK) {
            ESP_LOGE("FlightController", "I2C param config failed: %s", esp_err_to_name(ret));
            return ret;
        }

        ret = i2c_driver_install(I2C_MASTER_NUM, conf.mode,
                                 I2C_MASTER_RX_BUF_DISABLE,
                                 I2C_MASTER_TX_BUF_DISABLE, 0);
        if (ret != ESP_OK) {
            ESP_LOGE("FlightController", "I2C driver install failed: %s", esp_err_to_name(ret));
            return ret;
        }
        mpu.wake();
        mpu.calibrate_accel(1000);
        mpu.calibrate_gyro(1000);
        // Подождать инициализации сенсора
        vTaskDelay(pdMS_TO_TICKS(1000));
        return ESP_OK;
    }

    void update() {
        data.ax = mpu.get_accel().x();
        data.ay = mpu.get_accel().y();
        data.az = mpu.get_accel().z();

        data.gx = mpu.get_gyro().x();
        data.gy = mpu.get_gyro().y();
        data.gz = mpu.get_gyro().z();

        data.mx = mpu.get_mag().x();
        data.my = mpu.get_mag().y();
        data.mz = mpu.get_mag().z();

        uint64_t now = esp_timer_get_time();
        data.dt = (last_time_us == 0) ? 0.0000f : (now - last_time_us) / 1e6f;
        last_time_us = now;

        filter.update(data.ax, data.ay, data.az,
                      data.gx, data.gy, data.gz,
                      0, 0, 0,
                      data.dt);
    }

    std::tuple<float, float, float> getEuler() const {
        return filter.getEuler();
    }

    const SensorData& getSensorData() const {
        return data;
    }
};

#endif
