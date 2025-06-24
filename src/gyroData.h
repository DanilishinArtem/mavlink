#ifndef HYRODATA_H
#define HYRODATA_H
#pragma once

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "MadgwickFilter.h"

#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_SDA_IO 9
#define I2C_MASTER_SCL_IO 8
#define I2C_MASTER_FREQ_HZ 100000
#define MPU9250_ADDR 0x68
#define I2C_TIMEOUT_MS 5000

#define I2C_MASTER_TX_BUF_DISABLE 0
#define I2C_MASTER_RX_BUF_DISABLE 0


// Инициализация I2C с улучшенной обработкой ошибок
esp_err_t i2c_master_init() {
    i2c_config_t conf = {};
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;

    esp_err_t ret = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (ret != ESP_OK) {
        ESP_LOGE("I2C", "Config error: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ret = i2c_driver_install(I2C_MASTER_NUM, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
    if (ret != ESP_OK) {
        ESP_LOGE("I2C", "Driver install error: %s", esp_err_to_name(ret));
    }
    return ret;
}

// Функция записи с правильной обработкой ACK
esp_err_t mpu9250_write_byte(uint8_t reg, uint8_t data) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    
    i2c_master_write_byte(cmd, (MPU9250_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    
    if (ret != ESP_OK) {
        ESP_LOGE("MPU", "Write error [0x%02X]: %s", reg, esp_err_to_name(ret));
    }
    return ret;
}

esp_err_t mpu9250_read_bytes(uint8_t reg, uint8_t* data, size_t len) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    
    // Фаза записи (установка указателя регистра)
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU9250_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    
    // Фаза чтения
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU9250_ADDR << 1) | I2C_MASTER_READ, true);
    
    if (len > 1) {
        i2c_master_read(cmd, data, len - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, data + len - 1, I2C_MASTER_LAST_NACK);
    
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);
    
    if (ret != ESP_OK) {
        ESP_LOGE("MPU", "Read error [0x%02X]: %s", reg, esp_err_to_name(ret));
    }
    return ret;
}

// Функция чтения WHO_AM_I для диагностики
uint8_t mpu9250_read_whoami() {
    uint8_t data = 0;
    if (mpu9250_read_bytes(0x75, &data, 1) == ESP_OK) {
        printf("WHO_AM_I: 0x%02X\n", data);
    } else {
        printf("Failed to read WHO_AM_I\n");
    }
    return data;
}

void ping_esp(){
    bool found = false;
    for (uint8_t addr = 1; addr < 127; addr++) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);
        esp_err_t err = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(100));
        i2c_cmd_link_delete(cmd);
        if (err == ESP_OK) {
            found = true;
        }
    }
    if (!found) {
        printf("❌ MPU9250 NOT found. Check wiring, power, and pins.\n");
    }
}

void mpu9250_init() {
    // Добавляем задержку для инициализации устройства
    ping_esp();
    // Пробуждаем устройство
    esp_err_t err = mpu9250_write_byte(0x6B, 0x00);
    if (err == ESP_OK) {
        printf("✅ MPU9250 wake-up successful\n");
        // Проверяем идентификатор устройства
        uint8_t id = mpu9250_read_whoami();
        if (id == 0x71) {
            printf("✅ Valid MPU9250 detected (ID: 0x%02X)\n", id);
        } else {
            printf("⚠️ Unknown device ID: 0x%02X\n", id);
        }
    } else {
        printf("❌ Failed to wake up MPU9250: %s\n", esp_err_to_name(err));
        return;
    }
    
    // Настройка акселерометра и гироскопа
    mpu9250_write_byte(0x1C, 0x00);  // ±2g
    mpu9250_write_byte(0x1B, 0x00);  // ±250°/s
    vTaskDelay(pdMS_TO_TICKS(100));
}

// void mpu9250_read_test() {
//     ping_esp();
//     uint8_t buffer[14];  // ACCEL(6) + TEMP(2) + GYRO(6)
//     if (mpu9250_read_bytes(0x3B, buffer, 14) == ESP_OK) {
//         int16_t ax = (buffer[0] << 8) | buffer[1];
//         int16_t ay = (buffer[2] << 8) | buffer[3];
//         int16_t az = (buffer[4] << 8) | buffer[5];

//         int16_t gx = (buffer[8] << 8) | buffer[9];
//         int16_t gy = (buffer[10] << 8) | buffer[11];
//         int16_t gz = (buffer[12] << 8) | buffer[13];

//         printf("📈 Accel: X=%d Y=%d Z=%d | Gyro: X=%d Y=%d Z=%d\n", ax, ay, az, gx, gy, gz);
//     } else {
//         printf("❌ Failed to read MPU9250 data\n");
//     }
// }

void mpu9250_read_test(){
    static MadgwickFilter filter(0.1f); // β = 0.1 (подберите при необходимости)
    static uint64_t last_time_us = 0;
    ping_esp();
    uint8_t buffer[14];  // ACCEL(6) + TEMP(2) + GYRO(6)
    if (mpu9250_read_bytes(0x3B, buffer, 14) == ESP_OK) {
        // Распаковка
        int16_t ax = (buffer[0] << 8) | buffer[1];
        int16_t ay = (buffer[2] << 8) | buffer[3];
        int16_t az = (buffer[4] << 8) | buffer[5];
        int16_t gx = (buffer[8] << 8) | buffer[9];
        int16_t gy = (buffer[10] << 8) | buffer[11];
        int16_t gz = (buffer[12] << 8) | buffer[13];

        // Конвертация: raw → физические величины
        float axf = ax / 16384.0f;  // ±2g
        float ayf = ay / 16384.0f;
        float azf = az / 16384.0f;
        float gxf = gx / 131.0f;    // ±250 °/s
        float gyf = gy / 131.0f;
        float gzf = gz / 131.0f;

        // Магнитометр (опционально — пока передаём нули)
        float mx = 0.0f, my = 0.0f, mz = 0.0f;

        // Расчёт времени между вызовами
        uint64_t now = esp_timer_get_time(); // µs
        float dt = (last_time_us == 0) ? 0.0001f : (now - last_time_us) / 1e6f;
        last_time_us = now;

        // Обновление фильтра
        filter.update(axf, ayf, azf, gxf, gyf, gzf, mx, my, mz, dt);

        // Вывод ориентации
        printf("🎯 Pitch: %.2f° | Roll: %.2f° | Yaw: %.2f°\n",
               filter.getPitch(), filter.getRoll(), filter.getYaw());
    } else {
        printf("❌ Failed to read MPU9250 data\n");
    }
}

#endif