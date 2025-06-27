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
#define I2C_TIMEOUT_MS 100

#define I2C_MASTER_TX_BUF_DISABLE 0
#define I2C_MASTER_RX_BUF_DISABLE 0

#define AK8963_ADDR        0x0C
#define MPU9250_INT_PIN_CFG  0x37
#define AK8963_CNTL1         0x0A
#define AK8963_ASAX          0x10


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

esp_err_t i2c_write_byte(uint8_t device_addr, uint8_t reg_addr, uint8_t data) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);

    i2c_master_write_byte(cmd, (device_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_write_byte(cmd, data, true);

    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);
    return ret;
}

esp_err_t i2c_read_bytes(uint8_t device_addr, uint8_t reg_addr, uint8_t* data, size_t length) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    
    // Запись адреса регистра
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (device_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    
    // Чтение данных
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (device_addr << 1) | I2C_MASTER_READ, true);
    if (length > 1) {
        i2c_master_read(cmd, data, length - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, data + length - 1, I2C_MASTER_NACK);
    
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);
    return ret;
}

// Функция чтения WHO_AM_I для диагностики
uint8_t mpu9250_read_whoami() {
    uint8_t data = 0;
    if (i2c_read_bytes(MPU9250_ADDR, 0x75, &data, 1) == ESP_OK) {
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


uint8_t ak8963_read_whoami() {
    uint8_t data = 0;
    if (i2c_read_bytes(AK8963_ADDR, 0x00, &data, 1) == ESP_OK) {
        printf("🔍 AK8963 WHO_AM_I: 0x%02X\n", data);
    } else {
        printf("❌ Failed to read WHO_AM_I from AK8963\n");
    }
    return data;
}

// Включить bypass через регистр 0x37 на MPU9250
esp_err_t mpu9250_enable_bypass() {
    ping_esp();
    return i2c_write_byte(MPU9250_ADDR, 0x37, 0x02); // INT_PIN_CFG: BYPASS_EN = 1
}

esp_err_t ak8963_init() {
    esp_err_t err;

    // Включаем bypass для доступа к магнитометру
    err = mpu9250_enable_bypass();
    if (err != ESP_OK) return err;

    uint8_t whoami = ak8963_read_whoami();
    if (whoami != 0x48) {
        printf("⚠️  Unexpected AK8963 WHO_AM_I: 0x%02X (expected 0x48)\n", whoami);
        return ESP_FAIL;
    }

    // 1. Переводим в FUSE ROM access mode
    err = i2c_write_byte(AK8963_ADDR, AK8963_CNTL1, 0x0F);
    if (err != ESP_OK) return err;
    // vTaskDelay(pdMS_TO_TICKS(20));

    // 2. Читаем коэффициенты ASAX/ASAY/ASAZ (можно сохранить, как в MicroPython)
        // def _read(self, buf, memaddr, addr):
    //     self._mpu_i2c.readfrom_mem_into(addr, memaddr, buf)
    uint8_t asa[3];
    err = i2c_read_bytes(AK8963_ADDR, AK8963_ASAX, asa, 3);
    if (err == ESP_OK) {
        printf("AK8963 ASA: X=%d, Y=%d, Z=%d\n", asa[0], asa[1], asa[2]);
    } else {
        printf("❌ Не удалось прочитать коэффициенты ASA\n");
        return err;
    }

    // // 3. Power-down перед основным режимом
    err = i2c_write_byte(AK8963_ADDR, AK8963_CNTL1, 0x00);

    // 4. Включаем Continuous measurement mode 2 (16-bit output)
    err = i2c_write_byte(AK8963_ADDR, AK8963_CNTL1, 0x16);
    // // vTaskDelay(pdMS_TO_TICKS(50));
    // if (err == ESP_OK) {
    //     printf("✅ AK8963 инициализирован в режим 100 Гц (0x16)\n");
    // }
    return err;
}

void mpu9250_init() {
    ping_esp();

    // Сброс MPU9250
    i2c_write_byte(MPU9250_ADDR, 0x6B, 0x80);
    // Пробуждаем MPU9250
    esp_err_t err = i2c_write_byte(MPU9250_ADDR, 0x6B, 0x00);
    if (err == ESP_OK) {
        printf("✅ MPU9250 пробуждён\n");

        uint8_t id = mpu9250_read_whoami();
        if (id == 0x71) {
            printf("✅ Обнаружен MPU9250 (ID: 0x%02X)\n", id);
        } else {
            printf("⚠️ MPU9250: неизвестный ID = 0x%02X\n", id);
        }
    } else {
        printf("❌ Ошибка пробуждения MPU9250: %s\n", esp_err_to_name(err));
        return;
    }

    // Настройка диапазонов акселерометра и гироскопа
    i2c_write_byte(MPU9250_ADDR, 0x1C, 0x00);  // ±2g
    i2c_write_byte(MPU9250_ADDR, 0x1B, 0x00);  // ±250 °/s

    // Включаем bypass для AK8963
    i2c_write_byte(MPU9250_ADDR, MPU9250_INT_PIN_CFG, 0x02);

    // Инициализируем магнитометр AK8963
    if (ak8963_init() == ESP_OK) {
        printf("✅ Магнитометр AK8963 инициализирован успешно\n");
    } else {
        printf("❌ Не удалось инициализировать магнитометр AK8963\n");
    }
}

void mpu9250_read_test() {
    // static MadgwickFilter filter(0.1f);

    float mx = 0.0f, my = 0.0f, mz = 0.0f;
    // err = i2c_read_bytes(AK8963_ADDR, AK8963_ASAX, asa, 3);
    ping_esp();
    uint8_t mag[6];
    if (i2c_read_bytes(MPU9250_ADDR, 0x10, mag, 6) == ESP_OK) {
        int16_t mx_raw = (mag[1] << 8) | mag[0];
        int16_t my_raw = (mag[3] << 8) | mag[2];
        int16_t mz_raw = (mag[5] << 8) | mag[4];

        mx = mx_raw * 0.15f;
        my = my_raw * 0.15f;
        mz = mz_raw * 0.15f;

        printf("🧲 mx: %.2f, my: %.2f, mz: %.2f\n", mx, my, mz);
    } else {
        printf("❌ Ошибка чтения магнитометра или переполнение\n");
    }
}

#endif