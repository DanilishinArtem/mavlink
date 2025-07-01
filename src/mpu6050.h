// mpu6050.h
#ifndef MPU6050_H
#define MPU6050_H
#pragma once

#include "vector3d.h"
#include "freertos/FreeRTOS.h"
#include <array>
#include <stdexcept>
#include <cmath>
#include <functional>
#include "driver/i2c.h"

#define I2C_TIMEOUT_MS 100
#define I2C_MASTER_NUM I2C_NUM_0

class MPUException : public std::runtime_error {
public:
    explicit MPUException(const std::string& msg) : std::runtime_error(msg) {}
};

class MPU6050 {
public:
    static constexpr uint8_t MPU_ADDRESS_0 = 0x68;
    static constexpr uint8_t MPU_ADDRESS_1 = 0x69;
    static constexpr uint8_t EXPECTED_CHIP_ID = 0x68;

    MPU6050(uint8_t address = MPU_ADDRESS_0,
            std::array<int, 3> transposition = {0, 1, 2},
            std::array<float, 3> scaling = {1.0f, 1.0f, 1.0f})
        : mpu_addr(address),
          accel(transposition, scaling, [this]() { this->update_accel(); }),
          gyro(transposition, scaling, [this]() { this->update_gyro(); }) {
        wake();
        set_passthrough(true);
        set_accel_range(0);
        set_gyro_range(0);
    }

    void wake() {
        write_func(0x01, 0x6B, mpu_addr);
    }

    void sleep() {
        write_func(0x40, 0x6B, mpu_addr);
    }

    uint8_t chip_id() {
        uint8_t buf[1];
        read_func(buf, 0x75, mpu_addr, 1);
        return buf[0];
    }

    float temperature() {
        uint8_t buf[2];
        read_func(buf, 0x41, mpu_addr, 2);
        return bytes_to_int(buf[0], buf[1]) / 340.0f + 35.0f;
    }

    bool get_passthrough() {
        uint8_t buf[1];
        read_func(buf, 0x37, mpu_addr, 1);
        return buf[0] & 0x02;
    }

    void set_passthrough(bool enable) {
        write_func(enable ? 0x02 : 0x00, 0x37, mpu_addr);
        write_func(0x00, 0x6A, mpu_addr);
    }

    uint8_t get_sample_rate() {
        uint8_t buf[1];
        read_func(buf, 0x19, mpu_addr, 1);
        return buf[0];
    }

    void set_sample_rate(uint8_t rate) {
        write_func(rate, 0x19, mpu_addr);
    }

    uint8_t get_filter_range() {
        uint8_t buf[1];
        read_func(buf, 0x1A, mpu_addr, 1);
        return buf[0] & 7;
    }

    void set_filter_range(uint8_t range) {
        if (range > 6) {
            ESP_LOGE("mpu6050", "Invalid filter range");
            abort();
        }
        write_func(range, 0x1A, mpu_addr);
    }

    uint8_t get_accel_range() {
        uint8_t buf[1];
        read_func(buf, 0x1C, mpu_addr, 1);
        return buf[0] >> 3;
    }

    void set_accel_range(uint8_t range) {
        static constexpr uint8_t ar[] = {0x00, 0x08, 0x10, 0x18};
        if (range > 6) {
            ESP_LOGE("mpu6050", "Invalid accel range");
            abort();
        }
        write_func(ar[range], 0x1C, mpu_addr);
    }

    uint8_t get_gyro_range() {
        uint8_t buf[1];
        read_func(buf, 0x1B, mpu_addr, 1);
        return buf[0] >> 3;
    }

    void set_gyro_range(uint8_t range) {
        static constexpr uint8_t gr[] = {0x00, 0x08, 0x10, 0x18};
        if (range > 6) {
            ESP_LOGE("mpu6050", "Invalid gyro range");
            abort();
        }
        write_func(gr[range], 0x1B, mpu_addr);
    }

    Vector3d& get_accel() { return accel; }
    Vector3d& get_gyro() { return gyro; }

    esp_err_t read_func(uint8_t* data, uint8_t reg_addr, uint8_t device_addr, size_t length){
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

    esp_err_t write_func(uint8_t data, uint8_t reg_addr, uint8_t device_addr){
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

    static int16_t bytes_to_int(uint8_t msb, uint8_t lsb) {
        int16_t value = (lsb << 8) | msb;
        if (msb & 0x80) value -= 0x10000;
        return value;
    }

    void calibrate_gyro(int samples = 100) {
        printf("[INFO] Start of calibration of the gyroscope...\n");
        std::array<float, 3> sum = {0, 0, 0};
        for (int i = 0; i < samples; ++i) {
            update_gyro();
            auto s = gyro.get_scaled();
            for (int j = 0; j < 3; ++j)
                sum[j] += s[j];
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        std::array<float, 3> offset;
        for (int j = 0; j < 3; ++j)
            offset[j] = sum[j] / samples;
        gyro.set_offset(offset);
        printf("[INFO] Gyro calibrated: offset = %.3f %.3f %.3f\n", offset[0], offset[1], offset[2]);
    }

    void calibrate_accel(int samples = 100) {
        printf("[INFO] Start of calibration of the accelerometer...\n");
        std::array<float, 3> sum = {0, 0, 0};
        for (int i = 0; i < samples; ++i) {
            update_accel();
            auto s = accel.get_scaled();
            for (int j = 0; j < 3; ++j)
                sum[j] += s[j];
            vTaskDelay(pdMS_TO_TICKS(10));
        }

        std::array<float, 3> offset;
        for (int j = 0; j < 3; ++j)
            offset[j] = sum[j] / samples;

        // если по Z ожидаем 1g, то вычтем его (чтобы Z после калибровки стал ≈0)
        offset[2] -= 1.0f;

        accel.set_offset(offset);
        printf("[INFO] Accel calibrated: offset = %.3f %.3f %.3f\n", offset[0], offset[1], offset[2]);
    }

private:
    uint8_t mpu_addr;

    Vector3d accel;
    Vector3d gyro;

    void update_accel() {
        uint8_t buf[6];
        esp_err_t ret = read_func(buf, 0x3B, mpu_addr, 6);
        if (ret != ESP_OK) {
            printf("[INFO] failed Update_accel function\n");
        }
        int16_t x = bytes_to_int(buf[1], buf[0]);
        int16_t y = bytes_to_int(buf[3], buf[2]);
        int16_t z = bytes_to_int(buf[5], buf[4]);
        static constexpr float scale[] = {16384, 8192, 4096, 2048};
        int r = get_accel_range();
        accel.set_raw({x, y, z});
        accel.set_scaled({x / scale[r], y / scale[r], z / scale[r]});
    }

    void update_gyro() {
        uint8_t buf[6];
        esp_err_t ret = read_func(buf, 0x43, mpu_addr, 6);
        if (ret != ESP_OK) {
            printf("[INFO] failed Update_gyro function\n");
        }
        int16_t x = bytes_to_int(buf[1], buf[0]);
        int16_t y = bytes_to_int(buf[3], buf[2]);
        int16_t z = bytes_to_int(buf[5], buf[4]);
        static constexpr float scale[] = {131, 65.5, 32.8, 16.4};
        int r = get_gyro_range();
        gyro.set_raw({x, y, z});
        constexpr float deg_to_rad = M_PI / 180.0f;
        gyro.set_scaled({
            x / scale[r] * deg_to_rad,
            y / scale[r] * deg_to_rad,
            z / scale[r] * deg_to_rad
        });
    }

};
#endif