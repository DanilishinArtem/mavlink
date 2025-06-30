#ifndef MPU9250_H
#define MPU9250_H

#pragma once
#include "mpu6050.h"
#include "vector3d.h"
#include <cstdio>
#include <array>
#include <tuple>

class MPU9250 : public MPU6050 {
public:
    static constexpr uint8_t MAG_ADDRESS = 0x0C;

    MPU9250(uint8_t address = MPU_ADDRESS_0,
            std::array<int, 3> transposition = {0, 1, 2},
            std::array<float, 3> scaling = {1.0f, 1.0f, 1.0f})
        : MPU6050(address, transposition, scaling),
          mag(transposition, scaling, [this]() { this->update_mag(); })
    {
        mag_correction = mag_setup();
        update_mag(); // первый вызов для инициализации
    }

    Vector3d& get_mag() { return mag; }

    int mag_stale() const { return _mag_stale_count; }

private:
    Vector3d mag;
    std::array<float, 3> mag_correction;
    int _mag_stale_count = 0;

    std::array<float, 3> mag_setup() {
        write_func(0x0F, 0x0A, MAG_ADDRESS);
        uint8_t buf[3];
        read_func(buf, 0x10, MAG_ADDRESS, 3);
        write_func(0x00, 0x0A, MAG_ADDRESS);
        write_func(0x16, 0x0A, MAG_ADDRESS);

        float mag_x = (0.5f * (buf[0] - 128)) / 128.0f + 1.0f;
        float mag_y = (0.5f * (buf[1] - 128)) / 128.0f + 1.0f;
        float mag_z = (0.5f * (buf[2] - 128)) / 128.0f + 1.0f;

        return {mag_x, mag_y, mag_z};
    }

    void update_mag() {
        uint8_t status;
        read_func(&status, 0x02, MAG_ADDRESS, 1);
        if ((status & 0x01) == 0) return;

        uint8_t data[6];
        read_func(data, 0x03, MAG_ADDRESS, 6);

        uint8_t status2;
        read_func(&status2, 0x09, MAG_ADDRESS, 1);

        if ((status2 & 0x08) > 0) {
            _mag_stale_count++;
            return;
        }
        
        int16_t x_raw = bytes_to_int(data[1], data[0]);
        int16_t y_raw = bytes_to_int(data[3], data[2]);
        int16_t z_raw = bytes_to_int(data[5], data[4]);

        float scale = 0.15f;

        mag.set_raw({x_raw, y_raw, -z_raw});
        mag.set_scaled({
            x_raw * mag_correction[0] * scale,
            y_raw * mag_correction[1] * scale,
            -z_raw * mag_correction[2] * scale
        });

        _mag_stale_count = 0;
    }
};

#endif // MPU9250_H