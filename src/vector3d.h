#ifndef VECTOR_H
#define VECTOR_H

#pragma once
#include <array>
#include <cmath>
#include <functional>
#include <tuple>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include <cstdlib>  // для abort()

class Vector3d {
public:
    using UpdateFunc = std::function<void()>;

    Vector3d(const std::array<int, 3>& transposition,
             const std::array<float, 3>& scaling,
             UpdateFunc update_function)
        : _vector{0, 0, 0},
          _ivector{0, 0, 0},
          update(update_function),
          cal{0, 0, 0},
          _scale(scaling),
          _transpose(transposition)
    {
        if (!is_valid_tuple(transposition)) {
            ESP_LOGE("Vector3d", "Invalid transposition: must be permutation of 0,1,2");
            abort();
        }
    }

    void calibrate(std::function<bool()> stopfunc,
                   std::function<void()> waitfunc = []() { vTaskDelay(pdMS_TO_TICKS(20)); }) {
        update();
        std::array<float, 3> maxvec = _vector;
        std::array<float, 3> minvec = _vector;

        while (!stopfunc()) {
            waitfunc();
            update();
            for (int i = 0; i < 3; ++i) {
                maxvec[i] = std::max(maxvec[i], _vector[i]);
                minvec[i] = std::min(minvec[i], _vector[i]);
            }
        }
        for (int i = 0; i < 3; ++i) {
            cal[i] = (maxvec[i] + minvec[i]) / 2.0f;
        }
    }

    float x() { update(); return (_vector[_transpose[0]] - cal[_transpose[0]]) * _scale[0]; }
    float y() { update(); return (_vector[_transpose[1]] - cal[_transpose[1]]) * _scale[1]; }
    float z() { update(); return (_vector[_transpose[2]] - cal[_transpose[2]]) * _scale[2]; }

    std::tuple<float, float, float> xyz() {
        update();
        return {
            (_vector[_transpose[0]] - cal[_transpose[0]]) * _scale[0],
            (_vector[_transpose[1]] - cal[_transpose[1]]) * _scale[1],
            (_vector[_transpose[2]] - cal[_transpose[2]]) * _scale[2]
        };
    }

    float magnitude() {
        auto [xv, yv, zv] = xyz();
        return std::sqrt(xv * xv + yv * yv + zv * zv);
    }

    float inclination() {
        auto [xv, yv, zv] = xyz();
        return std::acos(zv / std::sqrt(xv * xv + yv * yv + zv * zv)) * 180.0f / M_PI;
    }

    float elevation() {
        return 90.0f - inclination();
    }

    float azimuth() {
        auto [xv, yv, _] = xyz();
        return std::atan2(yv, xv) * 180.0f / M_PI;
    }

    int ix() const { return _ivector[0]; }
    int iy() const { return _ivector[1]; }
    int iz() const { return _ivector[2]; }
    std::array<int, 3> ixyz() const { return _ivector; }

    std::array<int, 3> transpose() const { return _transpose; }
    std::array<float, 3> scale() const { return _scale; }

    void set_raw(const std::array<int, 3>& raw_values) {
        _ivector = raw_values;
    }

    void set_scaled(const std::array<float, 3>& scaled_values) {
        _vector = scaled_values;
    }

    std::array<float, 3> _vector;   // public for direct access by update function
    std::array<int, 3> _ivector;
    UpdateFunc update;

private:
    std::array<float, 3> cal;
    std::array<float, 3> _scale;
    std::array<int, 3> _transpose;

    bool is_valid_tuple(const std::array<int, 3>& t) {
        std::array<bool, 3> seen = {false, false, false};
        for (int val : t) {
            if (val < 0 || val > 2 || seen[val]) return false;
            seen[val] = true;
        }
        return true;
    }
};

#endif // VECTOR_H
