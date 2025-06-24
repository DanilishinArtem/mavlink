// #ifdef FILTER_H
// #define FILTER_H

#pragma once
#include <cmath>
#include <stdint.h>

class MadgwickFilter {
public:
    MadgwickFilter(float beta = 0.1f);

    void update(float ax, float ay, float az,
                float gx, float gy, float gz,
                float mx, float my, float mz, float dt);

    float getPitch() const;
    float getRoll() const;
    float getYaw() const;

private:
    float beta;
    float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;

    void normalize(float& x, float& y, float& z);
};

// #endif