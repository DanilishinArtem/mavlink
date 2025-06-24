#include "MadgwickFilter.h"
#include <math.h>

MadgwickFilter::MadgwickFilter(float beta) : beta(beta), q0(1), q1(0), q2(0), q3(0) {}

void MadgwickFilter::normalize(float& x, float& y, float& z) {
    float norm = sqrtf(x * x + y * y + z * z);
    if (norm == 0.0f) return;
    x /= norm;
    y /= norm;
    z /= norm;
}

void MadgwickFilter::update(float ax, float ay, float az,
                            float gx, float gy, float gz,
                            float mx, float my, float mz, float dt) {
    normalize(ax, ay, az);
    normalize(mx, my, mz);

    float q0q0 = q0 * q0;
    float q0q1 = q0 * q1;
    float q0q2 = q0 * q2;
    float q0q3 = q0 * q3;
    float q1q1 = q1 * q1;
    float q1q2 = q1 * q2;
    float q1q3 = q1 * q3;
    float q2q2 = q2 * q2;
    float q2q3 = q2 * q3;
    float q3q3 = q3 * q3;

    // Magnetic reference direction
    float _2q0mx = 2.0f * q0 * mx;
    float _2q0my = 2.0f * q0 * my;
    float _2q0mz = 2.0f * q0 * mz;
    float _2q1mx = 2.0f * q1 * mx;
    
    float hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1
             + 2.0f * q1 * my * q2 + 2.0f * q1 * mz * q3 - mx * q2q2 - mx * q3q3;
    float hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1
             + 2.0f * q1 * mx * q2 + my * q1q1 + 2.0f * q2 * mz * q3 - my * q2q2 - my * q3q3;
    float _2bx = sqrtf(hx * hx + hy * hy);
    float _2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0
               + 2.0f * q1 * mx * q3 + mz * q1q1 + 2.0f * q2 * my * q3 - mz * q2q2 + mz * q3q3;
    float _4bx = 2.0f * _2bx;
    float _4bz = 2.0f * _2bz;

    // Gradient descent algorithm corrective step
    float s0 = -2.0f * (q2 * (2.0f * q1q3 - 2.0f * q0q2 - ax) +
                        q1 * (2.0f * q0q1 + 2.0f * q2q3 - ay))
               - _2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx)
               + (-_2bx * q3 + _2bz * q1) *
                 (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my)
               + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);

    float s1 = 2.0f * (q3 * (2.0f * q1q3 - 2.0f * q0q2 - ax) +
                       q0 * (2.0f * q0q1 + 2.0f * q2q3 - ay))
              - 4.0f * q1 * (1.0f - 2.0f * q1q1 - 2.0f * q2q2 - az)
              + _2bz * q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx)
              + (_2bx * q2 + _2bz * q0) *
                (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my)
              + (_2bx * q3 - _4bz * q1) *
                (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);

    float s2 = -2.0f * (q0 * (2.0f * q1q3 - 2.0f * q0q2 - ax) +
                        q3 * (2.0f * q0q1 + 2.0f * q2q3 - ay))
               - 4.0f * q2 * (1.0f - 2.0f * q1q1 - 2.0f * q2q2 - az)
               + (-_4bx * q2 - _2bz * q0) *
                 (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx)
               + (_2bx * q1 + _2bz * q3) *
                 (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my)
               + (_2bx * q0 - _4bz * q2) *
                 (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);

    float s3 = 2.0f * (q1 * (2.0f * q1q3 - 2.0f * q0q2 - ax) +
                       q2 * (2.0f * q0q1 + 2.0f * q2q3 - ay))
              + (-_4bx * q3 + _2bz * q1) *
                (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx)
              + (-_2bx * q0 + _2bz * q2) *
                (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my)
              + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);

    float recipNorm = 1.0f / sqrtf(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
    s0 *= recipNorm;
    s1 *= recipNorm;
    s2 *= recipNorm;
    s3 *= recipNorm;

    // Integrate rate of change of quaternion
    float qDot0 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz) - beta * s0;
    float qDot1 = 0.5f * (q0 * gx + q2 * gz - q3 * gy) - beta * s1;
    float qDot2 = 0.5f * (q0 * gy - q1 * gz + q3 * gx) - beta * s2;
    float qDot3 = 0.5f * (q0 * gz + q1 * gy - q2 * gx) - beta * s3;

    q0 += qDot0 * dt;
    q1 += qDot1 * dt;
    q2 += qDot2 * dt;
    q3 += qDot3 * dt;

    recipNorm = 1.0f / sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;
}

float MadgwickFilter::getPitch() const {
    return asinf(-2.0f * (q1 * q3 - q0 * q2)) * 180.0f / M_PI;
}

float MadgwickFilter::getRoll() const {
    return atan2f(2.0f * (q0 * q1 + q2 * q3),
                  1.0f - 2.0f * (q1 * q1 + q2 * q2)) * 180.0f / M_PI;
}

float MadgwickFilter::getYaw() const {
    return atan2f(2.0f * (q0 * q3 + q1 * q2),
                  1.0f - 2.0f * (q2 * q2 + q3 * q3)) * 180.0f / M_PI;
}