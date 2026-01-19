#pragma once
#include "Vector3.h"

#include <cmath>

class Quaternion
{
public:
    float w, x, y, z;

    Quaternion(float w = 1, float x = 0, float y = 0, float z = 0) : w(w), x(x), y(y), z(z) {}

    void normalize()
    {
        float d = w * w + x * x + y * y + z * z;
        if (d == 0)
        {
            w = 1;
            return;
        }
        d = 1.0f / std::sqrt(d);
        w *= d;
        x *= d;
        y *= d;
        z *= d;
    }

    void addScaledVector(const Vector3& v, float scale) {
      Quaternion q(0, v.x * scale, v.y * scale, v.z * scale);
      q = q * (*this);
      w += q.w * 0.5f;
      x += q.x * 0.5f;
      y += q.y * 0.5f;
      z += q.z * 0.5f;
    }

    Quaternion operator*(const Quaternion& q) const
    {
        return Quaternion(
            w * q.w - x * q.x - y * q.y - z * q.z, w * q.x + x * q.w + y * q.z - z * q.y,
            w * q.y + y * q.w + z * q.x - x * q.z, w * q.z + z * q.w + x * q.y - y * q.x);
    }
};