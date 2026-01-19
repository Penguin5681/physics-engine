#pragma once
#include "Vector3.h"

class Matrix3
{
public:
    float data[9];

    Matrix3() { setIdentity(); }

    void setIdentity()
    {
        data[0] = 1;
        data[1] = 0;
        data[2] = 0;
        data[3] = 0;
        data[4] = 1;
        data[5] = 0;
        data[6] = 0;
        data[7] = 0;
        data[8] = 1;
    }

    void setDiagonal(float a, float b, float c)
    {
        setIdentity();
        data[0] = a;
        data[4] = b;
        data[8] = c;
    }

    Vector3 operator*(const Vector3& v) const
    {
        return Vector3(data[0] * v.x + data[1] * v.y + data[2] * v.z,
                       data[3] * v.x + data[4] * v.y + data[5] * v.z,
                       data[6] * v.x + data[7] * v.y + data[8] * v.z);
    }

    Matrix3 operator*(const Matrix3& o) const
    {
        Matrix3 r;
        r.data[0] = data[0] * o.data[0] + data[1] * o.data[3] + data[2] * o.data[6];
        r.data[1] = data[0] * o.data[1] + data[1] * o.data[4] + data[2] * o.data[7];
        r.data[2] = data[0] * o.data[2] + data[1] * o.data[5] + data[2] * o.data[8];

        r.data[3] = data[3] * o.data[0] + data[4] * o.data[3] + data[5] * o.data[6];
        r.data[4] = data[3] * o.data[1] + data[4] * o.data[4] + data[5] * o.data[7];
        r.data[5] = data[3] * o.data[2] + data[4] * o.data[5] + data[5] * o.data[8];

        r.data[6] = data[6] * o.data[0] + data[7] * o.data[3] + data[8] * o.data[6];
        r.data[7] = data[6] * o.data[1] + data[7] * o.data[4] + data[8] * o.data[7];
        r.data[8] = data[6] * o.data[2] + data[7] * o.data[5] + data[8] * o.data[8];
        return r;
    }

    void setInverse(const Matrix3& m)
    {
        float t4 = m.data[0] * m.data[4];
        float t6 = m.data[0] * m.data[5];
        float t8 = m.data[1] * m.data[3];
        float t10 = m.data[2] * m.data[3];
        float t12 = m.data[1] * m.data[6];
        float t14 = m.data[2] * m.data[6];

        float t16 = (t4 * m.data[8] - t6 * m.data[7] - t8 * m.data[8] + t10 * m.data[7] +
                     t12 * m.data[5] - t14 * m.data[4]);

        if (t16 == 0.0f)
            return;
        float t17 = 1.0f / t16;

        data[0] = (m.data[4] * m.data[8] - m.data[5] * m.data[7]) * t17;
        data[1] = -(m.data[1] * m.data[8] - m.data[2] * m.data[7]) * t17;
        data[2] = (m.data[1] * m.data[5] - m.data[2] * m.data[4]) * t17;
        data[3] = -(m.data[3] * m.data[8] - m.data[5] * m.data[6]) * t17;
        data[4] = (m.data[0] * m.data[8] - m.data[2] * m.data[6]) * t17;
        data[5] = -(m.data[0] * m.data[5] - m.data[2] * m.data[3]) * t17;
        data[6] = (m.data[3] * m.data[7] - m.data[4] * m.data[6]) * t17;
        data[7] = -(m.data[0] * m.data[7] - m.data[1] * m.data[6]) * t17;
        data[8] = (m.data[0] * m.data[4] - m.data[1] * m.data[3]) * t17;
    }

    Matrix3 transpose() const
    {
        Matrix3 result;
        result.data[0] = data[0];
        result.data[1] = data[3];
        result.data[2] = data[6];
        result.data[3] = data[1];
        result.data[4] = data[4];
        result.data[5] = data[7];
        result.data[6] = data[2];
        result.data[7] = data[5];
        result.data[8] = data[8];
        return result;
    }
};