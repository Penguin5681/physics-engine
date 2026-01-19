#pragma once
#include <cmath>
#include <emscripten/val.h>

class Vector3
{
public:
    float x, y, z;

    Vector3() : x(0), y(0), z(0) {}
    Vector3(float x, float y, float z) : x(x), y(y), z(z) {}

    void invert()
    {
        x = -x;
        y = -y;
        z = -z;
    }

    float dot(const Vector3& v) const { return x * v.x + y * v.y + z * v.z; }

    float magnitude() const { return std::sqrt(x * x + y * y + z * z); }

    float magnitudeSquared() const { return x * x + y * y + z * z; }

    void normalize()
    {
        float m = magnitude();
        if (m > 0)
        {
            (*this) *= (1.0f / m);
        }
    }

    Vector3 operator+(const Vector3& v) const { return Vector3(x + v.x, y + v.y, z + v.z); }

    Vector3 operator-(const Vector3& v) const { return Vector3(x - v.x, y - v.y, z - v.z); }

    Vector3 operator*(const float value) const { return Vector3(x * value, y * value, z * value); }

    void operator+=(const Vector3& v)
    {
        x += v.x;
        y += v.y;
        z += v.z;
    }

    void operator*=(const float value)
    {
        x *= value;
        y *= value;
        z *= value;
    }

    Vector3 cross(const Vector3& v) const
    {
        return Vector3(y * v.z - z * v.y, z * v.x - x * v.z, x * v.y - y * v.x);
    }

    emscripten::val toJs() const
    {
        emscripten::val obj = emscripten::val::object();
        obj.set("x", x);
        obj.set("y", y);
        obj.set("z", z);
        return obj;
    }
};