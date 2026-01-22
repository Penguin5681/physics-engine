#pragma once
#include "Shape.h"

class Cylinder : public Shape {
public:
    float radius;
    float halfHeight;

    Cylinder(float r, float h) : Shape(CYLINDER), radius(r), halfHeight (h / 2.0f) {}
};