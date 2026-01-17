#pragma once
#include "Shape.h"
#include "../core/Vector3.h"

class Box : public Shape {
public:
    Vector3 halfExtents;
    Box(float w, float h, float d) : Shape(BOX), halfExtents(w/2.0f, h/2.0f, d/2.0f) {}
};