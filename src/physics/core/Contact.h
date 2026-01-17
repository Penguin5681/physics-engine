#pragma once
#include "Vector3.h"
#include "RigidBody.h"

struct Contact {
    RigidBody* a;
    RigidBody* b;

    Vector3 point;
    Vector3 normal;
    float penetration;
};