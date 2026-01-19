#pragma once
#include "../geometry/Box.h"
#include "../geometry/Shape.h"
#include "../geometry/Sphere.h"
#include "Matrix3x3.h"
#include "Quaternion.h"
#include "Vector3.h"
class RigidBody
{
public:
    // Linear properties
    Vector3 position;
    Vector3 velocity;
    Vector3 acceleration;
    float inverseMass;
    float damping = 0.99f;
    float restitution = 0.7f;
    float friction = 0.5f;

    // Angular properties
    Quaternion orientation;
    Vector3 angularVelocity;
    Vector3 torqueAccum;
    float angularDamping = 0.50f;
    Matrix3 inverseInertiaTensor;
    Matrix3 inverseInertiaTensorWorld;

    // Shape
    Shape* shape;

    RigidBody(Shape* s, float x, float y, float z, float mass)
        : shape(s), position(x, y, z), angularVelocity(0, 0, 0), velocity(0, 0, 0),
          orientation(1, 0, 0, 0)
    {
        if (mass > 0.0f)
        {
            inverseMass = 1.0f / mass;
            calculateInertiaTensor(mass);
        }
        else
        {
            inverseMass = 0.0f;
            inverseInertiaTensor.setIdentity();
            inverseInertiaTensor.data[0] = 0;
            inverseInertiaTensor.data[4] = 0;
            inverseInertiaTensor.data[8] = 0;
        }
    }

    void calculateInertiaTensor(float mass)
    {
        Matrix3 it;
        if (shape->type == SPHERE)
        {
            Sphere* s = (Sphere*)shape;
            float coeff = 0.4f * mass * s->radius * s->radius;
            it.setDiagonal(coeff, coeff, coeff);
        }
        else if (shape->type == BOX)
        {
            Box* b = (Box*)shape;

            float w = b->halfExtents.x * 2;
            float h = b->halfExtents.y * 2;
            float d = b->halfExtents.z * 2;

            float ex2 = w * w;
            float ey2 = h * h;
            float ez2 = d * d;

            float factor = mass / 12.0f;

            it.setDiagonal(factor * (ey2 + ez2), factor * (ex2 + ez2), factor * (ex2 + ey2));
        }
        inverseInertiaTensor.setInverse(it);
    }

    bool hasFiniteMass() const { return inverseMass > 0.0f; }

    void integrate(float dt)
    {
        if (inverseMass <= 0.0f)
            return;

        position += velocity * dt;

        orientation.addScaledVector(angularVelocity, dt);
        orientation.normalize();

        velocity *= std::pow(damping, dt);
        angularVelocity *= std::pow(angularDamping, dt);
    }

    emscripten::val toJs() const
    {
        emscripten::val obj = emscripten::val::object();
        obj.set("pos", position.toJs());

        emscripten::val rot = emscripten::val::object();
        rot.set("w", orientation.w);
        rot.set("x", orientation.x);
        rot.set("y", orientation.y);
        rot.set("z", orientation.z);
        obj.set("rot", rot);

        return obj;
    }
};