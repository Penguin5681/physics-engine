#pragma once
#include "../geometry/Box.h"
#include "../geometry/Shape.h"
#include "../geometry/Sphere.h"
#include "../geometry/Cylinder.h"
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

    Vector3 forceAccum;

    bool isAwake;
    float motion;
    float sleepEpsilon = 0.3f;

    // Shape
    Shape* shape;

    RigidBody(Shape* s, float x, float y, float z, float mass)
        : shape(s), position(x, y, z), angularVelocity(0, 0, 0), velocity(0, 0, 0),
          orientation(1, 0, 0, 0), isAwake(true), motion(2.0f * 0.3f)
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
            isAwake = false;
        }
    }

    void setAwake(bool awake = true)
    {
        if (awake)
        {
            isAwake = true;
            motion = 2.0f * sleepEpsilon;
        }
        else
        {
            isAwake = false;
            velocity = Vector3(0, 0, 0);
            angularVelocity = Vector3(0, 0, 0);
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
        else if (shape->type == CYLINDER)
        {
            Cylinder *c = (Cylinder*)shape;
            float r = c->radius;
            float h = c->halfHeight * 2.0f;

            float r2 = r * r;
            float h2 = h * h;

            float iy = 0.5f * mass * r2;
            float ixz = (1.0f / 12.0f) * mass * (3.0f * r2 + h2);

            it.setDiagonal(ixz, iy, ixz);
        }
        inverseInertiaTensor.setInverse(it);
    }

    bool hasFiniteMass() const { return inverseMass > 0.0f; }

    void addForce(const Vector3& f)
    {
        forceAccum += f;
        setAwake(true);
    }

    void integrate(float dt)
    {
        if (!isAwake or inverseMass <= 0.0f)
        {
            return;
        }

        Vector3 linearAcc = forceAccum * inverseMass;

        velocity += linearAcc * dt;
        position += velocity * dt;

        orientation.addScaledVector(angularVelocity, dt);
        orientation.normalize();

        velocity *= std::pow(damping, dt);
        angularVelocity *= std::pow(angularDamping, dt);

        forceAccum = Vector3(0, 0, 0);
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
        obj.set("isAwake", isAwake);

        return obj;
    }
};
