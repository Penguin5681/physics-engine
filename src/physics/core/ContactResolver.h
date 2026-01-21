#pragma once
#include "Contact.h"
#include <algorithm>
#include <cmath>

class ContactResolver
{
public:
    static void resolve(Contact& contact)
    {
        RigidBody* bodyA = contact.a;
        RigidBody* bodyB = contact.b;

        Vector3 rA = contact.point - bodyA->position;
        Vector3 rB = bodyB ? (contact.point - bodyB->position) : Vector3(0, 0, 0);

        Vector3 velA = bodyA->velocity + bodyA->angularVelocity.cross(rA);
        Vector3 velB =
            bodyB ? (bodyB->velocity + bodyB->angularVelocity.cross(rB)) : Vector3(0, 0, 0);

        Vector3 relativeVelocity = velA - velB;

        float velocityAlongNormal = relativeVelocity.dot(contact.normal);

        if (velocityAlongNormal > 0)
            return;

        float invMassSum = bodyA->inverseMass;
        if (bodyB)
            invMassSum += bodyB->inverseMass;

        Vector3 rA_cross_n = rA.cross(contact.normal);
        Vector3 ar = bodyA->inverseInertiaTensorWorld * rA_cross_n;
        Vector3 angularFactorA = ar.cross(rA);
        float angularCompA = angularFactorA.dot(contact.normal);

        float angularCompB = 0;
        if (bodyB)
        {
            Vector3 rB_cross_n = rB.cross(contact.normal);
            Vector3 br = bodyB->inverseInertiaTensorWorld * rB_cross_n;
            Vector3 angularFactorB = br.cross(rB);
            angularCompB = angularFactorB.dot(contact.normal);
        }

        float totalInverseMass = invMassSum + angularCompA + angularCompB;

        float e = bodyA->restitution;
        if (bodyB)
            e = std::min(e, bodyB->restitution);

        if (velocityAlongNormal > -2.0f) { 
            e = 0.0f;
        }
        float jn = -(1 + e) * velocityAlongNormal;
        jn /= totalInverseMass;

        Vector3 imp = contact.normal * jn;

        bodyA->velocity += imp * bodyA->inverseMass;

        bodyA->angularVelocity += bodyA->inverseInertiaTensorWorld * rA.cross(imp);

        if (bodyB)
        {
            bodyB->velocity = bodyB->velocity - (imp * bodyB->inverseMass);
            bodyB->angularVelocity =
                bodyB->angularVelocity - (bodyB->inverseInertiaTensorWorld * rB.cross(imp));
        }

        velA = bodyA->velocity + bodyA->angularVelocity.cross(rA);
        velB = bodyB ? (bodyB->velocity + bodyB->angularVelocity.cross(rB)) : Vector3(0, 0, 0);
        relativeVelocity = velA - velB;

        Vector3 tangent =
            relativeVelocity - (contact.normal * relativeVelocity.dot(contact.normal));
        float tangentMag = tangent.magnitude();

        if (tangentMag > 0.001f)
        {
            tangent = tangent * (1.0f / tangentMag);

            Vector3 rA_cross_t = rA.cross(tangent);
            Vector3 ar_t = bodyA->inverseInertiaTensorWorld * rA_cross_t;
            float angularCompA_t = ar_t.cross(rA).dot(tangent);

            float angularCompB_t = 0;
            if (bodyB)
            {
                Vector3 rB_cross_t = rB.cross(tangent);
                Vector3 br_t = bodyB->inverseInertiaTensorWorld * rB_cross_t;
                angularCompB_t = br_t.cross(rB).dot(tangent);
            }

            float frictionMass = invMassSum + angularCompA_t + angularCompB_t;

            float jf = -relativeVelocity.dot(tangent);
            jf /= frictionMass;

            float mu = bodyA->friction;
            float maxFriction = mu * jn;
            if (std::abs(jf) > maxFriction)
            {
                jf = (jf > 0) ? maxFriction : -maxFriction;
            }

            Vector3 frictionImpulse = tangent * jf;

            bodyA->velocity += frictionImpulse * bodyA->inverseMass;
            bodyA->angularVelocity += bodyA->inverseInertiaTensorWorld * rA.cross(frictionImpulse);

            if (bodyB)
            {
                bodyB->velocity = bodyB->velocity - (frictionImpulse * bodyB->inverseMass);
                bodyB->angularVelocity =
                    bodyB->angularVelocity -
                    (bodyB->inverseInertiaTensorWorld * rB.cross(frictionImpulse));
            }
        }

        const float percent = 0.8f;
        const float slop = 0.01f;

        float linearInvMassSum = bodyA->inverseMass;
        if (bodyB)
        {
            linearInvMassSum += bodyB->inverseMass;
        }

        if (linearInvMassSum)
        {
            float correctionMag =
                std::max(contact.penetration - slop, 0.0f) / linearInvMassSum * percent;

            Vector3 correction = contact.normal * correctionMag;
            bodyA->position += correction * bodyA->inverseMass;
            if (bodyB)
            {
                bodyB->position = bodyB->position - correction * bodyB->inverseMass;
            }
        }
    }
};