#pragma once
#include "Contact.h"
#include <cmath>

class ContactResolver {
public:
    static void resolve(Contact& contact) {
        Vector3 relativeVelocity = contact.a->velocity;

        if (contact.b) {
            relativeVelocity = relativeVelocity - contact.b->velocity;
        }

        float velocityAlongNormal = relativeVelocity.dot(contact.normal);

        if (velocityAlongNormal > 0) {
            return;
        }

        float e = contact.a->restitution;
        if (contact.b) {
            e = std::min(e, contact.b->restitution);
        }

        float j = -(1 + e) * velocityAlongNormal;

        float inverseMassSum = contact.a->inverseMass;
        if (contact.b) {
            inverseMassSum += contact.b->inverseMass;
        }

        j /= inverseMassSum;

        Vector3 impulse = contact.normal * j;

        contact.a->velocity += impulse * contact.a->inverseMass;

        if (contact.b) {
            contact.b->velocity = contact.b->velocity - impulse * contact.b->inverseMass;
        }

        const float percent = 0.2f;
        const float slop = 0.01f;
        float correctionMag = std::max(contact.penetration - slop, 0.0f) / inverseMassSum * percent;

        Vector3 correction = contact.normal * correctionMag;

        contact.a->position += correction * contact.a->inverseMass;
        if (contact.b) {
            contact.b->position = contact.b->position - correction * contact.b->inverseMass;
        }
    }
};