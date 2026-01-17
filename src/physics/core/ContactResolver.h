#pragma once
#include "Contact.h"
#include <cmath>

class ContactResolver {
public:
    static void resolve(Contact& contact) {
        float separatingVelocity = contact.a->velocity.dot(contact.normal);

        if (separatingVelocity > 0) {
            return;
        }

        float newSepVelocity = -separatingVelocity * contact.a->restitution;
        float deltaVelocity = newSepVelocity - separatingVelocity;
        float impulse = deltaVelocity / contact.a->inverseMass;

        Vector3 impulsePerIMass = contact.normal * impulse;

        contact.a->velocity += impulsePerIMass * contact.a->inverseMass;

        contact.a->position += contact.normal * contact.penetration;
    }
};