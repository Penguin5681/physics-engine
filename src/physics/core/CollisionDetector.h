#pragma once
#include "Contact.h"
#include "../geometry/Sphere.h"
#include "../geometry/Box.h"

class CollisionDetector {
public:
    static bool checkSpherePlane(RigidBody* sphereBody, float planeY, Contact& contact) {
        Sphere* sphere = (Sphere*)sphereBody->shape;

        float distance = sphereBody->position.y - planeY;
        if (distance < sphere->radius) {
            // this is the case of collision detection
            contact.a = sphereBody;
            contact.b = nullptr; // since this is a floor

            contact.normal = Vector3(0, 1, 0);
            contact.penetration = sphere->radius - distance;

            return true;
        }
        return false;
    }

    static bool checkBoxPlane(RigidBody* boxBody, float planeY, Contact& contact) {
        Box* box = (Box*)boxBody->shape;

        float bottomY = boxBody->position.y - box->halfExtents.y;

        if (bottomY < planeY) {
            contact.a = boxBody;
            contact.b = nullptr;
            contact.normal = Vector3(0, 1, 0);
            contact.penetration = planeY - bottomY;
            return true;
        }
        return false;
    }
};