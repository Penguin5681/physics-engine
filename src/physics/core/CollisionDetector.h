#pragma once
#include "../geometry/Box.h"
#include "../geometry/Sphere.h"
#include "Contact.h"

class CollisionDetector
{
public:
    static bool checkSpherePlane(RigidBody* sphereBody, float planeY, Contact& contact)
    {
        Sphere* sphere = (Sphere*)sphereBody->shape;

        float distance = sphereBody->position.y - planeY;
        if (distance < sphere->radius)
        {
            // this is the case of collision detection
            contact.a = sphereBody;
            contact.b = nullptr; // since this is a floor

            contact.normal = Vector3(0, 1, 0);
            contact.penetration = sphere->radius - distance;
            contact.point = sphereBody->position - Vector3(0, sphere->radius, 0);

            return true;
        }
        return false;
    }

    static bool checkBoxPlane(RigidBody* boxBody, float planeY, Contact& contact)
    {
        Box* box = (Box*)boxBody->shape;

        float bottomY = boxBody->position.y - box->halfExtents.y;

        if (bottomY < planeY)
        {
            contact.a = boxBody;
            contact.b = nullptr;
            contact.normal = Vector3(0, 1, 0);
            contact.penetration = planeY - bottomY;
            contact.point = boxBody->position - Vector3(0, box->halfExtents.y, 0);
            return true;
        }
        return false;
    }

    static bool checkSphereSphere(RigidBody* a, RigidBody* b, Contact& contact)
    {
        Sphere* sA = (Sphere*)a->shape;
        Sphere* sB = (Sphere*)b->shape;

        Vector3 midLine = a->position - b->position;
        float distance = midLine.magnitude();
        float radiusSum = sA->radius + sB->radius;

        if (distance <= radiusSum and distance > 0)
        {
            contact.a = a;
            contact.b = b;
            contact.normal = midLine * (1.0f / distance);
            contact.penetration = radiusSum - distance;
            Vector3 dir = contact.normal; // Normalized direction A->B
            contact.point = a->position + (dir * sA->radius);
            return true;
        }
        return false;
    }

    static bool checkBoxBox(RigidBody* a, RigidBody* b, Contact& contact)
    {
        Box* boxA = (Box*)a->shape;
        Box* boxB = (Box*)b->shape;

        Vector3 posA = a->position;
        Vector3 posB = b->position;

        float x_overlap = (boxA->halfExtents.x + boxB->halfExtents.x) - std::abs(posA.x - posB.x);
        if (x_overlap <= 0)
        {
            return false;
        }

        float y_overlap = (boxA->halfExtents.y + boxB->halfExtents.y) - std::abs(posA.y - posB.y);
        if (y_overlap <= 0)
        {
            return false;
        }

        float z_overlap = (boxA->halfExtents.z + boxB->halfExtents.z) - std::abs(posA.z - posB.z);
        if (z_overlap <= 0)
        {
            return false;
        }

        contact.a = a;
        contact.b = b;

        if (x_overlap < y_overlap and x_overlap < z_overlap)
        {
            contact.penetration = x_overlap;
            contact.normal = (posA.x > posB.x) ? Vector3(1, 0, 0) : Vector3(-1, 0, 0);
            contact.point = Vector3((posA.x + posB.x) * 0.5f, posA.y, posA.z);
        }
        else if (y_overlap < z_overlap)
        {
            contact.penetration = y_overlap;
            contact.normal = (posA.y > posB.y) ? Vector3(0, 1, 0) : Vector3(0, -1, 0);
            contact.point = Vector3(posA.x, (posA.y + posB.y) * 0.5f, posA.z);
        }
        else
        {
            contact.penetration = z_overlap;
            contact.normal = (posA.z > posB.z) ? Vector3(0, 0, 1) : Vector3(0, 0, -1);
            contact.point = Vector3(posA.x, posA.y, (posA.z + posB.z) * 0.5f);
        }

        return true;
    }

    static bool checkSphereBox(RigidBody* sphereBody, RigidBody* boxBody, Contact& contact)
    {
        Sphere* sphere = (Sphere*)sphereBody->shape;
        Box* box = (Box*)boxBody->shape;

        Vector3 center = sphereBody->position;
        Vector3 boxPos = boxBody->position;

        Vector3 relCenter = center - boxPos;

        Vector3 closestPoint;
        closestPoint.x = std::max(-box->halfExtents.x, std::min(relCenter.x, box->halfExtents.x));
        closestPoint.y = std::max(-box->halfExtents.y, std::min(relCenter.y, box->halfExtents.y));
        closestPoint.z = std::max(-box->halfExtents.z, std::min(relCenter.z, box->halfExtents.z));

        Vector3 distVec = relCenter - closestPoint;
        float distance = distVec.magnitude();

        if (distance < sphere->radius && distance > 0)
        {
            contact.a = sphereBody;
            contact.b = boxBody;
            contact.penetration = sphere->radius - distance;
            contact.normal = distVec * (1.0f / distance);
            contact.point = boxPos + closestPoint;

            return true;
        }
        return false;
    }

    static bool checkBoxSphere(RigidBody* boxBody, RigidBody* sphereBody, Contact& contact)
    {
        bool result = checkSphereBox(sphereBody, boxBody, contact);
        if (result)
        {
            contact.a = boxBody;
            contact.b = sphereBody;
            contact.normal = contact.normal * -1.0f;
        }
        return result;
    }
};