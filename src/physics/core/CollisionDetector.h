#pragma once
#include "../geometry/Box.h"
#include "../geometry/Sphere.h"
#include "Contact.h"

class CollisionDetector {
 public:
  static bool checkSpherePlane(RigidBody* sphereBody, float planeY,
                               Contact& contact) {
    Sphere* sphere = (Sphere*)sphereBody->shape;

    float distance = sphereBody->position.y - planeY;
    if (distance < sphere->radius) {
      // this is the case of collision detection
      contact.a = sphereBody;
      contact.b = nullptr;  // since this is a floor

      contact.normal = Vector3(0, 1, 0);
      contact.penetration = sphere->radius - distance;

      return true;
    }
    return false;
  }

  static bool checkBoxPlane(RigidBody* boxBody, float planeY,
                            Contact& contact) {
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

  static bool checkSphereSphere(RigidBody* a, RigidBody* b, Contact& contact) {
    Sphere* sA = (Sphere*)a->shape;
    Sphere* sB = (Sphere*)b->shape;

    Vector3 midLine = a->position - b->position;
    float distance = midLine.magnitude();
    float radiusSum = sA->radius + sB->radius;

    if (distance <= radiusSum and distance > 0) {
      contact.a = a;
      contact.b = b;
      contact.normal = midLine * (1.0f / distance);
      contact.penetration = radiusSum - distance;
      return true;
    }
    return false;
  }

  static bool checkBoxBox(RigidBody* a, RigidBody* b, Contact& contact) {
    Box* boxA = (Box*)a->shape;
    Box* boxB = (Box*)b->shape;

    Vector3 posA = a->position;
    Vector3 posB = b->position;

    float x_overlap =
        (boxA->halfExtents.x + boxB->halfExtents.x) - std::abs(posA.x - posB.x);
    if (x_overlap <= 0) {
      return false;
    }

    float y_overlap =
        (boxA->halfExtents.y + boxB->halfExtents.y) - std::abs(posA.y - posB.y);
    if (y_overlap <= 0) {
      return false;
    }

    float z_overlap =
        (boxA->halfExtents.z + boxB->halfExtents.z) - std::abs(posA.z - posB.z);
    if (z_overlap <= 0) {
      return false;
    }

    contact.a = a;
    contact.b = b;

    if (x_overlap < y_overlap and x_overlap < z_overlap) {
      contact.penetration = x_overlap;
      contact.normal = (posA.x > posB.x) ? Vector3(1, 0, 0) : Vector3(-1, 0, 0);
    } else if (y_overlap < y_overlap and x_overlap < z_overlap) {
      contact.penetration = y_overlap;
      contact.normal = (posA.y > posB.y) ? Vector3(0, 1, 0) : Vector3(0, -1, 0);
    } else {
      contact.penetration = x_overlap;
      contact.normal = (posA.z > posB.z) ? Vector3(0, 0, 1) : Vector3(0, 0, -1);
    }

    return true;
  }

  static bool checkSphereBox(RigidBody* sphereBody, RigidBody* boxBody,
                             Contact& contact) {
    Sphere* sphere = (Sphere*)sphereBody->shape;
    Box* box = (Box*)boxBody->shape;

    Vector3 closestPoint;
    Vector3 spherePos = sphereBody->position;
    Vector3 boxPos = boxBody->position;

    // Find the closest point on the box to the sphere center
    closestPoint.x =
        std::max(boxPos.x - box->halfExtents.x,
                 std::min(spherePos.x, boxPos.x + box->halfExtents.x));
    closestPoint.y =
        std::max(boxPos.y - box->halfExtents.y,
                 std::min(spherePos.y, boxPos.y + box->halfExtents.y));
    closestPoint.z =
        std::max(boxPos.z - box->halfExtents.z,
                 std::min(spherePos.z, boxPos.z + box->halfExtents.z));

    Vector3 direction = spherePos - closestPoint;
    float distanceSquared = direction.magnitudeSquared();

    if (distanceSquared <= sphere->radius * sphere->radius) {
      contact.a = sphereBody;
      contact.b = boxBody;

      float distance = std::sqrt(distanceSquared);
      if (distance > 0) {
        contact.normal = direction * (1.0f / distance);
        contact.penetration = sphere->radius - distance;
      } else {
        contact.normal = Vector3(0, 1, 0);
        contact.penetration = sphere->radius;
      }

      return true;
    }
    return false;
  }

  static bool checkBoxSphere(RigidBody* boxBody, RigidBody* sphereBody,
                             Contact& contact) {
    bool result = checkSphereBox(sphereBody, boxBody, contact);
    if (result) {
      // Swap bodies and flip normal
      contact.a = boxBody;
      contact.b = sphereBody;
      contact.normal = contact.normal * -1.0f;
    }
    return result;
  }
};