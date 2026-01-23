#pragma once
#include "../geometry/Box.h"
#include "../geometry/Cylinder.h"
#include "../geometry/Sphere.h"
#include "Contact.h"

class CollisionDetector
{
public:
    static Vector3 toLocal(RigidBody* body, const Vector3& worldPt)
    {
        Vector3 rel = worldPt - body->position;
        Quaternion invQ = body->orientation;
        invQ.invert();
        return invQ.rotate(rel);
    }

    static Vector3 toWorld(RigidBody* body, const Vector3& localPt)
    {
        return body->position + body->orientation.rotate(localPt);
    }

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

        Vector3 corners[8] = {
            Vector3(box->halfExtents.x, box->halfExtents.y, box->halfExtents.z),
            Vector3(-box->halfExtents.x, box->halfExtents.y, box->halfExtents.z),
            Vector3(box->halfExtents.x, -box->halfExtents.y, box->halfExtents.z),
            Vector3(-box->halfExtents.x, -box->halfExtents.y, box->halfExtents.z),
            Vector3(box->halfExtents.x, box->halfExtents.y, -box->halfExtents.z),
            Vector3(-box->halfExtents.x, box->halfExtents.y, -box->halfExtents.z),
            Vector3(box->halfExtents.x, -box->halfExtents.y, -box->halfExtents.z),
            Vector3(-box->halfExtents.x, -box->halfExtents.y, -box->halfExtents.z)};

        float lowestY = 100000.0f;
        Vector3 lowestPoint;

        for (int i = 0; i < 8; i++)
        {
            Quaternion q = boxBody->orientation;
            Vector3 v = corners[i];

            float x = q.x, y = q.y, z = q.z, w = q.w;
            float x2 = x + x, y2 = y + y, z2 = z + z;
            float xx = x * x2, xy = x * y2, xz = x * z2;
            float yy = y * y2, yz = y * z2, zz = z * z2;
            float wx = w * x2, wy = w * y2, wz = w * z2;

            Vector3 rotated;
            rotated.x = (1.0f - (yy + zz)) * v.x + (xy - wz) * v.y + (xz + wy) * v.z;
            rotated.y = (xy + wz) * v.x + (1.0f - (xx + zz)) * v.y + (yz - wx) * v.z;
            rotated.z = (xz - wy) * v.x + (yz + wx) * v.y + (1.0f - (xx + yy)) * v.z;

            Vector3 worldPos = boxBody->position + rotated;

            if (worldPos.y < lowestY)
            {
                lowestY = worldPos.y;
                lowestPoint = worldPos;
            }
        }

        if (lowestY < planeY)
        {
            contact.a = boxBody;
            contact.b = nullptr;
            contact.normal = Vector3(0, 1, 0);
            contact.penetration = planeY - lowestY;
            contact.point = lowestPoint;
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

    static bool checkCylinderPlane(RigidBody* cylBody, float planeY, Contact& contact)
    {
        Cylinder* cylinder = (Cylinder*)cylBody->shape;

        std::vector<Vector3> rimPoints;
        float angleStep = (3.14159f * 2.0f) / 8.0f;

        for (int i = 0; i < 8; i++)
        {
            float theta = i * angleStep;
            float x = cylinder->radius * std::cos(theta);
            float z = cylinder->radius * std::sin(theta);

            rimPoints.push_back(Vector3(x, -cylinder->halfHeight, z));
            rimPoints.push_back(Vector3(x, cylinder->halfHeight, z));
        }

        float lowestY = 100000.0f;
        Vector3 lowestPoint;
        bool collided = false;

        for (const auto& p : rimPoints)
        {
            Vector3 worldPt = cylBody->orientation.rotate(p);
            worldPt += cylBody->position;

            if (worldPt.y < lowestY)
            {
                lowestY = worldPt.y;
                lowestPoint = worldPt;
            }
        }

        if (lowestY < planeY)
        {
            contact.a = cylBody;
            contact.b = nullptr;
            contact.normal = Vector3(0, 1, 0);
            contact.penetration = planeY - lowestY;
            contact.point = lowestPoint;
            return true;
        }
        return false;
    }


    static Vector3 radiusAtAngle(float angle, float radius)
    {
        return Vector3(std::cos(angle) * radius, 0, std::sin(angle) * radius);
    }

    static void generateCylinderPoints(Cylinder* cylinder, std::vector<Vector3>& outPoints)
    {
        int segments = 0;
        float step = 6.28318f / segments;
        for (int i = 0; i < segments; i++)
        {
            float angle = i * step;
            float x = std::cos(angle) * cylinder->radius;
            float z = std::sin(angle) * cylinder->radius;

            outPoints.push_back(Vector3(x, cylinder->halfHeight, z));
            outPoints.push_back(Vector3(x, -cylinder->halfHeight, z));
        }
    }

    static bool checkSphereCylinder(RigidBody* sphereBody, RigidBody* cylBody, Contact& contact)
    {
        Sphere* sphere = (Sphere*)sphereBody->shape;
        Cylinder* cylinder = (Cylinder*)cylBody->shape;

        Vector3 localSphere = toLocal(cylBody, sphereBody->position);
        float clampedY =
            std::max(-cylinder->halfHeight, std::min(localSphere.y, cylinder->halfHeight));

        Vector3 closestLocal;
        Vector3 xzVec(localSphere.x, 0, localSphere.z);
        float distXZ = xzVec.magnitude();

        if (distXZ > 0.0001f)
        {
            float clamedR = std::min(distXZ, cylinder->radius);

            if (distXZ < cylinder->radius and std::abs(localSphere.y) < cylinder->halfHeight)
            {
                float distToSide = cylinder->radius - distXZ;
                float distToTop = cylinder->halfHeight - localSphere.y;
                float distToBottom = localSphere.y - (-cylinder->halfHeight);

                if (distToSide < distToTop and distToSide < distToBottom)
                {
                    closestLocal = xzVec * (cylinder->radius / distXZ);
                    closestLocal.y = localSphere.y;
                }
                else if (distToTop < distToBottom)
                {
                    closestLocal = Vector3(localSphere.x, cylinder->halfHeight, localSphere.z);
                }
                else
                {
                    closestLocal = Vector3(localSphere.x, -cylinder->halfHeight, localSphere.z);
                }
            }
            else
            {
                float r = cylinder->radius;
                Vector3 edge = xzVec * (r / distXZ);
                closestLocal = Vector3(edge.x, clampedY, edge.z);

                if (distXZ < cylinder->radius)
                {
                    closestLocal.x = localSphere.x;
                    closestLocal.z = localSphere.z;
                }
            }
        }
        else
        {
            closestLocal = Vector3(radiusAtAngle(0, cylinder->radius).x, clampedY,
                                   radiusAtAngle(0, cylinder->radius).z);
        }

        // NOTE: This is almost correct, will make it more accurate in the future, peace!
        Vector3 worldClosest = toWorld(cylBody, closestLocal);

        Vector3 diff = sphereBody->position - worldClosest;
        float dist = diff.magnitude();

        if (dist < sphere->radius)
        {
            contact.a = sphereBody;
            contact.b = cylBody;
            contact.penetration = sphere->radius - dist;
            contact.normal = (dist > 0) ? diff * (1.0f / dist) : Vector3(0, 1, 0);
            contact.point = worldClosest;
            return true;
        }
        return false;
    }

    static bool checkCylinderBox(RigidBody* cylBody, RigidBody* boxBody, Contact& contact)
    {
        Cylinder* cylinder = (Cylinder*)cylBody->shape;
        Box* box = (Box*)boxBody->shape;

        float deepestPenetration = -1000.0f;
        Vector3 collisionPoint;
        Vector3 collisionNormal;
        bool hit = false;

        std::vector<Vector3> cylPoints;
        generateCylinderPoints(cylinder, cylPoints);

        for (const auto& localPt : cylPoints)
        {
            Vector3 worldPt = toWorld(cylBody, localPt);
            Vector3 boxLocal = toLocal(boxBody, worldPt);

            if (std::abs(boxLocal.x) < box->halfExtents.x and
                std::abs(boxLocal.y) < box->halfExtents.y and
                std::abs(boxLocal.z) < box->halfExtents.z)
            {
                float dx = box->halfExtents.x - std::abs(boxLocal.x);
                float dy = box->halfExtents.y - std::abs(boxLocal.y);
                float dz = box->halfExtents.z - std::abs(boxLocal.z);

                float pen = std::min(dx, std::min(dy, dz));
                if (pen > deepestPenetration)
                {
                    deepestPenetration = pen;
                    collisionPoint = worldPt;

                    Vector3 localNormal;
                    if (pen == dx)
                    {
                        localNormal = Vector3((boxLocal.x > 0) ? 1 : -1, 0, 0);
                    }
                    else if (pen == dy)
                    {
                        localNormal = Vector3(0, (boxLocal.y > 0) ? 1 : -1, 0);
                    }
                    else
                    {
                        localNormal = Vector3(0, 0, (boxLocal.z > 0) ? 1 : -1);
                    }

                    collisionNormal = boxBody->orientation.rotate(localNormal);
                    hit = true;
                }
            }
        }

        if (hit)
        {
            contact.a = cylBody;
            contact.b = boxBody;
            contact.penetration = deepestPenetration;
            contact.normal = collisionNormal * -1.0f;
            contact.point = collisionPoint;
            return true;
        }
        return false;
    }

    static bool checkCylinderCylinder(RigidBody* a, RigidBody* b, Contact& contact)
    {
        Cylinder* cylA = (Cylinder*)a->shape;
        Cylinder* cylB = (Cylinder*)b->shape;

        float deepestPenetration = -1000.0f;
        Vector3 collisionPoint;
        Vector3 collisionNormal;
        bool hit = false;

        std::vector<Vector3> ptsA;
        generateCylinderPoints(cylA, ptsA);
        for (auto& lp : ptsA)
        {
            Vector3 wp = toWorld(a, lp);
            Vector3 localB = toLocal(b, wp);

            if (std::abs(localB.y) < cylB->halfHeight)
            {
                float distSq = localB.x * localB.x + localB.z * localB.z;
                if (distSq < cylB->radius * cylB->radius)
                {
                    float dist = std::sqrt(distSq);
                    float penR = cylB->radius - dist;
                    float penY = cylB->halfHeight - std::abs(localB.y);

                    float pen = std::min(penR, penY);
                    if (pen > deepestPenetration)
                    {
                        deepestPenetration = pen;
                        collisionPoint = wp;

                        if (pen == penY)
                        {
                            Vector3 ln = Vector3(0, (localB.y > 0) ? 1 : -1, 0);
                            collisionNormal = b->orientation.rotate(ln);
                        }
                        else
                        {
                            Vector3 ln = Vector3(localB.x, 0, localB.z) * (1.0f / dist);
                            collisionNormal = b->orientation.rotate(ln);
                        }
                        hit = true;
                    }
                }
            }
        }

        if (hit)
        {
            contact.a = a;
            contact.b = b;
            contact.penetration = deepestPenetration;
            contact.normal = collisionNormal * -1.0f;
            contact.point = collisionPoint;
            return true;
        }
        return false;
    }
};
