#include <emscripten/bind.h>
#include <emscripten/emscripten.h>
#include <vector>
#include "core/Vector3.h"
#include "core/RigidBody.h"
#include "geometry/Sphere.h"
#include "core/CollisionDetector.h"
#include "core/ContactResolver.h"
#include "geometry/Box.h"

using namespace emscripten;

class PhysicsWorld {
    std::vector<RigidBody*> bodies;
    Vector3 gravity = Vector3(0, -9.81f, 0);
   
public:
    PhysicsWorld() {}

    ~PhysicsWorld() {
        for (auto body : bodies) {
            delete body->shape;
            delete body;
        }
    }

    void addSphere(float x, float y, float z, float radius, float mass) {
        Sphere* sphereShape = new Sphere(radius);
        RigidBody* body = new RigidBody(sphereShape, x, y, z, mass);
        body->friction = 0.5f;
        bodies.push_back(body);
    }

    void addBox(float x, float y, float z, float w, float h, float d, float mass) {
        Box* boxShape = new Box(w, h, d);
        RigidBody* body = new RigidBody(boxShape, x, y, z, mass);
        body->restitution = 0.5f;
        body->friction = 0.5f;
        bodies.push_back(body);
    }

    void setGravity(float gy) {
        gravity.y = gy;
    }

    void setRestitution(float r) {
        for (auto body : bodies) {
            body->restitution = r;
        }
    }

    void reset() {
        for (auto body : bodies) {
            delete body->shape;
            delete body;
        }
        bodies.clear();
    }

    void step(float dt) {
        updateInertiaTensors();
        for (auto body : bodies) {
            if (body->hasFiniteMass()) {
                body->velocity += gravity * dt;
                body->integrate(dt);
            }
        }

        for (auto body : bodies) {
            Contact contact;
            bool hitFloor = false;

            if (body->shape->type == SPHERE) {
                hitFloor = CollisionDetector::checkSpherePlane(body, 0.0f, contact);
            } else if (body->shape->type == BOX) {
                hitFloor = CollisionDetector::checkBoxPlane(body, 0.0f, contact);
            }

            if (hitFloor) {
                ContactResolver::resolve(contact);
            }
        }

        for (size_t i = 0; i < bodies.size(); i++) {
            for (size_t j = i + 1; j < bodies.size(); j++) {
                RigidBody* bodyA = bodies[i];
                RigidBody* bodyB = bodies[j];
                
                Contact contact;
                bool collided = false;

                if (bodyA->shape->type == SPHERE and bodyB->shape->type == SPHERE) {
                    collided = CollisionDetector::checkSphereSphere(bodyA, bodyB, contact);
                } else if (bodyA->shape->type == BOX and bodyB->shape->type == BOX) {
                    collided = CollisionDetector::checkBoxBox(bodyA, bodyB, contact);
                } else if (bodyA->shape->type == BOX and bodyB->shape->type == SPHERE) {
                    collided = CollisionDetector::checkBoxSphere(bodyA, bodyB, contact);
                } else if (bodyA->shape->type == SPHERE and bodyB->shape->type == BOX) {
                    collided = CollisionDetector::checkSphereBox(bodyA, bodyB, contact);
                }

                if (collided) {
                    ContactResolver::resolve(contact);
                }
            }
        }
    }

    val getBodyPosition(int index) {
        if (index >= 0 and index < bodies.size()) {
            return bodies[index]->toJs();
        }
        return val::null();
    }

    int getBodyCount() {
        return bodies.size(); 
    }

    void setFriction(float f) {
        for (auto body : bodies) {
            body->friction = f;
        }
    }

    void updateInertiaTensors() {
        for (auto body : bodies) {
            if (body->inverseMass  > 0) {
                body->inverseInertiaTensorWorld = body->inverseInertiaTensor;
            }
        }
    }

};

EMSCRIPTEN_BINDINGS(applicable_physics_engine) {
    class_<PhysicsWorld>("PhysicsWorld")
        .constructor<>()
        .function("addSphere", &PhysicsWorld::addSphere)
        .function("addBox", &PhysicsWorld::addBox)
        .function("setGravity", &PhysicsWorld::setGravity)
        .function("setRestitution", &PhysicsWorld::setRestitution)
        .function("step", &PhysicsWorld::step)
        .function("setFriction", &PhysicsWorld::setFriction)
        .function("reset", &PhysicsWorld::reset)
        .function("getBodyCount", &PhysicsWorld::getBodyCount)
        .function("getBodyPosition", &PhysicsWorld::getBodyPosition);
}
