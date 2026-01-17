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
        bodies.push_back(body);
    }

    void addBox(float x, float y, float z, float w, float h, float d, float mass) {
        Box* boxShape = new Box(w, h, d);
        RigidBody* body = new RigidBody(boxShape, x, y, z, mass);
        body->restitution = 0.5f;
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
        for (auto body : bodies) {
            if (body->hasFiniteMass()) {
                body->velocity += gravity * dt;
            }

            body->integrate(dt);
        }

        for (auto body : bodies) {
            Contact contact;
            bool collided = false;

            if (body->shape->type == SPHERE) {
                collided = CollisionDetector::checkSpherePlane(body, 0.0f, contact);
            } else if (body->shape->type == BOX) {
                collided = CollisionDetector::checkBoxPlane(body, 0.0f, contact);
            }

            if (collided) {
                ContactResolver::resolve(contact);
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

};

EMSCRIPTEN_BINDINGS(applicable_physics_engine) {
    class_<PhysicsWorld>("PhysicsWorld")
        .constructor<>()
        .function("addSphere", &PhysicsWorld::addSphere)
        .function("addBox", &PhysicsWorld::addBox)
        .function("setGravity", &PhysicsWorld::setGravity)
        .function("setRestitution", &PhysicsWorld::setRestitution)
        .function("step", &PhysicsWorld::step)
        .function("reset", &PhysicsWorld::reset)
        .function("getBodyCount", &PhysicsWorld::getBodyCount)
        .function("getBodyPosition", &PhysicsWorld::getBodyPosition);
}
