#include <emscripten/bind.h>
#include <emscripten/emscripten.h>
#include <vector>

using namespace emscripten;

struct Vector3 {
    float x;
    float y;
    float z;
};

class Particle {
public:
    Vector3 position;
    Vector3 velocity;

    Particle(float x, float y, float z) {
        this->position.x = x;
        this->position.y = y;
        this->position.z = z;

        this->velocity.x = 0;
        this->velocity.y = 0;
        this->velocity.z = 0;
    }

    val getPosition() const {
        val obj = val::object();
        obj.set("x", position.x);
        obj.set("y", position.y);
        obj.set("z", position.z);
        return obj;
    }
};

class PhysicsWorld {
    std::vector<Particle> particles;
    float gravity = -9.81;
    float restitution = 0.5;

public:
    PhysicsWorld() {}

    void setGravity(float g) {
        gravity = g;
    }

    void setRestitution(float r) {
        restitution = r;
    }

    void reset() {
        particles.clear();
    }

    void addParticle(float x, float y, float z) {
        particles.push_back(Particle(x, y, z));
    }

    void step(float dt) {
        for (auto &particle : particles) {
            // NOTE: this is a simple semi-implicit euler integration
            particle.velocity.y += gravity * dt;

            particle.position.x += particle.velocity.x * dt;
            particle.position.y += particle.velocity.y * dt;
            particle.position.z += particle.velocity.z * dt;
            
            // NOTE: velocity going less than zero is just floor collision.
            // Assuming the particle represents a cube of height 1, half-height is 0.5
            if (particle.position.y < 0.5) {
                particle.position.y = 0.5;
                particle.velocity.y *= -restitution;
            }
        }
    }

    val getParticlePosition(int index) {
        if (index >= 0 and index < particles.size()) {
            return particles[index].getPosition();
        }
        return val::null();
    }

    int getParticleCount() {
        return particles.size();
    }
};

EMSCRIPTEN_BINDINGS(applicable_physics_engine) {
    class_<PhysicsWorld>("PhysicsWorld")
        .constructor<>()
        .function("setGravity", &PhysicsWorld::setGravity)
        .function("setRestitution", &PhysicsWorld::setRestitution)
        .function("reset", &PhysicsWorld::reset)
        .function("addParticle", &PhysicsWorld::addParticle)
        .function("step", &PhysicsWorld::step)
        .function("getParticlePosition", &PhysicsWorld::getParticlePosition)
        .function("getParticleCount", &PhysicsWorld::getParticleCount);
}
