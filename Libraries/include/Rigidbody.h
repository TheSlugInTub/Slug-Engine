#ifndef RIGIDBODY_CLASS_H
#define RIGIDBODY_CLASS_H

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/rotate_vector.hpp>
#include <model.h>

class Rigidbody {
private:
    glm::vec3 position;
    glm::vec3 velocity;
    glm::vec3 acceleration;
    glm::vec3 force;
    float mass;
    glm::vec3 gravity;
    glm::vec3 rotation;
    glm::vec3 angularVelocity;
    glm::mat3 inertiaTensor; 

    // Bounding box dimensions
    glm::vec3 boundingBoxMin;
    glm::vec3 boundingBoxMax;
    glm::vec3 colliderRotation;

    float radius;

public:
    Rigidbody(glm::vec3 initialPosition, float initialMass, glm::vec3 initialGravity,
        Model& model, glm::vec3 initialRotation)
        : position(initialPosition), mass(initialMass), velocity(glm::vec3(0.0f)),
        acceleration(glm::vec3(0.0f)), force(glm::vec3(0.0f)), gravity(initialGravity),
        rotation(initialRotation), angularVelocity(glm::vec3(0.0f)), // Initialize angular velocity
        inertiaTensor(glm::mat3(1.0f)), // Initialize inertia tensor
        colliderType(ColliderType::BoundingBox) {
        // Get bounding box from the model
        boundingBoxMax = model.GetMaxBoundingBox();
        boundingBoxMin = model.GetMinBoundingBox();
    }

    // Constructor for spherical collider
    Rigidbody(glm::vec3 initialPosition, float initialMass, glm::vec3 initialGravity,
        float initialRadius)
        : position(initialPosition), mass(initialMass), velocity(glm::vec3(0.0f)),
        acceleration(glm::vec3(0.0f)), force(glm::vec3(0.0f)), gravity(initialGravity),
        rotation(glm::vec3(0.0f)), angularVelocity(glm::vec3(0.0f)), // Initialize angular velocity
        inertiaTensor((2.0f / 5.0f)* initialMass* initialRadius* initialRadius* glm::mat3(1.0f)), // Spherical inertia tensor
        radius(initialRadius), colliderType(ColliderType::Sphere) {}

    void applyForce(glm::vec3 externalForce) {
        force += externalForce;
    }

    void update(float deltaTime) {
        // Apply gravity
        force += mass * gravity;

        // Apply Newton's second law: F = ma
        acceleration = force / mass;

        // Update velocity
        velocity += acceleration * deltaTime;

        // Update position
        position += velocity * deltaTime;

        // Update rotation based on angular velocity
        angularVelocity *= 0.8f;
        velocity *= 0.98f;

        rotation += angularVelocity;

        // Reset force for next update
        force = glm::vec3(0.0f);
    }

    glm::vec3 getPosition() const {
        return position;
    }

    glm::vec3 getVelocity() const {
        return velocity;
    }

    glm::vec3 getAngularVelocity() const {
        return angularVelocity;
    }

    glm::mat3 getInertia()
    {
        return inertiaTensor;
    }

    void setPosition(glm::vec3 newPosition) {
        position = newPosition;
    }

    void setVelocity(glm::vec3 newVelocity) {
        velocity = newVelocity;
    }

    void setAngularVelocity(glm::vec3 newAngularVelocity) {
        angularVelocity = newAngularVelocity;
    }

    float getMass() const {
        return mass;
    }

    void setMass(float newMass) {
        mass = newMass;
    }

    glm::vec3 getGravity() const {
        return gravity;
    }

    void setGravity(glm::vec3 newGravity) {
        gravity = newGravity;
    }

    glm::vec3 getRotation() const {
        return rotation;
    }

    void setRotation(glm::vec3 newRotation) {
        rotation = newRotation;
    }

    void setColliderRotation(const glm::vec3& newRotation) {
        colliderRotation = newRotation;
    }

    float getRadius() {
        return radius;
    }

    enum class ColliderType {
        BoundingBox,
        Sphere
    } colliderType;

    // Methods to get the bounding box dimensions
    glm::vec3 getBoundingBoxMin() const {
        // Apply rotation to the min bounding box coordinates
        glm::vec3 rotatedMin = boundingBoxMin;
        rotatedMin = glm::rotateX(rotatedMin, colliderRotation.x);
        rotatedMin = glm::rotateY(rotatedMin, colliderRotation.y);
        rotatedMin = glm::rotateZ(rotatedMin, colliderRotation.z);
        return rotatedMin;
    }

    glm::vec3 getBoundingBoxMax() const {
        // Apply rotation to the max bounding box coordinates
        glm::vec3 rotatedMax = boundingBoxMax;
        rotatedMax = glm::rotateX(rotatedMax, colliderRotation.x);
        rotatedMax = glm::rotateY(rotatedMax, colliderRotation.y);
        rotatedMax = glm::rotateZ(rotatedMax, colliderRotation.z);
        return rotatedMax;
    }

    // Bounding box collision detection
    bool checkCollision(const Rigidbody& other) const {
        // Get the bounding box of the other Rigidbody
        glm::vec3 minB = other.getPosition() + other.getBoundingBoxMin();
        glm::vec3 maxB = other.getPosition() + other.getBoundingBoxMax();

        // Check if this bounding box intersects with the other bounding box
        return (position.x + boundingBoxMax.x >= minB.x &&
            position.x + boundingBoxMin.x <= maxB.x &&
            position.y + boundingBoxMax.y >= minB.y &&
            position.y + boundingBoxMin.y <= maxB.y &&
            position.z + boundingBoxMax.z >= minB.z &&
            position.z + boundingBoxMin.z <= maxB.z);
    }
};

#endif
