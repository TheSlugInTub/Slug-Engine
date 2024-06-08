#ifndef COLLISION_CLASS_H
#define COLLISION_CLASS_H

#include <Rigidbody.h>

class CollisionDetector {
public:
    static bool detectBoundingBoxCollision(Rigidbody& rb1, Rigidbody& rb2) {
        glm::vec3 minA = rb1.getPosition() + rb1.getBoundingBoxMin();
        glm::vec3 maxA = rb1.getPosition() + rb1.getBoundingBoxMax();
        glm::vec3 minB = rb2.getPosition() + rb2.getBoundingBoxMin();
        glm::vec3 maxB = rb2.getPosition() + rb2.getBoundingBoxMax();

        return (maxA.x >= minB.x && minA.x <= maxB.x &&
            maxA.y >= minB.y && minA.y <= maxB.y &&
            maxA.z >= minB.z && minA.z <= maxB.z);
    }

    // Detect sphere vs. sphere collision
    static bool detectSphereCollision(Rigidbody& rb1, Rigidbody& rb2) {
        float distanceSquared = glm::distance2(rb1.getPosition(), rb2.getPosition());
        float combinedRadius = rb1.getRadius() + rb2.getRadius();
        return distanceSquared <= (combinedRadius * combinedRadius);
    }

    // Detect sphere vs. bounding box collision
    static bool detectSphereBoundingBoxCollision(Rigidbody& sphere, Rigidbody& box) {
        glm::vec3 boxMin = box.getPosition() + box.getBoundingBoxMin();
        glm::vec3 boxMax = box.getPosition() + box.getBoundingBoxMax();
        glm::vec3 sphereCenter = sphere.getPosition();
        float sphereRadius = sphere.getRadius();

        float distanceSquared = sphereRadius * sphereRadius;

        if (sphereCenter.x < boxMin.x) distanceSquared -= (sphereCenter.x - boxMin.x) * (sphereCenter.x - boxMin.x);
        else if (sphereCenter.x > boxMax.x) distanceSquared -= (sphereCenter.x - boxMax.x) * (sphereCenter.x - boxMax.x);

        if (sphereCenter.y < boxMin.y) distanceSquared -= (sphereCenter.y - boxMin.y) * (sphereCenter.y - boxMin.y);
        else if (sphereCenter.y > boxMax.y) distanceSquared -= (sphereCenter.y - boxMax.y) * (sphereCenter.y - boxMax.y);

        if (sphereCenter.z < boxMin.z) distanceSquared -= (sphereCenter.z - boxMin.z) * (sphereCenter.z - boxMin.z);
        else if (sphereCenter.z > boxMax.z) distanceSquared -= (sphereCenter.z - boxMax.z) * (sphereCenter.z - boxMax.z);

        return distanceSquared > 0;
    }

    // Overloaded collision detection method to handle different types of colliders
    static bool detectCollisions(Rigidbody& rb1, Rigidbody& rb2) {
        if (rb1.colliderType == Rigidbody::ColliderType::BoundingBox && rb2.colliderType == Rigidbody::ColliderType::BoundingBox) {
            return detectBoundingBoxCollision(rb1, rb2);
        }
        else if (rb1.colliderType == Rigidbody::ColliderType::Sphere && rb2.colliderType == Rigidbody::ColliderType::Sphere) {
            return detectSphereCollision(rb1, rb2);
        }
        else if (rb1.colliderType == Rigidbody::ColliderType::Sphere && rb2.colliderType == Rigidbody::ColliderType::BoundingBox) {
            return detectSphereBoundingBoxCollision(rb1, rb2);
        }
        else if (rb1.colliderType == Rigidbody::ColliderType::BoundingBox && rb2.colliderType == Rigidbody::ColliderType::Sphere) {
            return detectSphereBoundingBoxCollision(rb2, rb1);
        }
        return false;
    }
};

#endif