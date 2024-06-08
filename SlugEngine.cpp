#define GLM_ENABLE_EXPERIMENTAL
#include <SlugEngine.h>

glm::mat4 makeModel(Rigidbody& rigidbody, glm::vec3 scale)
{
    glm::mat4 model = glm::mat4(1.0f);

    // Translate to the rigidbody's position
    model = glm::translate(model, rigidbody.getPosition());

    // Apply rotations around the x, y, and z axes
    model = glm::rotate(model, glm::radians(rigidbody.getRotation().x), glm::vec3(0, 0, 1));
    model = glm::rotate(model, glm::radians(rigidbody.getRotation().y), glm::vec3(0, 1, 0));
    model = glm::rotate(model, glm::radians(rigidbody.getRotation().z), glm::vec3(1, 0, 0));

    // Apply scaling
    model = glm::scale(model, scale);

    return model;
}

bool GetKeyDown(GLFWwindow* window, int key) {
    static std::map<int, bool> keyState;
    static std::map<int, bool> keyStatePrev;

    keyStatePrev[key] = keyState[key];
    keyState[key] = glfwGetKey(window, key) == GLFW_PRESS;

    return keyState[key] && !keyStatePrev[key];
}

bool GetKey(GLFWwindow* window, int key)
{
    if (glfwGetKey(window, key) == GLFW_PRESS)
    {
        return true;
    }
}

void EndProgram()
{
    glfwTerminate();
}

void RunProgram(GLFWwindow* window)
{
    glfwSwapBuffers(window);
    glfwPollEvents();
}

unsigned int loadTexture(char const* path)
{
    unsigned int textureID;
    glGenTextures(1, &textureID);

    int width, height, nrComponents;
    unsigned char* data = stbi_load(path, &width, &height, &nrComponents, 0);
    if (data)
    {
        GLenum format;
        if (nrComponents == 1)
            format = GL_RED;
        else if (nrComponents == 3)
            format = GL_RGB;
        else if (nrComponents == 4)
            format = GL_RGBA;

        glBindTexture(GL_TEXTURE_2D, textureID);
        glTexImage2D(GL_TEXTURE_2D, 0, format, width, height, 0, format, GL_UNSIGNED_BYTE, data);
        glGenerateMipmap(GL_TEXTURE_2D);

        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, format == GL_RGBA ? GL_CLAMP_TO_EDGE : GL_REPEAT); // for this tutorial: use GL_CLAMP_TO_EDGE to prevent semi-transparent borders. Due to interpolation it takes texels from next repeat 
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, format == GL_RGBA ? GL_CLAMP_TO_EDGE : GL_REPEAT);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

        glBindTexture(GL_TEXTURE_2D, 0);
        stbi_image_free(data);
    }
    else
    {
        std::cout << "Texture failed to load at path: " << path << std::endl;
        stbi_image_free(data);
    }

    return textureID;
}

float calculatePenetrationDepth(const Rigidbody& rb1, const Rigidbody& rb2) {
    // Get the bounding box dimensions of both rigidbodies
    glm::vec3 minA = rb1.getPosition() + rb1.getBoundingBoxMin();
    glm::vec3 maxA = rb1.getPosition() + rb1.getBoundingBoxMax();
    glm::vec3 minB = rb2.getPosition() + rb2.getBoundingBoxMin();
    glm::vec3 maxB = rb2.getPosition() + rb2.getBoundingBoxMax();

    // Calculate the penetration depth in each axis
    float penetrationX = std::min(maxA.x - minB.x, maxB.x - minA.x);
    float penetrationY = std::min(maxA.y - minB.y, maxB.y - minA.y);
    float penetrationZ = std::min(maxA.z - minB.z, maxB.z - minA.z);

    // Find the minimum penetration depth among the axes
    float minPenetrationDepth = std::min(std::min(penetrationX, penetrationY), penetrationZ);

    // Return the minimum penetration depth
    return minPenetrationDepth;
}

void renderObject(Model OurModel, glm::mat4 model, Shader DefaultShader, unsigned int path)
{
    DefaultShader.setMat4("model", model);
    DefaultShader.setTexture2D("diffuseTexture", path, 0);
    OurModel.Draw(DefaultShader);
}

void ResolveCollisions(Rigidbody& rb1, Rigidbody& rb2)
{
    glm::vec3 collisionNormal = glm::normalize(rb2.getPosition() - rb1.getPosition());

    // Relative velocity
    glm::vec3 relativeVelocity = rb2.getVelocity() - rb1.getVelocity();

    // Calculate relative velocity in terms of the normal direction
    float velocityAlongNormal = glm::dot(relativeVelocity, collisionNormal);

    // Do not resolve if velocities are separating
    if (velocityAlongNormal > 0) {
        return;
    }

    // Calculate restitution (elasticity of the collision)
    float restitution = 0.1f;
    float e = restitution;

    // Calculate impulse scalar
    float j = -(1 + e) * velocityAlongNormal;
    j /= (1 / rb1.getMass() + 1 / rb2.getMass());

    // Apply impulse
    glm::vec3 impulse = j * collisionNormal;
    rb1.setVelocity(rb1.getVelocity() / rb1.getMass());
    rb2.setVelocity(rb2.getVelocity() / rb2.getMass());

    // Position correction to avoid sinking
    const float percent = 0.2f; // usually 20% to 80%
    const float slop = 0.01f; // usually 0.01 to 0.1
    glm::vec3 correction = std::max(glm::length(rb2.getPosition() - rb1.getPosition()) - slop, 0.0f) / (1 / rb1.getMass() + 1 / rb2.getMass()) * percent * collisionNormal;
    rb1.setPosition(rb1.getPosition() - correction / rb1.getMass());
    rb2.setPosition(rb2.getPosition() + correction / rb2.getMass());
}

void ResolveCollisionIgnoreFirst(Rigidbody& boxRigidbody, Rigidbody& rigidbody)
{
    float distance = glm::distance(rigidbody.getPosition(), boxRigidbody.getPosition());
    std::cout << distance;

    // Calculate minimum penetration depth based on combined radius/bounding box half size
    float minimumPenetrationDepth = rigidbody.getMass() + boxRigidbody.getMass();

    if (distance - minimumPenetrationDepth <= 0.01f)
    {
        glm::vec3 collisionNormal = glm::normalize(rigidbody.getPosition() - boxRigidbody.getPosition());

        // Calculate relative velocity
        glm::vec3 relativeVelocity = rigidbody.getVelocity() - boxRigidbody.getVelocity();
        float relativeVelocityNormal = glm::dot(relativeVelocity, collisionNormal);

        float restitution = 0.1f; // Adjust the coefficient as needed

        // Calculate impulse magnitude for normal direction
        float j = -(1 + restitution) * relativeVelocityNormal;
        j /= 1 / rigidbody.getMass() + 1 / boxRigidbody.getMass();

        // Apply impulse for normal direction
        glm::vec3 impulse = j * collisionNormal;

        // Update velocities for normal direction
        rigidbody.setVelocity(rigidbody.getVelocity() / rigidbody.getMass());

        // Resolve penetration
        (rigidbody.getMass() + boxRigidbody.getMass()); // Use combined mass for center of mass calculation
        const float percent = 0.2f; // Penetration percentage to correct
        const float slop = 0.5f; // Allowance to prevent jittering
        float penetrationDepth = calculatePenetrationDepth(rigidbody, boxRigidbody);
        glm::vec3 desiredDistance =
            0.5f * (rigidbody.getBoundingBoxMax() - rigidbody.getBoundingBoxMin()) +
            0.5f * (boxRigidbody.getBoundingBoxMax() - boxRigidbody.getBoundingBoxMin());; // Calculate desired non-penetration distance (e.g., sum of bounding box half sizes)
        float desiredDistanceMagnitude = glm::length(desiredDistance);
        float penetrationDepthBruh = desiredDistanceMagnitude - distance;

        if (penetrationDepthBruh > slop) {
            glm::vec3 correction = penetrationDepth * collisionNormal;
            rigidbody.setPosition(rigidbody.getPosition() + correction);
        }

        // Calculate relative velocity in the direction of the tangent (friction)
        glm::vec3 relativeVelocityTangent = relativeVelocity - (glm::dot(relativeVelocity, collisionNormal) * collisionNormal);
        float relativeVelocityTangentMagnitude = glm::length(relativeVelocityTangent);

        // Calculate friction coefficient
        float staticFrictionThreshold = 0.001f;
        float frictionCoefficient = 0.1f;

        // Apply friction impulse if there's relative tangential velocity
        if (relativeVelocityTangentMagnitude < staticFrictionThreshold) {
            // If relative tangential velocity is low, apply static friction to prevent sliding
            // Calculate static friction impulse
            glm::vec3 staticFrictionImpulseA = -relativeVelocityTangent * rigidbody.getMass(); // Opposes motion
            glm::vec3 staticFrictionImpulseB = -relativeVelocityTangent * boxRigidbody.getMass(); // Opposes motion

            // Apply static friction impulse
            rigidbody.setVelocity(rigidbody.getVelocity() + staticFrictionImpulseA / rigidbody.getMass());
        }
        else {
            // If relative tangential velocity is high, apply dynamic friction
            // Calculate friction coefficient
            // Apply friction impulse if there's relative tangential velocity
            // Calculate impulse magnitude for friction
            float frictionImpulseMagnitude = 0.1f * j;

            // Clamp friction impulse magnitude to prevent reversal of relative motion
            frictionImpulseMagnitude = std::min(frictionImpulseMagnitude, relativeVelocityTangentMagnitude);

            // Calculate friction impulse vector
            glm::vec3 frictionImpulse = glm::normalize(relativeVelocityTangent) * frictionImpulseMagnitude;

            // Apply friction impulse
            rigidbody.setVelocity(rigidbody.getVelocity() - frictionImpulse);
        }

        // Calculate angular velocity change due to collision
        glm::vec3 rA = rigidbody.getPosition() - boxRigidbody.getPosition();
        glm::vec3 rB = boxRigidbody.getPosition() - rigidbody.getPosition();
        glm::vec3 angularVelocityChangeA = glm::cross(rA, impulse) / rigidbody.getMass();
        glm::vec3 angularVelocityChangeB = glm::cross(rB, -impulse) / boxRigidbody.getMass();

        // Apply angular velocity change
        rigidbody.setRotation(rigidbody.getRotation() + angularVelocityChangeA);
    }
}

void ResolveSphereCollision(Rigidbody& sphere1, Rigidbody& sphere2, float deltaTime) {

    glm::vec3 pos1 = sphere1.getPosition();
    glm::vec3 pos2 = sphere2.getPosition();
    float radius1 = sphere1.getRadius();
    float radius2 = sphere2.getRadius();

    glm::vec3 collisionNormal = pos2 - pos1;
    float distance = glm::length(collisionNormal);
    float combinedRadii = radius1 + radius2;

    // Check for collision
    if (distance < combinedRadii) {
        // Normalize the collision normal
        collisionNormal = glm::normalize(collisionNormal);

        // Calculate relative velocity
        glm::vec3 relativeVelocity = sphere1.getVelocity() - sphere2.getVelocity();
        float relativeVelocityNormal = glm::dot(relativeVelocity, collisionNormal);

        // Calculate restitution (bounciness)
        float restitution = 0.5f;

        // Calculate impulse scalar
        float j = -(1 + restitution) * relativeVelocityNormal;
        j /= (1 / sphere1.getMass()) + (1 / sphere2.getMass());

        // Calculate impulse vector
        glm::vec3 impulse = j * collisionNormal;

        // Apply impulse to the velocities
        sphere1.setVelocity(sphere1.getVelocity() + impulse / sphere1.getMass());
        sphere2.setVelocity(sphere2.getVelocity() - impulse / sphere2.getMass());

        // Position correction
        float penetrationDepth = combinedRadii - distance;
        glm::vec3 correction = (penetrationDepth / ((1 / sphere1.getMass()) + (1 / sphere2.getMass()))) * collisionNormal;

        sphere1.setPosition(pos1 - correction * (1 / sphere1.getMass()));
        sphere2.setPosition(pos2 + correction * (1 / sphere2.getMass()));

        glm::vec3 contactPoint = pos1 + collisionNormal * radius1;
        glm::vec3 r1 = contactPoint - pos1;
        glm::vec3 r2 = contactPoint - pos2;

        glm::vec3 angularChange1 = r1 / sphere1.getMass();
        glm::vec3 angularChange2 = r2 / sphere2.getMass();

        sphere1.setAngularVelocity(sphere1.getAngularVelocity() + angularChange1 * 3.f);
        sphere2.setAngularVelocity(sphere2.getAngularVelocity() - angularChange2 * 3.f);

        // Apply torque to rotation
        //sphere2.setAngularVelocity(sphere2.getRotation() + r2);
        //sphere1.setAngularVelocity(sphere1.getRotation() + r1);
    }
}


void ResolveSphereVsBoundingBoxCollision(Rigidbody& sphere, Rigidbody& box) {
    glm::vec3 spherePos = sphere.getPosition();
    glm::vec3 boxMin = box.getBoundingBoxMin() + box.getPosition();
    glm::vec3 boxMax = box.getBoundingBoxMax() + box.getPosition();

    glm::vec3 closestPoint = glm::clamp(spherePos, boxMin, boxMax);
    float distance = glm::distance(spherePos, closestPoint);

    if (distance < sphere.getRadius()) {
        glm::vec3 collisionNormal = glm::normalize(spherePos - closestPoint);
        glm::vec3 relativeVelocity = sphere.getVelocity() - box.getVelocity();
        float relativeVelocityNormal = glm::dot(relativeVelocity, collisionNormal);

        float restitution = 0.1f;

        float j = -(1 + restitution) * relativeVelocityNormal;
        j /= 1 / sphere.getMass() + 1 / box.getMass();

        glm::vec3 impulse = j * collisionNormal;

        sphere.setVelocity(sphere.getVelocity() + impulse / sphere.getMass());

        float penetrationDepth = sphere.getRadius() - distance;
        glm::vec3 correction = penetrationDepth * collisionNormal;
        sphere.setPosition(sphere.getPosition() + correction * (box.getMass() / (sphere.getMass() + box.getMass())));

        // Calculate friction impulse
        glm::vec3 tangent = relativeVelocity - relativeVelocityNormal * collisionNormal;
        if (glm::length(tangent) > 0.0001f) {
            tangent = glm::normalize(tangent);
        }

        float relativeVelocityTangent = glm::dot(relativeVelocity, tangent);
        float frictionCoefficient = 0.05f; // This can be adjusted based on material properties
        float jt = -relativeVelocityTangent;
        jt /= 1 / sphere.getMass() + 1 / box.getMass();
        jt = glm::clamp(jt, -j * frictionCoefficient, j * frictionCoefficient);

        glm::vec3 frictionImpulse = jt * tangent;
        sphere.setVelocity(sphere.getVelocity() + frictionImpulse / sphere.getMass());
    }
}
