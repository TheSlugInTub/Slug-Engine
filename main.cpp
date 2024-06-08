#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include "stb_image.h"
#include "stb_truetype.h"

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/common.hpp>
#include <glm/gtx/quaternion.hpp>
#include <glm/gtx/extended_min_max.hpp>

#include <SlugEngine.h>
#include <Rigidbody.h>
#include <Collision.h>
#include <Window.h>
#include <ShadowConfiguration.h>
#include <Shader.h>
#include <Skybox.h>
#include <model.h>
#include <AL/al.h>
#include <SoundDevice.h>
#include <SoundBuffer.h>
#include <SoundSource.h>
#include <CameraClass.h>
#include <iostream>

void mouse_callback(GLFWwindow* window, double xpos, double ypos);
void processInput(GLFWwindow* window);

const unsigned int SCR_WIDTH = 1920;
const unsigned int SCR_HEIGHT = 1080;

Camera camera(glm::vec3(0.0f, -2.0f, 10.0f));
float lastX = (float)SCR_WIDTH / 2.0;
float lastY = (float)SCR_HEIGHT / 2.0;
bool firstMouse = true;

float deltaTime = 0.0f;
float lastFrame = 0.0f;

float near_plane = 1.0f;
float far_plane = 25.0f;

glm::vec3 lightPos = glm::vec3(0.0f, 2.0f, 0.0f);

int main()
{
    GLFWwindow* window = initializeWindow("SlugEngine", SCR_WIDTH, SCR_HEIGHT);

    glfwSetCursorPosCallback(window, mouse_callback);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glEnable(GL_MULTISAMPLE);

    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);

    SoundDevice* mySoundDevice = SoundDevice::get();
    uint32_t mySound = SoundBuffer::get()->addSoundEffect("Resources/Flicky.wav");
    SoundSource mySource;

    Shader DefaultShader("Shaders/vertex.shad", "Shaders/fragment.shad");
    Shader ShadowShader("Shaders/shadowvertex.shad", "Shaders/shadowfragment.shad", "Shaders/shadowcalculations.shad");
    Shader FlatShader("Shaders/vertex2d.shad", "Shaders/fragment2d.shad");

    unsigned int popCat = loadTexture("Slugarius.png");
    unsigned int woodTexture = loadTexture("wood.png");

    Model OurModel("Resources/cube.obj");
    Model OurSphere("Resources/ball.obj");

    ShadowMapping shadowMapping(1500, 1500);

    glm::vec3 boxRotation = glm::vec3(0.0f, 0.0f, 0.0f);
    glm::vec3 boxPosition = glm::vec3(0.0f, -5.0f, 0.0f); // Set the initial position
    Rigidbody boxRigidbody(boxPosition, 10000.0f, glm::vec3(0.0f), OurModel, boxRotation);

    glm::vec3 rotation = glm::vec3(0.0f, 0.0f, 0.0f);
    glm::vec3 objectPosition = glm::vec3(0.0f, 20.0f, 0.0f); // Sethe initial position
    Rigidbody rigidbody(objectPosition, 10.0f, glm::vec3(0.0f, -5.8f, 0.0f), 1.0f);

    glm::vec3 rotation2 = glm::vec3(0.0f, 0.0f, 0.0f);
    glm::vec3 objectPosition2 = glm::vec3(5.0f, 20.0f, 0.0f); // Sethe initial position
    Rigidbody rigidbody2(objectPosition2, 10.0f, glm::vec3(0.0f, -5.8f, 0.0f), 1.0f);

    boxRigidbody.setColliderRotation(glm::vec3(0.0f, 0.0f, 0.0f));

    std::vector<Rigidbody> rigidbodies;
    rigidbodies.push_back(boxRigidbody);
    rigidbodies.push_back(rigidbody);
    rigidbodies.push_back(rigidbody2);

    std::vector<Rigidbody> instantiatedSpheres;

    // render loop
    // -----------
    while (!glfwWindowShouldClose(window))
    {
        float currentFrame = static_cast<float>(glfwGetTime());
        deltaTime = currentFrame - lastFrame;
        lastFrame = currentFrame;

        processInput(window);
        glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
        glClear(GL_COLOR_BUFFER_BIT);

        rigidbody.update(deltaTime);
        rigidbody2.update(deltaTime);
        boxRigidbody.update(deltaTime);

        if (glfwGetKey(window, GLFW_KEY_LEFT) == GLFW_PRESS)
        {
            rigidbody.setPosition(glm::vec3(rigidbody.getPosition().x, rigidbody.getPosition().y, rigidbody.getPosition().z + 0.1f));
        }
        if (glfwGetKey(window, GLFW_KEY_RIGHT) == GLFW_PRESS)
        {
            rigidbody.setPosition(glm::vec3(rigidbody.getPosition().x, rigidbody.getPosition().y, rigidbody.getPosition().z - 0.1f));
        }
        if (glfwGetKey(window, GLFW_KEY_UP) == GLFW_PRESS)
        {
            rigidbody.setPosition(glm::vec3(rigidbody.getPosition().x - 0.1f, rigidbody.getPosition().y, rigidbody.getPosition().z));
        }
        if (glfwGetKey(window, GLFW_KEY_DOWN) == GLFW_PRESS)
        {
            rigidbody.setPosition(glm::vec3(rigidbody.getPosition().x + 0.1f, rigidbody.getPosition().y, rigidbody.getPosition().z));
        }

        glm::mat4 model = makeModel(boxRigidbody, glm::vec3(1.0f, 1.0f, 1.0f));
        glm::mat4 model2 = makeModel(rigidbody, glm::vec3(1.0f, 1.0f, 1.0f));
        glm::mat4 model3 = makeModel(rigidbody2, glm::vec3(1.0f, 1.0f, 1.0f));

        // Check for collision only if the distance is less than a certain threshold
        if (CollisionDetector::detectCollisions(rigidbody, boxRigidbody)) {
            ResolveSphereVsBoundingBoxCollision(rigidbody, boxRigidbody);
        }
        if (CollisionDetector::detectCollisions(rigidbody2, boxRigidbody)) {
            ResolveSphereVsBoundingBoxCollision(rigidbody2, boxRigidbody);
        }
        if (CollisionDetector::detectCollisions(rigidbody, rigidbody2)) {
            ResolveSphereCollision(rigidbody, rigidbody2, deltaTime);
        }
        for (size_t i = 0; i < instantiatedSpheres.size(); ++i)
        {
            if (CollisionDetector::detectCollisions(instantiatedSpheres[i], boxRigidbody)) {
                ResolveSphereVsBoundingBoxCollision(instantiatedSpheres[i], boxRigidbody);
            }
            if (CollisionDetector::detectCollisions(instantiatedSpheres[i], rigidbody)) {
                ResolveSphereCollision(instantiatedSpheres[i], rigidbody, deltaTime);
            }
            for (size_t j = i + 1; j < instantiatedSpheres.size(); ++j) {
                if (CollisionDetector::detectCollisions(instantiatedSpheres[i], instantiatedSpheres[j])) {
                    ResolveSphereCollision(instantiatedSpheres[i], instantiatedSpheres[j], deltaTime);
                }
            }
        }

        std::vector<std::pair<Model, glm::mat4>> models = {
            {OurSphere, model2},
            {OurSphere, model3}
        }; 

        shadowMapping.CreateDepthCubemap(lightPos, near_plane, far_plane);

        shadowMapping.RenderDepthCubemap(ShadowShader, models);

        glViewport(0, 0, SCR_WIDTH, SCR_HEIGHT);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        DefaultShader.use();
        glm::mat4 projection = glm::perspective(glm::radians(camera.Zoom), (float)SCR_WIDTH / (float)SCR_HEIGHT, 0.1f, 100.0f);
        glm::mat4 view = camera.GetViewMatrix();
        DefaultShader.setMat4("projection", projection);
        DefaultShader.setMat4("view", view);
        // set lighting uniforms
        DefaultShader.setVec3("lightPos", lightPos);
        DefaultShader.setVec3("viewPos", camera.Position);
        DefaultShader.setInt("shadows", true);
        DefaultShader.setFloat("far_plane", far_plane);
        DefaultShader.setFloat("lightIntensity", 1.5f);

        renderObject(OurModel, model, DefaultShader, woodTexture);
        renderObject(OurSphere, model2, DefaultShader, popCat);
        renderObject(OurSphere, model3, DefaultShader, popCat);
        for (size_t i = 0; i < instantiatedSpheres.size(); ++i) {

            glm::mat4 model = makeModel(instantiatedSpheres[i], glm::vec3(1.0f, 1.0f, 1.0f));
            renderObject(OurSphere, model, DefaultShader, popCat);
            instantiatedSpheres[i].update(deltaTime);
        }

        if (GetKeyDown(window, GLFW_KEY_U))
        {
            mySource.Play(mySound);
            Rigidbody newRigidbody(glm::vec3(0.0f, 20.0f, 0.0f), 10.f, glm::vec3(0.0f, -5.8f, 0.0f), 1.0f);
            instantiatedSpheres.push_back(newRigidbody);
        }

        RunProgram(window);
    }

    DefaultShader.deuse();
    ShadowShader.deuse();
    EndProgram();
    return 0;
}

void processInput(GLFWwindow* window)
{
    if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS)
        camera.ProcessKeyboard(FORWARD, deltaTime);
    if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS)
        camera.ProcessKeyboard(BACKWARD, deltaTime);
    if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS)
        camera.ProcessKeyboard(LEFT, deltaTime);
    if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS)
        camera.ProcessKeyboard(RIGHT, deltaTime);
    if (GetKeyDown(window, GLFW_KEY_ESCAPE))
        glfwSetWindowShouldClose(window, true);
}

void mouse_callback(GLFWwindow* window, double xposIn, double yposIn)
{
    float xpos = static_cast<float>(xposIn);
    float ypos = static_cast<float>(yposIn);
    if (firstMouse)
    {
        lastX = xpos;
        lastY = ypos;
        firstMouse = false;
    }

    float xoffset = xpos - lastX;
    float yoffset = lastY - ypos; // reversed since y-coordinates go from bottom to top

    lastX = xpos;
    lastY = ypos;

    camera.ProcessMouseMovement(xoffset, yoffset);
}