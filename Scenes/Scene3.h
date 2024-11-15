#include "Scene.h"

class Scene3 : public Scene{
    struct Point
    {
        glm::vec3 position;
        glm::vec3 velocity;
        float radius;
        glm::vec4 color;
        float mass;
    };

    Point point1 = {glm::vec3(0), glm::vec3(-1, 0, 0), .1f, glm::vec4(1, 0, 0, 1), 10};
    Point point2 = {glm::vec3(0, 2, 0), glm::vec3(1, 0, 0), .1f,glm::vec4(0, 1, 0, 1), 10};

    float timeStep = 0.005f;
    float K = 40;
    float L = 1;

    virtual void onDraw(Renderer &renderer) override;
    virtual void simulateStep() override;
    virtual void onGUI() override;
    
    glm::vec3 Scene3::computeSpringForce(glm::vec3 p1, glm::vec3 p2, float len);
};