#include "Scene.h"

class Scene3 : public Scene
{
    using vec3 = glm::vec3;
    using vec4 = glm::vec4;
    using quat = glm::quat;
    using mat3 = glm::mat3;
    using mat4 = glm::mat4;

    struct Body
    {
        vec3 pos;
        quat ori;
        vec3 extent;
        vec3 lin_vec;
        vec3 ang_vec;
        mat3 inver_mo_inertia; // inverse of moment of inertia I^(-1)
        float mass;
        vec3 mo_ang; // momentum of angular L
        vec4 color;
    };

    Body body1 = {vec3(0, -3, 0), 
    quat(vec3(0, glm::radians(45.f), glm::radians(45.f))), vec3(1), 
    vec3(0, 0.1f, 0), vec3(0), mat3(1), 6.f, vec3(0), vec4(1, 0, 0, 1)};
    
    Body body2 = {vec3(0, 3, 0), quat(vec3(0, 0, 0)), vec3(2, 1, 1), 
    vec3(0), vec3(0), mat3(vec3(0.5f, 0, 0), vec3(0, 0.2f, 0), vec3(0, 0, 0.2f)), 
    12.f, vec3(0, 0, 0), vec4(0, 1, 0, 1)};

    float timeStep = 0.01f;
    float c = 1; // coefficient of restitution

    virtual void init() override;
    virtual void onDraw(Renderer &renderer) override;
    virtual void simulateStep() override;
    virtual void onGUI() override;

    glm::mat4 getWorldFromObj(Body rb);
    glm::mat3 computeInverseOfMomentOfInertia(Body rb);
};