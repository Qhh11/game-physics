#include "Scene.h"
#include <glm/glm.hpp>
#include <glm/gtx/quaternion.hpp>
#include <vector>

class Scene1 : public Scene
{
  public:
    Scene1();
    void init() override;

  private:
    virtual void onDraw(Renderer &renderer) override;
    virtual void simulateStep() override;
    void simulateOneStep();
    glm::vec3 springForce(const glm::vec3 &p1, const glm::vec3 &p2, float restLength, float stiffness);
    void eulerIntegration(glm::vec3 &pos, glm::vec3 &vel, const glm::vec3 &acc, float dt);
    void midpointIntegration(glm::vec3 &pos0,
                             glm::vec3 &vel0,
                             glm::vec3 &pos1,
                             glm::vec3 &vel1,
                             const glm::vec3 &acc0,
                             const glm::vec3 &acc1,
                             float dt);
    void calculateMidpoint(const glm::vec3 &position,
                           const glm::vec3 &velocity,
                           const glm::vec3 &accel,
                           float deltaTime,
                           glm::vec3 &positionMid,
                           glm::vec3 &velocityMid)
    {
        float halfTimeStep = deltaTime * 0.5f;
        positionMid = position + halfTimeStep * velocity;
        velocityMid = velocity + halfTimeStep * accel;
    }

    glm::vec3 x0, x1;
    glm::vec3 v0, v1;
    glm::vec3 f0, f1;
    float springLength;
    float stiffness;
    float mass;
    float dt;
};
