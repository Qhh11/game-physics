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
    glm::mat3 computeCuboidInertiaTensor(float width, float height, float depth, float mass);

    glm::vec3 x_cm;
    glm::vec3 x_world;
    glm::vec3 linear_veolcity;
    glm::vec3 ext_f;
    glm::vec3 ext_f_pos;
    float mass;
    float dt;
    float width;
    float height;
    float depth;
    glm::quat orientation;
    glm::vec3 angular_velocity;
    glm::mat3 inertia_tensor;
    glm::vec3 angular_momentum;
};
