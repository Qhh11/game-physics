#include "Renderer.h"
#include "Scene.h"

// #include "glm/ext.hpp"
// #include "glm/gtx/string_cast.hpp"

#include <imgui.h>

#include <memory>
#include <utility>
#include <vector>

class Scene2 : public Scene
{
  private:
    // Rigid body state
    glm::vec3 xcm;
    glm::vec3 vcm;
    glm::quat r;
    glm::vec3 L;
    glm::mat3 I_0;
    glm::mat3 I_inv;
    glm::vec3 w;

    // Simulation parameters
    float dt;
    float mass;
    glm::vec3 dimensions;
    glm::vec3 F;
    glm::vec3 F_extra;

    float scalar;
    bool A, W, S, D, Q, E;

    void updateRigidBodyState(float h);

    void initializeRigidBody();

  public:
    void init() override;

    void simulateStep() override;

    void onDraw(Renderer &renderer) override;

    void onGUI() override;
};
