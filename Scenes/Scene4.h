#include "Scene.h"

struct RigidBox {
    float total_mass;
    glm::vec3 x_cm;
    glm::vec3 extent;
    glm::vec4 color;
    glm::mat3 inertia_inversed;
    glm::vec3 velocity;
    glm::quat orientation;
    glm::vec3 angular_momentum;
    glm::vec3 angular_velocity;
    glm::vec3 torque = glm::vec3(0.f);
    glm::vec3 external_force = glm::vec3(0.f);
    glm::vec3 delta_momentum = glm::vec3(0.f);
    
    RigidBox(glm::vec3 center = glm::vec3(0.f),
             glm::vec4 color = glm::vec4(1.f),
             glm::quat orientation = glm::quat(glm::vec3(0.f)),
             float mass = 1.f,
             glm::vec3 extent = glm::vec3(1.f),
             glm::vec3 velocity = glm::vec3(0.f))
        : total_mass(mass),
          x_cm(center),
          extent(extent),
          color(color),
          velocity(velocity),
          orientation(orientation),
          angular_momentum(glm::vec3(0.f)),
          angular_velocity(glm::vec3(0.f)),
          torque(glm::vec3(0.f)) {

        float Ixx = (1.f / 12.f) * total_mass * (extent.y * extent.y + extent.z * extent.z);
        float Iyy = (1.f / 12.f) * total_mass * (extent.x * extent.x + extent.z * extent.z);
        float Izz = (1.f / 12.f) * total_mass * (extent.x * extent.x + extent.y * extent.y);

        glm::mat3 inertia(Ixx, 0.f, 0.f,
                          0.f, Iyy, 0.f,
                          0.f, 0.f, Izz);

        inertia_inversed = glm::inverse(inertia);
    }
};

class Scene4 : public Scene
{
public:
    void init() override;
    void simulateStep() override;
    void onDraw(Renderer &renderer) override;
    // void onGUI() override;
    
private:
    std::vector<RigidBox> boxes = {
        RigidBox(glm::vec3(0.f, 0.f, 0.f), glm::vec4(1.f, 0.f, 0.f, 1.f), glm::quat(glm::vec3(glm::radians(45.f), glm::radians(45.f), glm::radians(45.f))), 0.5f, glm::vec3(0.5f)),
        RigidBox(glm::vec3(0.f, 2.f, 0.f)),
        RigidBox(glm::vec3(0.f, -2.f, 0.f)),
        RigidBox(glm::vec3(0.f, 0.f, 2.f)),
        RigidBox(glm::vec3(0.f, 0.f, -2.f))
    };
    float time_step = 0.1f;
    float restitution = 1.f;
    float force_unit = 0.1f;
    float room_scale_half = 5.f;

    // camera information, from https://github.com/tum-pbs/game-physics-template/blob/main/guide.md
    glm::mat4 camera_mat = glm::mat4(1.f);
    glm::vec3 fwd = glm::vec3(1.f, 0.f, 0.f);
    glm::vec3 right = glm::vec3(0.f, 1.f, 0.f);
    glm::vec3 up = glm::vec3(0.f, 0.f, 1.f);
};