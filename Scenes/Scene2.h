#include "Scene.h"
#include <glm/glm.hpp>

class Scene2 : public Scene {
class MassPoint {
private:
    glm::vec3 x;
    glm::vec3 v;
    glm::vec3 a = {0, 0, 0};
    float m;

public:
    MassPoint(glm::vec3 x, glm::vec3 v, float m): x(x), v(v), m(m) {}
    void set_force(glm::vec3 f);
    void update_position_Euler(float dt);
    void update_velocity_Euler(float dt);
    glm::vec3 get_position();
    glm::vec3 get_velocity();
};

class Spring {
private:
    MassPoint& p1;
    MassPoint& p2;
    float rest_length;
    float k;

public:
    Spring(MassPoint& p1, MassPoint& p2, float rest_length, float k): p1(p1), p2(p2), rest_length(rest_length), k(k) {}
    void apply_force();

    glm::vec3 get_position_of_p1();
    glm::vec3 get_position_of_p2();
    glm::vec3 get_color();
    float get_delta_length();
};

private:
    float dt = 0.005;
    MassPoint p0, p1;
    Spring spring;
    float sphere_size = 0.05;

public:
    virtual void onDraw(Renderer &renderer) override;
    virtual void simulateStep() override;
    virtual void onGUI() override;

    Scene2():
        p0(glm::vec3(0, 0, 0), glm::vec3(-1, 0, 0), 10),
        p1(glm::vec3(0, 2, 0), glm::vec3(1, 0, 0), 10),
        spring(p0, p1, 1, 40) {}
};
