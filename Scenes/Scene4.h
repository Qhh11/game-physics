#include "Renderer.h"
#include "Scene.h"


#include <memory>
#include <utility>
#include <vector>

enum class IntegrationType { Euler, MidPoint, LeapFrog };

class MassPoint;

class Spring : public std::enable_shared_from_this<Spring>
{
    friend MassPoint;

  public:
    Spring(float rest_length, float stiffness);

    void onDraw(Renderer &renderer);
    void connect(std::shared_ptr<MassPoint> mp1, std::shared_ptr<MassPoint> mp2);
    std::pair<std::shared_ptr<MassPoint>, std::shared_ptr<MassPoint>> get_mass_points();

    void apply_elastic_force();

  private:
    const float m_rest_length;
    const float m_stiffness;

    std::pair<std::shared_ptr<MassPoint>, std::shared_ptr<MassPoint>> m_mass_points;
};

class MassPoint
{
    friend Spring;

  public:
    MassPoint(float mass);
    MassPoint(float mass, float px, float py, float pz);
    MassPoint(float mass, float px, float py, float pz, float vx, float vy, float vz);

    void onDraw(Renderer &renderer);
    glm::vec3 m_p, m_v, m_a, m_f;
    bool m_allow_gravity = false;

    void update_acceleration();

  private:
    const float m_mass;

    std::vector<std::shared_ptr<Spring>> m_springs;

    void apply_gravity();
    void apply_force(glm::vec3 f);
};

class Scene4 : public Scene
{
  public:
    void init() override;

    void simulateStep() override;

    void onDraw(Renderer &renderer) override;

    void onGUI() override;

  private:
    std::vector<std::shared_ptr<MassPoint>> m_mass_points;
    std::vector<std::shared_ptr<Spring>> m_springs;
    float m_dt;
    int m_mode;
    bool m_allow_collision = false;
    bool m_allow_gravity = false;
    const float m_cube_size = 5;

    template <IntegrationType> void update_position();
};

template <> inline void Scene4::update_position<IntegrationType::Euler>()
{
    for (auto s : m_springs) {
        s->apply_elastic_force();
    }

    for (auto mp : m_mass_points) {
        mp->m_p += m_dt * mp->m_v;

        if (m_allow_collision) {
            if (mp->m_p.x > m_cube_size / 2)
                mp->m_p.x = m_cube_size / 2, mp->m_v.x = -mp->m_v.x;
            if (mp->m_p.y > m_cube_size / 2)
                mp->m_p.y = m_cube_size / 2, mp->m_v.y = -mp->m_v.y;
            if (mp->m_p.z > m_cube_size / 2)
                mp->m_p.z = m_cube_size / 2, mp->m_v.z = -mp->m_v.z;
            if (mp->m_p.x < -m_cube_size / 2)
                mp->m_p.x = -m_cube_size / 2, mp->m_v.x = -mp->m_v.x;
            if (mp->m_p.y < -m_cube_size / 2)
                mp->m_p.y = -m_cube_size / 2, mp->m_v.y = -mp->m_v.y;
            if (mp->m_p.z < -m_cube_size / 2)
                mp->m_p.z = -m_cube_size / 2, mp->m_v.z = -mp->m_v.z;
        }
    }

    for (auto mp : m_mass_points) {
        mp->update_acceleration();
    }

    for (auto mp : m_mass_points) {
        mp->m_v += m_dt * mp->m_a;
    }

    for (auto mp : m_mass_points) {
        if (m_allow_collision) {
            if (mp->m_p.x > m_cube_size / 2)
                mp->m_p.x = m_cube_size / 2, mp->m_v.x = -mp->m_v.x;
            if (mp->m_p.y > m_cube_size / 2)
                mp->m_p.y = m_cube_size / 2, mp->m_v.y = -mp->m_v.y;
            if (mp->m_p.z > m_cube_size / 2)
                mp->m_p.z = m_cube_size / 2, mp->m_v.z = -mp->m_v.z;
            if (mp->m_p.x < -m_cube_size / 2)
                mp->m_p.x = -m_cube_size / 2, mp->m_v.x = -mp->m_v.x;
            if (mp->m_p.y < -m_cube_size / 2)
                mp->m_p.y = -m_cube_size / 2, mp->m_v.y = -mp->m_v.y;
            if (mp->m_p.z < -m_cube_size / 2)
                mp->m_p.z = -m_cube_size / 2, mp->m_v.z = -mp->m_v.z;
        }
    }
}

template <> inline void Scene4::update_position<IntegrationType::MidPoint>()
{
    std::vector<glm::vec3> tmp_p, tmp_v;

    for (auto s : m_springs) {
        s->apply_elastic_force();
    }

    for (auto mp : m_mass_points) {
        mp->update_acceleration();
    }

    for (auto mp : m_mass_points) {
        tmp_p.push_back(mp->m_p);
        mp->m_p += 0.5f * m_dt * mp->m_v;
    }

    for (auto mp : m_mass_points) {
        tmp_v.push_back(mp->m_v);
        mp->m_v += 0.5f * m_dt * mp->m_a;
    }

    for (auto s : m_springs) {
        s->apply_elastic_force();
    }

    for (auto mp : m_mass_points) {
        mp->update_acceleration();
    }

    for (int i = 0; i < m_mass_points.size(); i++) {
        m_mass_points[i]->m_p = tmp_p[i] + m_dt * m_mass_points[i]->m_v;
    }

    for (int i = 0; i < m_mass_points.size(); i++) {
        m_mass_points[i]->m_v = tmp_v[i] + m_dt * m_mass_points[i]->m_a;
    }

    for (int i = 0; i < m_mass_points.size(); i++) {
        if (m_allow_collision) {
            if (m_mass_points[i]->m_p.x > m_cube_size / 2)
                m_mass_points[i]->m_p.x = m_cube_size / 2, m_mass_points[i]->m_v.x = -m_mass_points[i]->m_v.x;
            if (m_mass_points[i]->m_p.y > m_cube_size / 2)
                m_mass_points[i]->m_p.y = m_cube_size / 2, m_mass_points[i]->m_v.y = -m_mass_points[i]->m_v.y;
            if (m_mass_points[i]->m_p.z > m_cube_size / 2)
                m_mass_points[i]->m_p.z = m_cube_size / 2, m_mass_points[i]->m_v.z = -m_mass_points[i]->m_v.z;
            if (m_mass_points[i]->m_p.x < -m_cube_size / 2)
                m_mass_points[i]->m_p.x = -m_cube_size / 2, m_mass_points[i]->m_v.x = -m_mass_points[i]->m_v.x;
            if (m_mass_points[i]->m_p.y < -m_cube_size / 2)
                m_mass_points[i]->m_p.y = -m_cube_size / 2, m_mass_points[i]->m_v.y = -m_mass_points[i]->m_v.y;
            if (m_mass_points[i]->m_p.z < -m_cube_size / 2)
                m_mass_points[i]->m_p.z = -m_cube_size / 2, m_mass_points[i]->m_v.z = -m_mass_points[i]->m_v.z;
        }
    }
}