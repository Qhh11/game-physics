#include "Scene4.h"

#include <imgui.h>

Spring::Spring(float rest_length, float stiffness) : m_rest_length{rest_length}, m_stiffness{stiffness} {}

void Spring::connect(std::shared_ptr<MassPoint> mp1, std::shared_ptr<MassPoint> mp2)
{
    m_mass_points = std::make_pair(mp1, mp2);
    mp1->m_springs.push_back(shared_from_this());
    mp2->m_springs.push_back(shared_from_this());
}

std::pair<std::shared_ptr<MassPoint>, std::shared_ptr<MassPoint>> Spring::get_mass_points()
{
    return m_mass_points;
}

void Spring::onDraw(Renderer &renderer)
{
    renderer.drawLine(m_mass_points.first->m_p, m_mass_points.second->m_p, {255, 255, 255});
}

void Spring::apply_elastic_force()
{
    float current_length = glm::length(m_mass_points.second->m_p - m_mass_points.first->m_p);
    float displacement = current_length - m_rest_length;
    glm::vec3 force_1_2 =
        m_stiffness * displacement * glm::normalize(m_mass_points.second->m_p - m_mass_points.first->m_p);
    glm::vec3 force_2_1 = -force_1_2;
    m_mass_points.first->apply_force(force_1_2);
    m_mass_points.second->apply_force(force_2_1);
}

MassPoint::MassPoint(float mass, float px, float py, float pz, float vx, float vy, float vz)
    : m_mass{mass}, m_p{px, py, pz}, m_v{vx, vy, vz}, m_a{}, m_f{}
{
}

MassPoint::MassPoint(float mass, float px, float py, float pz) : MassPoint(mass, px, py, pz, 0, 0, 0) {}

MassPoint::MassPoint(float mass) : MassPoint(mass, 0, 0, 0) {}

void MassPoint::onDraw(Renderer &renderer)
{
    renderer.drawSphere(m_p, 0.05);
}

void MassPoint::apply_gravity()
{
    if (m_allow_gravity)
        m_f.z -= 9.81f * m_mass;
}

void MassPoint::apply_force(glm::vec3 f)
{
    m_f += f;
}

void MassPoint::update_acceleration()
{
    apply_gravity();
    m_a = m_f / m_mass;
}

void Scene4::init()
{
    m_dt = 0.0001;

    m_mass_points.emplace_back(std::make_shared<MassPoint>(10, 0, 0, 0, -1, 0, 0));
    m_mass_points.emplace_back(std::make_shared<MassPoint>(10, 0, 2, 0, 1, 0, 0));

    m_springs.emplace_back(std::make_shared<Spring>(1, 40));

    m_springs[0]->connect(m_mass_points[0], m_mass_points[1]);
}

// int cnt = 0;
void Scene4::simulateStep()
{
    // if (cnt >= 1) {
    //     exit(0);
    // }
    // update_position<IntegrationType::MidPoint>();
    // cnt++;

    // std::cout<< "x0 " << m_mass_points[0]->m_p.x << " "<< m_mass_points[0]->m_p.y << " "<< m_mass_points[0]->m_p.z <<
    // std::endl; std::cout<< "x1 " << m_mass_points[1]->m_p.x << " "<< m_mass_points[1]->m_p.y << " "<<
    // m_mass_points[1]->m_p.z << std::endl; std::cout<< "v0 " << m_mass_points[0]->m_v.x << " "<<
    // m_mass_points[0]->m_v.y << " "<< m_mass_points[0]->m_v.z << std::endl; std::cout<< "v1 " <<
    // m_mass_points[1]->m_v.x << " "<< m_mass_points[1]->m_v.y << " "<< m_mass_points[1]->m_v.z << std::endl;

    if (m_mode == 0) {
        update_position<IntegrationType::Euler>();
    } else {
        update_position<IntegrationType::MidPoint>();
    }
}

void Scene4::onDraw(Renderer &renderer)
{
    renderer.drawWireCube(glm::vec3(0), glm::vec3(m_cube_size), glm::vec3(1));

    for (auto mp : m_mass_points) {
        mp->onDraw(renderer);
    }

    for (auto s : m_springs) {
        s->onDraw(renderer);
    }
}

void Scene4::onGUI()
{
    ImGui::SliderFloat("Delta t", &m_dt, 0.00001f, 0.001f);

    if (ImGui::RadioButton("Euler", &m_mode, 0)) {
        std::cout << "Set to Euler" << std::endl;
    }

    if (ImGui::RadioButton("MidPoint", &m_mode, 1)) {
        std::cout << "Set to MidPoint" << std::endl;
    }

    if (ImGui::Checkbox("Allow Collision", &m_allow_collision)) {
        std::cout << "Allow Collision: " << (m_allow_collision ? "True" : "False") << std::endl;
    }

    if (ImGui::Checkbox("Allow Gravity", &m_allow_gravity)) {
        std::cout << "Allow Gravity: " << (m_allow_gravity ? "True" : "False") << std::endl;

        for (auto mp : m_mass_points) {
            mp->m_allow_gravity = true;
        }
    }
}