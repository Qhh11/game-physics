#include "Scene2.h"
#include <imgui.h>
#include <iostream>

void Scene2::MassPoint::set_force(glm::vec3 f)
{
    a = f / m;
}

void Scene2::MassPoint::update_position_Euler(float dt)
{
    x += v * dt;
}

void Scene2::MassPoint::update_velocity_Euler(float dt)
{
    v += a * dt;
}

glm::vec3 Scene2::MassPoint::get_position()
{
    return x;
}

glm::vec3 Scene2::MassPoint::get_velocity()
{
    return v;
}

void Scene2::Spring::apply_force()
{
    float length = glm::distance(p1.get_position(), p2.get_position());
    glm::vec3 force_on_p1 = -k * (length - rest_length) * (p1.get_position() - p2.get_position()) / length;
    glm::vec3 force_on_p2 = -force_on_p1;
    p1.set_force(force_on_p1);
    p2.set_force(force_on_p2);
}

glm::vec3 Scene2::Spring::get_position_of_p1()
{
    return p1.get_position();
}
glm::vec3 Scene2::Spring::get_position_of_p2()
{
    return p2.get_position();
}

glm::vec3 Scene2::Spring::get_color()
{
    float length = glm::distance(p1.get_position(), p2.get_position());
    float delta_length_ratio = (length - rest_length) / rest_length;
    float color = abs(delta_length_ratio) * 255.f;
    return delta_length_ratio > 0 ? glm::vec3(color, 0, 0) : glm::vec3(0, color, 0);
}

float Scene2::Spring::get_delta_length()
{
    return glm::distance(p1.get_position(), p2.get_position()) - rest_length;
}

void Scene2::onDraw(Renderer &renderer)
{
    renderer.drawWireCube(glm::vec3(0), glm::vec3(5), glm::vec3(1));
    renderer.drawSphere(p0.get_position(), sphere_size);
    renderer.drawSphere(p1.get_position(), sphere_size);
    renderer.drawLine(spring.get_position_of_p1(), spring.get_position_of_p2(), spring.get_color());
}

std::ostream &operator<<(std::ostream &os, const glm::vec3 &vec)
{
    os << "(" << vec.x << ", " << vec.y << ", " << vec.z << ")";
    return os;
}

void Scene2::simulateStep()
{
    spring.apply_force();
    // For test:
    // dt = 0.1;
    // std::cout << "x0: " << p0.get_position() << std::endl;
    // std::cout << "x1: " << p1.get_position() << std::endl;
    // std::cout << "v0: " << p0.get_velocity() << std::endl;
    // std::cout << "v1: " << p1.get_velocity() << std::endl;
    // std::cout << std::endl;

    p0.update_position_Euler(dt);
    p1.update_position_Euler(dt);
    p0.update_velocity_Euler(dt);
    p1.update_velocity_Euler(dt);
}

void Scene2::onGUI()
{
    ImGui::SliderFloat("Time Step", &dt, 0.f, 0.1f);
}