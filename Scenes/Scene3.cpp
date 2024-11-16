#include "Scene3.h"
#include <imgui.h>

void Scene3::onDraw(Renderer &renderer)
{
    renderer.drawWireCube(glm::vec3(0), glm::vec3(5), glm::vec3(1));

    renderer.drawSphere(point1.position, point1.radius, point1.color);
    renderer.drawSphere(point2.position, point2.radius, point2.color);
    renderer.drawLine(point1.position, point2.position, point1.color, point2.color);
}

void Scene3::simulateStep()
{
    auto len = glm::distance(point1.position, point2.position);
    auto f_0 = computeSpringForce(point1.position, point2.position, len);
    auto a1_0 = f_0 / point1.mass;
    auto a2_0 = -f_0 / point2.mass;

    auto half_x1 = point1.position + (timeStep / 2) * point1.velocity;
    auto half_x2 = point2.position + (timeStep / 2) * point2.velocity;

    auto half_v1 = point1.velocity + (timeStep / 2) * a1_0;
    auto half_v2 = point2.velocity + (timeStep / 2) * a2_0;

    auto half_len = glm::distance(half_x1, half_x2);
    auto half_f = computeSpringForce(half_x1, half_x2, half_len);
    auto half_a1 = half_f / point1.mass;
    auto half_a2 = -half_f / point2.mass;

    point1.position = point1.position + timeStep * half_v1;
    point2.position = point2.position + timeStep * half_v2;

    point1.velocity = point1.velocity + timeStep * half_a1;
    point2.velocity = point2.velocity + timeStep * half_a2;
}

glm::vec3 Scene3::computeSpringForce(glm::vec3 p1, glm::vec3 p2, float len)
{
    return -K * (len - L) * (p1 - p2) / len;
}

void Scene3::onGUI()
{
    ImGui::SliderFloat("Time Step", &timeStep, 0.f, 0.01f);
}