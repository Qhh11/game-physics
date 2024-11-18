#include "Scene1.h"

#include <iostream>

Scene1::Scene1()
    : x0(0.0f, 0.0f, 0.0f), x1(0.0f, 2.0f, 0.0f),
      v0(-1.0f, 0.0f, 0.0f), v1(1.0f, 0.0f, 0.0f),
      springLength(1.0f), stiffness(40.0f), mass(10.0f), dt(0.1f) {}

void Scene1::init()
{
    simulateOneStep();
}

void Scene1::eulerIntegration(glm::vec3 &pos, glm::vec3 &vel, const glm::vec3 &acc, float dt)
{
    pos += vel * dt;
    vel += acc * dt;
}

void Scene1::midpointIntegration(glm::vec3 &pos0, glm::vec3 &vel0, glm::vec3 &pos1, glm::vec3 &vel1, const glm::vec3 &acc0, const glm::vec3 &acc1, float dt)
{
    glm::vec3 x0_mid, v0_mid;
    calculateMidpoint(x0, v0, acc0, dt, x0_mid, v0_mid);
    glm::vec3 x1_mid, v1_mid;
    calculateMidpoint(x1, v1, acc1, dt, x1_mid, v1_mid);
    glm::vec3 forceMid0 = springForce(x0_mid, x1_mid, springLength, stiffness);
    glm::vec3 forceMid1 = -forceMid0;
    glm::vec3 accMid0 = forceMid0 / mass;
    glm::vec3 accMid1 = forceMid1 / mass;

    pos0 += v0_mid * dt;
    pos1 += v1_mid * dt;
    vel0 += accMid0 * dt;
    vel1 += accMid1 * dt;
}

glm::vec3 Scene1::springForce(const glm::vec3 &p1, const glm::vec3 &p2, float restLength, float stiffness)
{
    glm::vec3 d = p2 - p1;
    float length = glm::length(d);
    if (length == 0.0f)
        return glm::vec3(0.0f);
    return stiffness * (length - restLength) * d / length;
}

void Scene1::simulateOneStep()
{
    glm::vec3 force0 = springForce(x0, x1, springLength, stiffness);
    glm::vec3 force1 = -force0;
    glm::vec3 acc0 = force0 / mass;
    glm::vec3 acc1 = force1 / mass;

    glm::vec3 x0_euler = x0, v0_euler = v0;
    glm::vec3 x1_euler = x1, v1_euler = v1;
    eulerIntegration(x0_euler, v0_euler, acc0, dt);
    eulerIntegration(x1_euler, v1_euler, acc1, dt);

    std::cout << "Euler Integration:\n";
    std::cout << "x0: (" << x0_euler.x << ", " << x0_euler.y << ", " << x0_euler.z << ")\n";
    std::cout << "x1: (" << x1_euler.x << ", " << x1_euler.y << ", " << x1_euler.z << ")\n";
    std::cout << "v0: (" << v0_euler.x << ", " << v0_euler.y << ", " << v0_euler.z << ")\n";
    std::cout << "v1: (" << v0_euler.x << ", " << v1_euler.y << ", " << v1_euler.z << ")\n";

    glm::vec3 x0_midpoint = x0, v0_midpoint = v0;
    glm::vec3 x1_midpoint = x1, v1_midpoint = v1;

    midpointIntegration(x0_midpoint, v0_midpoint, x1_midpoint, v1_midpoint, acc0, acc1, dt);

    std::cout << "Midpoint Integration:\n";
    std::cout << "x0: (" << x0_midpoint.x << ", " << x0_midpoint.y << ", " << x0_midpoint.z << ")\n";
    std::cout << "x1: (" << x1_midpoint.x << ", " << x1_midpoint.y << ", " << x1_midpoint.z << ")\n";
    std::cout << "v0: (" << v0_midpoint.x << ", " << v0_midpoint.y << ", " << v0_midpoint.z << ")\n";
    std::cout << "v1: (" << v1_midpoint.x << ", " << v1_midpoint.y << ", " << v1_midpoint.z << ")\n";
}

void Scene1::simulateStep() {};
void Scene1::onDraw(Renderer &renderer) {};