#include "Scene2.h"

#include <imgui.h>

void Scene2::init()
{
    initializeRigidBody();
}

void Scene2::simulateStep()
{
    updateRigidBodyState(dt);
}

void Scene2::onDraw(Renderer &renderer)
{
    renderer.drawWireCube(glm::vec3(0), glm::vec3(5.0f), glm::vec3(1));

    renderer.drawCube(xcm, r, dimensions, glm::vec4(1.0f, 0.0f, 0.0f, 1.0f));
}

void Scene2::onGUI()
{
    ImGui::Text("Use ASWDQE to add force");

    ImGui::SliderFloat("Delta t", &dt, 0.00001f, 0.1f);

    ImGui::SliderFloat("Keyboard Force", &scalar, 0.00001f, 10.0f);
}

void Scene2::initializeRigidBody()
{
    xcm = glm::vec3(0.0f);
    vcm = glm::vec3(0.0f);

    r = glm::angleAxis(glm::radians(90.0f), glm::vec3(0.0f, 0.0f, 1.0f));

    dimensions = glm::vec3(1.0f, 0.6f, 0.5f);
    mass = 2.0f;

    I_0 = glm::mat3(0.0f);
    I_0[0][0] = mass / 12.0f * (dimensions.y * dimensions.y + dimensions.z * dimensions.z);
    I_0[1][1] = mass / 12.0f * (dimensions.x * dimensions.x + dimensions.z * dimensions.z);
    I_0[2][2] = mass / 12.0f * (dimensions.x * dimensions.x + dimensions.y * dimensions.y);

    L = glm::vec3(0.0f);
    I_inv = glm::inverse(I_0);
    w = glm::vec3(0.0f);

    F = glm::vec3(1.0f, 1.0f, 0.0f);

    // dt = 2.0f;
    dt = 0.01f;

    scalar = 3.0f;
}

void Scene2::updateRigidBodyState(float h)
{
    F_extra = glm::vec3(0.0f);
    if (ImGui::IsKeyDown(ImGuiKey_W))
        F_extra += scalar * glm::vec3(0.0f, 0.0f, 1.0f);
    if (ImGui::IsKeyDown(ImGuiKey_S))
        F_extra += scalar * glm::vec3(0.0f, 0.0f, -1.0f);
    if (ImGui::IsKeyDown(ImGuiKey_A))
        F_extra += scalar * glm::vec3(1.0f, 0.0f, 0.0f);
    if (ImGui::IsKeyDown(ImGuiKey_D))
        F_extra += scalar * glm::vec3(1.0f, 0.0f, 0.0f);
    if (ImGui::IsKeyDown(ImGuiKey_Q))
        F_extra += scalar * glm::vec3(0.0f, 1.0f, 0.0f);
    if (ImGui::IsKeyDown(ImGuiKey_E))
        F_extra += scalar * glm::vec3(0.0f, 1.0f, 0.0f);

    glm::vec3 x_extra(-0.3f, -0.5f, -0.25f);
    glm::vec3 x(0.3f, 0.5f, 0.25f);

    glm::vec3 force = F + F_extra;
    glm::vec3 q = glm::cross(x, F) + glm::cross(x_extra, F_extra);
    // std::cout<<"Torque: "<<q.x<<" "<<q.y<<" "<<q.z<<std::endl;

    xcm += h * vcm;
    vcm += h * force / mass;
    // std::cout<<"xcm: "<<xcm.x<<" "<<xcm.y<<" "<<xcm.z<<std::endl;
    // std::cout<<"vcm: "<<vcm.x<<" "<<vcm.y<<" "<<vcm.z<<std::endl;

    glm::quat w_quat(0, w.x, w.y, w.z);
    r += h / 2.0f * (w_quat * r);
    r = glm::normalize(r);
    // std::cout<<"Quaternion: "<<r.x<<" "<<r.y<<" "<<r.z<<" "<<r.w<<std::endl;

    L += h * q;
    // std::cout<<"Angular Momentum: "<<L.x<<" "<<L.y<<" "<<L.z<<std::endl;
    glm::mat3 Rot = glm::mat3_cast(r);
    I_inv = Rot * glm::inverse(I_0) * glm::transpose(Rot);
    // std::cout<<"Inertia Tensor I_0"<<std::endl;
    // std::cout<<I_0[0][0]<<" "<<I_0[0][1]<<" "<<I_0[0][2]<<" "<<std::endl;
    // std::cout<<I_0[1][0]<<" "<<I_0[1][1]<<" "<<I_0[1][2]<<" "<<std::endl;
    // std::cout<<I_0[2][0]<<" "<<I_0[2][1]<<" "<<I_0[2][2]<<" "<<std::endl;

    // std::cout<<"Inertia Tensor I_inv"<<std::endl;
    // std::cout<<I_inv[0][0]<<" "<<I_inv[0][1]<<" "<<I_inv[0][2]<<" "<<std::endl;
    // std::cout<<I_inv[1][0]<<" "<<I_inv[1][1]<<" "<<I_inv[1][2]<<" "<<std::endl;
    // std::cout<<I_inv[2][0]<<" "<<I_inv[2][1]<<" "<<I_inv[2][2]<<" "<<std::endl;
    w = I_inv * L;
    // std::cout<<"Angular Velocity: "<<w.x <<" "<<w.y<<" "<<w.z<<std::endl;

    // assert(false);
}