#include "Scene1.h"

#include <iostream>

Scene1::Scene1()
    : x_cm(0.0f, 0.0f, 0.0f), x_world(-0.3, -0.5, -0.25), linear_veolcity(0.0f, 0.0f, 0.0f), ext_f(1.0, 1.0, 0.0),
      ext_f_pos(0.3, 0.5, 0.25), mass(2.0f), dt(2.0f), width(1.0f), height(0.5f), depth(0.6f)
{
    glm::vec3 rotation_axis(0.0f, 0.0f, 1.0f);
    float rotation_angle = glm::radians(90.0f);
    orientation = glm::angleAxis(rotation_angle, rotation_axis);
    inertia_tensor = computeCuboidInertiaTensor(width, height, depth, mass);
    angular_velocity = glm::vec3(0.0f);
    angular_momentum = glm::vec3(0.0f);
}

void Scene1::init()
{
    simulateOneStep();
}

glm::mat3 Scene1 ::computeCuboidInertiaTensor(float width, float height, float depth, float mass)
{
    float ixx = (1.0f / 12.0f) * mass * (height * height + depth * depth);
    float iyy = (1.0f / 12.0f) * mass * (width * width + height * height);
    float izz = (1.0f / 12.0f) * mass * (width * width + depth * depth);

    return glm::mat3(glm::vec3(ixx, 0.0f, 0.0f), glm::vec3(0.0f, iyy, 0.0f), glm::vec3(0.0f, 0.0f, izz));
}

void printMatrix(const glm::mat3 &matrix, const std::string &name)
{
    std::cout << name << ":\n";
    for (int i = 0; i < 3; ++i)
    {
        std::cout << "[ ";
        for (int j = 0; j < 3; ++j)
        {
            float value = glm::abs(matrix[i][j]) < 1e-6f ? 0.0f : matrix[i][j];
            std::cout << value << " ";
        }
        std::cout << "]\n";
    }
}

void printQuaternion(const glm::quat &quaternion, const std::string &name)
{
    std::cout << name << ":\n";
    std::cout << "x: " << quaternion.x << "\n";
    std::cout << "y: " << quaternion.y << "\n";
    std::cout << "z: " << quaternion.z << "\n";
    std::cout << "w: " << quaternion.w << "\n";
}

void Scene1::simulateOneStep()
{
    glm::vec3 torque = glm::cross(ext_f_pos - x_cm, ext_f);
    // std::cout << "Torque: (" << torque.x << ", " << torque.y << ", " << torque.z << ")\n";

    x_cm += dt * linear_veolcity;
    std::cout << "Updated mass point position (x_cm): (" << x_cm.x << ", " << x_cm.y << ", " << x_cm.z << ")\n";

    linear_veolcity += dt * ext_f / mass;
    std::cout << "Updated Linear Velocity: (" << linear_veolcity.x << ", " << linear_veolcity.y << ", "
              << linear_veolcity.z << ")\n";

    if (glm::length(angular_velocity) > 1e-6f)
    {
        glm::quat angular_velocity_quat(0.0f, angular_velocity.x, angular_velocity.y, angular_velocity.z);
        glm::quat delta_orientation = 0.5f * dt * angular_velocity_quat * orientation;
        orientation = glm::normalize(orientation + delta_orientation); // idk if this step is needed
    }
    printQuaternion(orientation, "Updated Orientation (Quaternion)");

    angular_momentum += dt * torque;
    // std::cout << "Updated Angular Momentum: (" << angular_momentum.x << ", " << angular_momentum.y << ", "
    //           << angular_momentum.z << ")\n";

    // printMatrix(inertia_tensor, "ori Inertia Tensor");
    glm::mat3 rotation_matrix = glm::toMat3(orientation);
    inertia_tensor = glm::inverse(inertia_tensor) * glm::transpose(rotation_matrix);
    inertia_tensor = rotation_matrix * inertia_tensor;
    // printMatrix(inertia_tensor, "updated Inertia Tensor");

    angular_velocity = inertia_tensor * angular_momentum;
    std::cout << "Angular Velocity: (" << angular_velocity.x << ", " << angular_velocity.y << ", " << angular_velocity.z
              << ")\n";

    glm::vec3 x_world_pos = x_cm + rotation_matrix * x_world;
    std::cout << "World Position (x_world_pos): (" << x_world_pos.x << ", " << x_world_pos.y << ", " << x_world_pos.z
              << ")\n";
    glm::vec3 v_world = linear_veolcity + glm::cross(angular_velocity, x_world);
    std::cout << "World Velocity (v_world): (" << v_world.x << ", " << v_world.y << ", " << v_world.z << ")\n";
}

void Scene1::simulateStep() {};
void Scene1::onDraw(Renderer &renderer) {};