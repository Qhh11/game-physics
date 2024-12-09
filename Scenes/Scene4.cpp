#include "Scene4.h"

#include "util/CollisionDetection.h"
using namespace collisionTools;

#include <imgui.h>

void Scene4::init() {
    // for (auto& box: boxes) {
    //     box.external_force += glm::vec3(1.f, 1.f, 0.f);
    //     box.torque += glm::cross(glm::vec3(0.3f, 0.5f, 0.25f) - box.x_cm, glm::vec3(1.f, 1.f, 0.f));
    // }
}

// From task sheet
glm::mat4 get_world_from_obj(RigidBox rb) {
    glm::mat4 rotation = glm::toMat4(rb.orientation);
    glm::mat4 scale = glm::scale(glm::mat4(1.f), rb.extent);
    glm::mat4 translation = glm::translate(glm::mat4(1.f), rb.x_cm);
    return translation * rotation * scale;
}

void Scene4::simulateStep() {
    // check collision among boxes; apply via momentum changes if any
    for (int i = 0; i < boxes.size(); ++i) {
        for (int j = i + 1; j < boxes.size(); ++j) {
            CollisionInfo info = checkCollisionSAT(get_world_from_obj(boxes[i]), get_world_from_obj(boxes[j]));
            if (info.isColliding) {
                glm::vec3 point_relative_to_A = info.collisionPointWorld - boxes[i].x_cm;
                glm::vec3 point_relative_to_B = info.collisionPointWorld - boxes[j].x_cm;
                glm::vec3 velocity_of_collision_point_on_A = boxes[i].velocity + glm::cross(boxes[i].angular_velocity, point_relative_to_A);
                glm::vec3 velocity_of_collision_point_on_B = boxes[j].velocity + glm::cross(boxes[j].angular_velocity, point_relative_to_B);
                glm::vec3 velocity_of_point_on_A_relative_to_point_on_B = velocity_of_collision_point_on_A - velocity_of_collision_point_on_B;
                if (glm::dot(velocity_of_point_on_A_relative_to_point_on_B, info.normalWorld) < 0) { // Colliding contact
                    float J = - (1 + restitution) * glm::dot(velocity_of_point_on_A_relative_to_point_on_B, info.normalWorld) /
                                (1.f / boxes[i].total_mass + 1.f / boxes[j].total_mass +
                                 glm::dot(glm::cross(boxes[i].inertia_inversed * glm::cross(point_relative_to_A, info.normalWorld), point_relative_to_A) +
                                          glm::cross(boxes[j].inertia_inversed * glm::cross(point_relative_to_B, info.normalWorld), point_relative_to_B),
                                          info.normalWorld));

                    boxes[i].delta_momentum += J * info.normalWorld;
                    boxes[j].delta_momentum -= J * info.normalWorld;
                    boxes[i].angular_momentum += glm::cross(point_relative_to_A, J * info.normalWorld);
                    boxes[j].angular_momentum -= glm::cross(point_relative_to_B, J * info.normalWorld);
                }
            }
        }
    }

    for (auto& box: boxes) {
        // TODO: apply force via mouse:
        // box.external_force += ...
        // box.torque += glm::cross(p_relative_to_cm, f)
        box.x_cm += time_step * box.velocity;

        box.velocity += time_step * box.external_force / box.total_mass;
        box.velocity += box.delta_momentum / box.total_mass;

        box.orientation += time_step * 0.5f * glm::quat(0, box.angular_velocity) * box.orientation;
        box.orientation = glm::normalize(box.orientation);
        box.angular_momentum += time_step * box.torque;

        // std::cout<<"Inertia Tensor box.inertia_inversed"<<std::endl;
        // auto tmp_mat = glm::inverse(box.inertia_inversed);
        // std::cout<<tmp_mat[0][0]<<" "<<tmp_mat[0][1]<<" "<<tmp_mat[0][2]<<" "<<std::endl; std::cout<<tmp_mat[1][0]<<" "<<tmp_mat[1][1]<<" "<<tmp_mat[1][2]<<" "<<std::endl; std::cout<<tmp_mat[2][0]<<" "<<tmp_mat[2][1]<<""<<tmp_mat[2][2]<<" "<<std::endl;

        auto rot = glm::mat3_cast(box.orientation);
        box.inertia_inversed = rot * box.inertia_inversed * glm::transpose(rot);
        box.angular_velocity = box.inertia_inversed * box.angular_momentum;

        // clear
        box.torque = glm::vec3(0.f);
        box.external_force = glm::vec3(0.f);
        box.delta_momentum = glm::vec3(0.f);

        // std::cout << "updated mass point position: " << box.x_cm.x << " " << box.x_cm.y << " " << box.x_cm.z << std::endl;
        // std::cout << "updated linear velocity: " << box.velocity.x << " " << box.velocity.y << " " << box.velocity.z << std::endl;
        // std::cout << "updated orientation: " << box.orientation.x << " " << box.orientation.y << " " << box.orientation.z << " " << box.orientation.w << std::endl;
        // std::cout << "Angular Momentum: " << box.angular_momentum.x << " " << box.angular_momentum.y << " " << box.angular_momentum.z << std::endl;
        // std::cout<<"Inertia Tensor box.inertia_inversed"<<std::endl;
        // std::cout<<box.inertia_inversed[0][0]<<" "<<box.inertia_inversed[0][1]<<" "<<box.inertia_inversed[0][2]<<" "<<std::endl; std::cout<<box.inertia_inversed[1][0]<<" "<<box.inertia_inversed[1][1]<<" "<<box.inertia_inversed[1][2]<<" "<<std::endl; std::cout<<box.inertia_inversed[2][0]<<" "<<box.inertia_inversed[2][1]<<""<<box.inertia_inversed[2][2]<<" "<<std::endl;

        // std::cout<<"Angular Velocity: "<<box.angular_velocity.x<<" "<<box.angular_velocity.y<<" "<<box.angular_velocity.z<<std::endl;
        // assert(false);
    }
}

void Scene4::onDraw(Renderer &renderer) {
    renderer.drawWireCube(glm::vec3(0), glm::vec3(5), glm::vec3(1));
    
    for (auto& box: boxes) {
        renderer.drawCube(box.x_cm, box.orientation, box.extent, box.color);
    }
}
