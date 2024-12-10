#include "Scene3.h"
#include <imgui.h>
#include "util/CollisionDetection.h"
#include <glm/gtx/quaternion.hpp>
#include <glm/gtx/string_cast.hpp>

glm::mat4 Scene3::getWorldFromObj(Body rb)
{
    mat4 rotationMatrix = glm::toMat4(rb.ori);
    mat4 scaleMatrix = glm::scale(mat4(1), rb.extent);
    mat4 translationMatrix = glm::translate(mat4(1), rb.pos);
    return translationMatrix * rotationMatrix * scaleMatrix;
}

glm::mat3 Scene3::computeInverseOfMomentOfInertia(Body rb)
{
    mat3 rot = glm::toMat3(rb.ori);
    return rot * rb.inver_mo_inertia * glm::transpose(rot);
}

void Scene3::init()
{
    body1.inver_mo_inertia = computeInverseOfMomentOfInertia(body1);
    body2.inver_mo_inertia = computeInverseOfMomentOfInertia(body2);

    body1.ang_vec = body1.inver_mo_inertia * body1.mo_ang;
    body2.ang_vec = body2.inver_mo_inertia * body2.mo_ang;
}

void Scene3::onDraw(Renderer &renderer)
{
    renderer.drawWireCube(glm::vec3(0), glm::vec3(5), glm::vec3(1));
    
    renderer.drawCube(body1.pos, body1.ori, body1.extent, body1.color);
    renderer.drawCube(body2.pos, body2.ori, body2.extent, body2.color);
}

void Scene3::simulateStep()
{
    CollisionInfo cf = collisionTools::checkCollisionSAT(getWorldFromObj(body1), getWorldFromObj(body2));
    if (cf.isColliding) {
        auto x1_rel = cf.collisionPointWorld - body1.pos;
        auto x2_rel = cf.collisionPointWorld - body2.pos;
        auto v_rel = body1.lin_vec + glm::cross(body1.ang_vec, x1_rel) - 
        body2.lin_vec - glm::cross(body2.ang_vec, x2_rel);
        if (glm::dot(v_rel, cf.normalWorld) <= 0) {
            body2.color = vec4(0, 0, 1, 1);

            float J = -(1 + c) * glm::dot(v_rel, cf.normalWorld) / 
            (1/body1.mass + 1/body2.mass + 
            glm::dot((glm::cross(body1.inver_mo_inertia * (glm::cross(x1_rel, cf.normalWorld)), x1_rel) + 
            glm::cross(body2.inver_mo_inertia * (glm::cross(x2_rel, cf.normalWorld)), x2_rel)), cf.normalWorld));

            body1.lin_vec += J * cf.normalWorld / body1.mass;
            body2.lin_vec -= J * cf.normalWorld / body2.mass;

            body1.mo_ang += glm::cross(x1_rel, J * cf.normalWorld);
            body2.mo_ang -= glm::cross(x2_rel, J * cf.normalWorld);

            body1.ang_vec = body1.inver_mo_inertia * body1.mo_ang;
            body2.ang_vec = body2.inver_mo_inertia * body2.mo_ang;
        }
    }

    body1.pos += timeStep * body1.lin_vec;
    body2.pos += timeStep * body2.lin_vec;

    body1.ori = glm::normalize(body1.ori + (timeStep / 2) * glm::quat(0, body1.ang_vec.x, body1.ang_vec.y, body1.ang_vec.z) * body1.ori);
    body2.ori = glm::normalize(body2.ori + (timeStep / 2) * glm::quat(0, body2.ang_vec.x, body2.ang_vec.y, body2.ang_vec.z) * body2.ori);

    body1.inver_mo_inertia = computeInverseOfMomentOfInertia(body1);
    body2.inver_mo_inertia = computeInverseOfMomentOfInertia(body2);

    body1.ang_vec = body1.inver_mo_inertia * body1.mo_ang;
    body2.ang_vec = body2.inver_mo_inertia * body2.mo_ang;
}

void Scene3::onGUI()
{
    ImGui::SliderFloat("Time Step", &timeStep, 0.f, 0.1f);
    ImGui::SliderFloat("Coefficient of Restitution", &c, 0.f, 1.f);
}