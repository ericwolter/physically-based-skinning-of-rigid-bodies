#include "RigidBody.h"

#include <iostream>

RigidBody::RigidBody() {}
RigidBody::RigidBody(Polyhedron polyhedron, Vector3f worldPosition, Vector4f color, bool fixed)
{
    mesh = polyhedron;

    if (fixed)
    {
        massInverse = 0;
        inertiaTensorInverse = Matrix3f::Zero();
    }
    else
    {
        massInverse = (1 / polyhedron.mass);
        inertiaTensorInverse = polyhedron.inertiaTensorInverse;
    }

    position = worldPosition;
    orientation = Quaternionf(1.0f,0.0f,0.0f,0.0f);
    linearVelocity = Vector3f::Zero();
    linearMomentum = Vector3f::Zero();
    force = Vector3f::Zero();
    torque = Vector3f::Zero();



    glGenBuffers(1, &vertexBufferObject);
    glBindBuffer(GL_ARRAY_BUFFER, vertexBufferObject);
    glBufferData(GL_ARRAY_BUFFER, mesh.vertices.size() * sizeof(mesh.vertices[0]), &mesh.vertices[0], GL_STATIC_DRAW);
    glBindBuffer(GL_ARRAY_BUFFER, 0);

    glGenBuffers(1, &indexBufferObject);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, indexBufferObject);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, mesh.indices.size() * sizeof(mesh.indices[0]), &mesh.indices[0], GL_STATIC_DRAW);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

    float colorBuffer[mesh.vertices.size() * 4];

    for (int i = 0; i < mesh.vertices.size() * 4; i++)
    {
        colorBuffer[i] = color[i % 4];
    }

    glGenBuffers(1, &colorBufferObject);
    glBindBuffer(GL_ARRAY_BUFFER, colorBufferObject);
    glBufferData(GL_ARRAY_BUFFER, sizeof(colorBuffer), colorBuffer, GL_STATIC_DRAW);
    glBindBuffer(GL_ARRAY_BUFFER, 0);

    glGenBuffers(1, &normalBufferObject);
    glBindBuffer(GL_ARRAY_BUFFER, normalBufferObject);
    glBufferData(GL_ARRAY_BUFFER, mesh.normals.size() * sizeof(mesh.normals[0]), &mesh.normals[0], GL_STATIC_DRAW);
    glBindBuffer(GL_ARRAY_BUFFER, 0);

    glGenVertexArrays(1, &vertexArrayObject);
    glBindVertexArray(vertexArrayObject);

    glBindBuffer(GL_ARRAY_BUFFER, vertexBufferObject);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0);

    glBindBuffer(GL_ARRAY_BUFFER, colorBufferObject);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, 0, 0);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, indexBufferObject);

    glBindBuffer(GL_ARRAY_BUFFER, normalBufferObject);
    glEnableVertexAttribArray(2);
    glVertexAttribPointer(2, 4, GL_FLOAT, GL_FALSE, 0 , 0);

    glBindVertexArray(0);

}

RigidBody::~RigidBody() {}

void RigidBody::render(Affine3f cameraTransform, GLuint modelToCameraMatrixUnif)
{
    cameraTransform.translate(position - mesh.centerOfMass);

    glUniformMatrix4fv(modelToCameraMatrixUnif, 1, GL_FALSE, cameraTransform.data());
    glBindVertexArray(vertexArrayObject);
    glDrawElements(GL_TRIANGLES, mesh.indices.size(), GL_UNSIGNED_INT, 0);
    glBindVertexArray(0);
}

void RigidBody::integrate(float dt)
{
    std::cout << "over dt: " << dt << std::endl;
    std::cout << "old momentum: " << linearMomentum << std::endl;
    linearMomentum += force * dt;
    std::cout << "new momentum: " << linearMomentum << std::endl;
    std::cout << "old velocity: " << linearVelocity << std::endl;
    linearVelocity = linearMomentum * massInverse;
    std::cout << "new velocity: " << linearVelocity << std::endl;
    std::cout << "old position: " << position << std::endl;
    position += linearVelocity * dt;
    std::cout << "new position: " << position << std::endl;

    angularMomentum += torque * dt;
    Matrix3f R = orientation.matrix();
    inertiaInverse = R * inertiaTensorInverse * R.transpose();
    angularVelocity = inertiaInverse * angularMomentum;
    float rotSpeed = angularVelocity.norm();
    if (rotSpeed != 0)
    {
        Quaternionf q(AngleAxisf(rotSpeed * dt, (1 / rotSpeed)*angularVelocity));
        orientation *= q;
    }
}

void RigidBody::save()
{
    state[0] = position[0];
    state[1] = position[1];
    state[2] = position[2];
    state[3] = orientation.w();
    state[4] = orientation.x();
    state[5] = orientation.y();
    state[6] = orientation.z();
    state[7] = linearMomentum[0];
    state[8] = linearMomentum[1];
    state[9] = linearMomentum[2];
    state[10] = angularMomentum[0];
    state[11] = angularMomentum[1];
    state[12] = angularMomentum[2];
}

void RigidBody::restore()
{
    position = Vector3f(state[0], state[1], state[2]);
    orientation = Quaternionf(state[3], state[4], state[5], state[6]);
    linearMomentum = Vector3f(state[7], state[8], state[9]);
    angularMomentum = Vector3f(state[10], state[11], state[12]);
}

Vector3f RigidBody::bodyPointToWorld(Vector3f &bodyPoint)
{
    std::cout << "bodyPoint:" << bodyPoint << std::endl;
    Affine3f t = Affine3f::Identity();
    t.translate(position);
    std::cout << "worldPoint:" << t * bodyPoint << std::endl;
    return t * bodyPoint;
}

Vector3f RigidBody::normalToWorld(Vector3f &normal)
{
    Affine3f t = Affine3f::Identity();
    t.translate(position);
    return (t.linear().inverse().transpose() * normal).normalized();
}