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
    linearVelocity = Vector3f::Zero();
    linearMomentum = Vector3f::Zero();
    force = Vector3f::Zero();
    force << 0.0f, 0.0f, 0.0f;

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
    std::cout << "position_before: " << position << std::endl;
    // integrate position
    linearVelocity = linearMomentum / mesh.mass;
    std::cout << "linearVelocity: " << linearVelocity << std::endl;
    position += linearVelocity * dt;
    std::cout << "position_after: " << position << std::endl;

    // integrate orientation
    Matrix3f R = orientation.matrix();
    inertiaInverse = R * mesh.inertiaTensorInverse * R.transpose();
    angularVelocity = inertiaInverse * angularMomentum;
    float rotSpeed = angularVelocity.norm();
    if (rotSpeed != 0)
    {
        Quaternionf q(AngleAxisf(rotSpeed * dt, (1 / rotSpeed)*angularVelocity));
        orientation *= q;
    }

    // integrate linear momentum
    std::cout << "linearMomentum_before: " << linearMomentum << std::endl;
    std::cout << "force: " << force << std::endl;
    linearMomentum += force * dt;
    std::cout << "linearMomentum_after: " << linearMomentum << std::endl;

    // integrate angular momentum
    angularMomentum += torque * dt;
}