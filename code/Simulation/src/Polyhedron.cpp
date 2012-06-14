#include "Polyhedron.h"
#include <iostream>

using namespace std;

Polyhedron::Polyhedron() {}
Polyhedron::~Polyhedron() {}

void Polyhedron::computeProperties()
{
    this->inertiaTensor = Matrix3f::Zero();
    this->mass = 0.0f;
    this->centerOfMass = Vector3f::Zero();

    Matrix3f covariance = Matrix3f::Zero();

    // printf("Computing properties of polyhedron\n");
    vector<Face>::iterator face;
    for (face = this->faces.begin(); face != this->faces.end(); face++)
    {
        Matrix3f single_covariance;
        float single_mass;
        Vector3f single_centerOfMass;
        (*face).computeProperties(single_covariance, single_mass, single_centerOfMass);

        covariance += single_covariance;
        this->centerOfMass = (this->centerOfMass * this->mass + single_centerOfMass * single_mass) / (this->mass + single_mass);
        this->mass += single_mass;
    }

    // translate to center of mass
    Vector3f offset = (-1) * this->centerOfMass;
    Matrix3f o1 = offset * this->centerOfMass.transpose();
    Matrix3f o2 = this->centerOfMass * offset.transpose();
    Matrix3f o3 = offset * offset.transpose();
    covariance = covariance + this->mass * (o1 + o2 + o3);

    this->inertiaTensor = Matrix3f::Identity() * covariance.trace() - covariance;
    this->inertiaTensorInverse = inertiaTensor.inverse();
    // std::cout << "inertiaTensor: " << std::endl;
    // std::cout << this->inertiaTensor << std::endl;
};
