#include "Face.h"
#include <iostream>

Face::Face() {}
Face::~Face() {}

void Face::computeProperties(Matrix3f &covariance, float &mass, Vector3f &centerOfMass)
{
    // printf("Computing properties of face\n");
    Matrix3f canoical_covariance;
    canoical_covariance << 2, 1, 1, 1, 2, 1, 1, 1, 2;
    canoical_covariance /= 120;

    Matrix3f A;
    A << this->vertices.at(0), this->vertices.at(1), this->vertices.at(2);
    // std::cout << "A: " << std::endl;
    // std::cout << A << std::endl;

    covariance = A.determinant() * A * canoical_covariance * A.transpose();
    // std::cout << "covariance: " << std::endl;
    // std::cout << covariance << std::endl;
    float density = 1.0f;
    mass = density * (A.determinant() / 6);
    // std::cout << "mass: " << std::endl;
    // std::cout << mass << std::endl;
    centerOfMass = A.rowwise().sum() / 4;
    // std::cout << "centerOfMass: " << std::endl;
    // std::cout << centerOfMass << std::endl;
}