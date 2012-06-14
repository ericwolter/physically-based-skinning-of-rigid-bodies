#ifndef __POLYHEDRON_H__
#define __POLYHEDRON_H__

#include <vector>
#include <Eigen/Dense>
using namespace Eigen;

#include "Face.h"

class Polyhedron
{
public:
    Polyhedron();
    ~Polyhedron();

    std::vector<Face> faces;

    std::vector<float> vertices;
    std::vector<unsigned int> indices;
    std::vector<float> normals;

    float mass;
    Vector3f centerOfMass;
    Matrix3f inertiaTensor;
    Matrix3f inertiaTensorInverse;

    void computeProperties();
};

#endif