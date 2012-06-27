#ifndef __FACE_H__
#define __FACE_H__

#include <vector>

#include <Eigen/Dense>
using namespace Eigen;

#include "Edge.h"

class Face
{
public:
    Face();
    ~Face();

    std::vector<Vector3f> vertices;
    std::vector<Edge> edges;
    Vector3f normal;

    void computeProperties(Matrix3f &covariance, float &mass, Vector3f &centerOfMass);
};

#endif