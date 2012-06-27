#ifndef __EDGE_H__
#define __EDGE_H__

#include <vector>
#include <Eigen/Dense>

using namespace Eigen;

class Edge
{
public:
    Edge(Vector3f vertex1, Vector3f vertex2);
    ~Edge();

    std::vector<Vector3f> vertices;
    Vector3f direction;
};

#endif