#ifndef __CONTACT_H__
#define __CONTACT_H__

#include "RigidBody.h"
#include <Eigen/Dense>
using namespace Eigen;

class Contact
{
public:
    Contact();
    ~Contact();

    RigidBody *a;
    RigidBody *b;

    Vector3f point;
    Vector3f normal;

    Vector3f edgeDirectionA;
    Vector3f edgeDirectionB;

    bool isVertexFaceContact;

    bool isColliding();
    void resolve(float epsilon);
};

#endif