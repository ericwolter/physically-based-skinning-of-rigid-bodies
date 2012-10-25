//
//  Particle.h
//  BachelorThesis
//
//  Created by Eric Wolter on 9/28/12.
//  Copyright (c) 2012 Eric Wolter. All rights reserved.
//

#ifndef __BachelorThesis__Particle__
#define __BachelorThesis__Particle__

#include <iostream>
#include <vector>
using namespace std;
#include <Eigen/Dense>
using namespace Eigen;

class Particle
{
public:
    Particle();
    ~Particle();
    
    float mass;
    Vector3f radii;
    Vector3f restPosition;
    Vector3f relativePosition;
    Quaternionf restOrientation;
    
    Vector3f position;
    Quaternionf orientation;
    Vector3f linearVelocity;
    Vector3f angularVelocity;
    
    Vector3f predictedPosition;
    Quaternionf predictedOrientation;
    
    Matrix3f getMomentMatrix();
    
    void init();
    static float getSqrDistance(Particle *p1, Particle *p2);
    static Vector3f axisQuaternion(Quaternionf quaternion);
    static float angleQuaternion(Quaternionf quaternion);
};


#endif /* defined(__BachelorThesis__Particle__) */
