//
//  Particle.cpp
//  BachelorThesis
//
//  Created by Eric Wolter on 9/28/12.
//  Copyright (c) 2012 Eric Wolter. All rights reserved.
//

#include "Particle.h"

Particle::Particle() {}
Particle::~Particle() {}

void Particle::init()
{
    restPosition = position;
    restOrientation = orientation;
    predictedPosition = position;
    predictedOrientation = orientation;
    linearVelocity = Vector3f::Zero();
    angularVelocity = Vector3f::Zero();
}

Matrix3f Particle::getMomentMatrix()
{
    Matrix3f radiiMatrix;
    radiiMatrix <<  radii.x() * radii.x(),  0,                      0,
                    0,                      radii.y() * radii.y(),  0,
                    0,                      0,                      radii.z() * radii.z();
    Matrix3f R = orientation.toRotationMatrix();
    Matrix3f moment;
    moment = mass * radiiMatrix * R;
    moment /= 5;
    
    return moment;
}

float Particle::getSqrDistance(Particle *p1, Particle *p2)
{
    return  (p1->position.x() - p2->position.x()) * (p1->position.x() - p2->position.x()) +
            (p1->position.y() - p2->position.y()) * (p1->position.y() - p2->position.y()) +
            (p1->position.z() - p2->position.z()) * (p1->position.z() - p2->position.z());
}

Vector3f Particle::axisQuaternion(Quaternionf quaternion)
{
	float angle = Particle::angleQuaternion(quaternion);
	if (fabs(angle) < 0.01f) // divide by zero -> any normalized axis is valid
	{
		return Vector3f(1.0f, 0.0f, 0.0f);
	}
	else
	{
		float temp = sqrt(1 - quaternion.w() * quaternion.w());
		return Vector3f(quaternion.x() / temp, quaternion.y() / temp, quaternion.z() / temp);
	}
}

float Particle::angleQuaternion(Quaternionf quaternion)
{
	return 2 * acos(quaternion.w());
}