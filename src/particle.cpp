#include "particle.h"

Particle::Particle() {}
Particle::~Particle() {}

void Particle::init()
{
    restPosition = position;
    restOrientation = orientation;
    predictedPosition = position;
    predictedOrientation = orientation;
    linearVelocity = btVector3(0,0,0);
	angularVelocity = btVector3(0,0,0);
    
    radiiMatrix = btMatrix3x3(radii.x() * radii.x(), 0, 0, 0, radii.y() * radii.y(), 0, 0, 0, radii.z() * radii.z());
}

btMatrix3x3 Particle::getMomentMatrix()
{
    btMatrix3x3 R = btMatrix3x3(orientation);
    
    btScalar tmp = mass/5;
    btMatrix3x3 moment = radiiMatrix * R;
    
    btVector3 row1 = tmp * moment[0];
    btVector3 row2 = tmp * moment[1];
    btVector3 row3 = tmp * moment[2];
    return btMatrix3x3(row1.x(), row1.y(), row1.z(),row2.x(), row2.y(), row2.z(),row3.x(), row3.y(), row3.z());
}

float Particle::getSqrDistance(Particle *p1, Particle *p2)
{
    return  (p1->position.x() - p2->position.x()) * (p1->position.x() - p2->position.x()) +
            (p1->position.y() - p2->position.y()) * (p1->position.y() - p2->position.y()) +
            (p1->position.z() - p2->position.z()) * (p1->position.z() - p2->position.z());
}

btTransform Particle::getPredictedTransform() {
    btTransform transform = btTransform();
    transform.setOrigin(predictedPosition);
    transform.setRotation(predictedOrientation);
    
    return transform;
}
btTransform Particle::getTransform() {
    btTransform transform = btTransform();
    transform.setOrigin(position);
    transform.setRotation(orientation);
    
    return transform;
}

btMatrix3x3 Particle::getInvInertiaTensorWorld() {
    btMatrix3x3 tensor = btMatrix3x3(
        (mass/5)*(radii.y()*radii.y() + radii.z()*radii.z()),0,0,
        0,(mass/5)*(radii.x()*radii.x() + radii.z()*radii.z()),0,
        0,0,(mass/5)*(radii.x()*radii.x() + radii.y()*radii.y()));
    
    return tensor.inverse();
}
