//
//  ParticleGroup.cpp
//  BachelorThesis
//
//  Created by Eric Wolter on 9/29/12.
//  Copyright (c) 2012 Eric Wolter. All rights reserved.
//

#include "ParticleGroup.h"
#include <BulletDynamics/btBulletDynamicsCommon.h>

ParticleGroup::ParticleGroup() {}
ParticleGroup::~ParticleGroup() {}

void ParticleGroup::init()
{
    groupMass = 0.0f;
    restCenterOfMass = Vector3f::Zero();
    
    vector<Particle*>::iterator p;
    for (p = m_particles.begin(); p!= m_particles.end(); p++)
    {
        Particle *particle = *p;
        particle->init();
        
        groupMass += particle->mass;
        restCenterOfMass += particle->mass * particle->restPosition;
    }
    restCenterOfMass /= groupMass;
}

void ParticleGroup::update()
{
    cout << "particle group: " << endl;
    Vector3f currentCenterOfMass = calculateCenterOfMass();
    cout << "mass center: " << currentCenterOfMass << endl;
    Matrix3f R = polarDecomposition(calculateMomentMatrix());
    cout << "rotation matrix: " << R << endl;
    
    vector<Particle*>::iterator p;
    for (p = m_particles.begin(); p!= m_particles.end(); p++)
    {
        Particle *particle = *p;
        cout << "particle: " << particle << endl;
        
        Vector3f goalPosition = R * (particle->restPosition - restCenterOfMass) + currentCenterOfMass;
        cout << "old particle position: " << particle->predictedPosition << endl;
        particle->predictedPosition += 0.8 * (goalPosition - particle->predictedPosition);
        cout << "new particle position: " << particle->predictedPosition << endl;
    }
}

Vector3f ParticleGroup::calculateCenterOfMass()
{
    Vector3f centerOfMass = Vector3f::Zero();
    
    vector<Particle*>::iterator p;
    for (p = m_particles.begin(); p!= m_particles.end(); p++)
    {
        Particle *particle = *p;
        
        centerOfMass += particle->mass * particle->predictedPosition;
    }
    
    return centerOfMass /= groupMass;
}

Matrix3f ParticleGroup::calculateMomentMatrix()
{
    Matrix3f moment = Matrix3f::Zero();
    
    vector<Particle*>::iterator p;
    for (p = m_particles.begin(); p!= m_particles.end(); p++)
    {
        Particle *particle = *p;
        Matrix3f pMoment = particle->getMomentMatrix();
        moment += pMoment + particle->mass * particle->predictedPosition * particle->restPosition.transpose();
    }
    
    return moment - groupMass * calculateCenterOfMass() * restCenterOfMass.transpose();
}

Matrix3f ParticleGroup::polarDecomposition(Matrix3f moment)
{
    static const float half = 0.5;
    static const float accurarcy = 0.0001;
    static const int maxiterations= 16;
    
    Matrix3f q = Matrix3f::Identity();
    Vector3f diagonal = moment.diagonal();
    q = moment * (1/sqrt(diagonal.dot(diagonal)));
    double det = q.determinant();
    
    if(fabs(det) < 0.0001) {
        for(int i = 0; i < maxiterations; i++) {
            q = (q + (q.adjoint() * (1/det)).transpose()) * half;
            float ndet = q.determinant();
            if((ndet-det)*(ndet-det)>accurarcy) {
                det = ndet;
            } else {
                break;
            }
        }
        q.row(2) = (q.row(0).cross(q.row(1))).normalized();
        q.row(1) = (q.row(2).cross(q.row(0))).normalized();
        q.row(0) = (q.row(1).cross(q.row(2))).normalized();
        return q;
    } else {
        return q;
    }
}