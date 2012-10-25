//
//  CombinedBody.h
//  BachelorThesis
//
//  Created by Eric Wolter on 9/28/12.
//  Copyright (c) 2012 Eric Wolter. All rights reserved.
//

#ifndef __BachelorThesis__CombinedBody__
#define __BachelorThesis__CombinedBody__

#include <iostream>
#include <vector>
using namespace std;

#include <BulletDynamics/btBulletDynamicsCommon.h>

#include "ParticleGroup.h"
#include "Particle.h"

class CombinedBody
{
public:
    CombinedBody(btRigidBody *rigidBody);
    ~CombinedBody();
    
    Vector3f restPosition;
    
    vector<ParticleGroup*> m_particleGroups;
    vector<Particle*> m_particles;
    vector<Particle*> m_attachedParticles;
    vector<Particle*> m_outerParticles;
    
    btRigidBody *m_rigidBody;
    void applyGravity(float timeStep);
    void integrateVelocities(float timeStep);
    void predictIntegratedTransform(float timeStep, btTransform &predictedTransform);
    void integrateTransforms(float timeStep);
    
private:
    void addAttachedParticle(Particle *p);
    void addOuterParticle(Particle *p);
    
    bool vectorIsEqualWithinMargin(Vector3f v1, Vector3f v2, float margin);
};

#endif /* defined(__BachelorThesis__CombinedBody__) */
