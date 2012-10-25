//
//  ParticleGroup.h
//  BachelorThesis
//
//  Created by Eric Wolter on 9/29/12.
//  Copyright (c) 2012 Eric Wolter. All rights reserved.
//

#ifndef __BachelorThesis__ParticleGroup__
#define __BachelorThesis__ParticleGroup__

#include <iostream>
#include <vector>
using namespace std;

#include "Particle.h"

class ParticleGroup
{
public:
    ParticleGroup();
    ~ParticleGroup();

    vector<Particle*> m_particles;
    float groupMass;
    Vector3f restCenterOfMass;
    
    void init();
    void update();
    
    Matrix3f calculateMomentMatrix();
    Vector3f calculateCenterOfMass();
    Matrix3f polarDecomposition(Matrix3f moment);
    
    
};

#endif /* defined(__BachelorThesis__ParticleGroup__) */
